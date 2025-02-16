#include <fstream>
#include <sstream>
#include <thread>
#include <math.h>
#include <Eigen/Core>
#include <mutex>

#include "coloradar_cuda.h"


double coloradar::RadarProcessor::blackman(const int& idx, const int& size) {
    double angle = 2.0 * M_PI * idx / size;
    return blackmanA0 - blackmanA1 * cos(angle) + blackmanA2 * cos(2 * angle);
}

std::vector<cuDoubleComplex> coloradar::RadarProcessor::toCudaComplex(const std::vector<std::complex<double>>& array) {
    std::vector<cuDoubleComplex> cudaArray(array.size());
    for (size_t i = 0; i < array.size(); ++i) {
        cudaArray[i] = make_cuDoubleComplex(array[i].real(), array[i].imag());
    }
    return cudaArray;
}

void coloradar::RadarProcessor::initWindowFunc(double*& windowFunc, const int& size) {
    std::vector<double> windowFuncLocal(size);
    for (size_t i = 0; i < size; ++i) {
        windowFuncLocal[i] = blackman(i, size);
    }
    cudaMalloc(&windowFunc, sizeof(double) * size);
    coloradar::copyToGpu(windowFuncLocal, windowFunc);
}


void coloradar::RadarProcessor::initFftPlans() {
    int rank = 1, angleRank = 2;
    int nRange[1] = {config->numRangeBins}, nDoppler[1] = {config->numDopplerBins}, nAngle[2] = {config->numAzimuthBeams, config->numElevationBeams};
    int howmanyRange = config->numTxAntennas * config->numRxAntennas * config->numDopplerBins;
    int howmanyDoppler = config->numTxAntennas * config->numRxAntennas * config->numPosRangeBins;
    int howmanyAngle = config->numPosRangeBins * config->numDopplerBins;
    int rangeDist = config->numRangeBins, dopplerDist = 1, angleDist = config->numAzimuthBeams * config->numElevationBeams;
    int rangeStride = 1, dopplerStride = config->numPosRangeBins * config->numTxAntennas * config->numRxAntennas, angleStride = 1;
    int *rangeEmbed = nRange, *dopplerEmbed = nDoppler, *angleEmbed = nAngle;
    cufftPlanMany(&rangePlan, rank, nRange, rangeEmbed, rangeStride, rangeDist, rangeEmbed, rangeStride, rangeDist, CUFFT_Z2Z , howmanyRange);
    cufftPlanMany(&dopplerPlan, rank, nDoppler, dopplerEmbed, dopplerStride, dopplerDist, dopplerEmbed, dopplerStride, dopplerDist, CUFFT_Z2Z, howmanyDoppler);
    cufftPlanMany(&anglePlan, angleRank, nAngle, angleEmbed, angleStride, angleDist, angleEmbed, angleStride, angleDist, CUFFT_Z2Z, howmanyAngle);
    cudaDeviceSynchronize();
}

coloradar::RadarProcessor::RadarProcessor(const RadarConfig* radarConfig, const double& blackmanParamA0, const double& blackmanParamA1, const double& blackmanParamA2)
        : config(radarConfig), blackmanA0(blackmanParamA0), blackmanA1(blackmanParamA1), blackmanA2(blackmanParamA2) {

    cudaMalloc(&virtualArrayMap, sizeof(int) * 4 * config->numVirtualElements);
    coloradar::copyToGpu(config->virtualArrayMap, virtualArrayMap);
    cudaDeviceSynchronize();

    cudaMalloc(&couplingSignature, sizeof(cuDoubleComplex) * config->numPosRangeBins * config->numTxAntennas * config->numRxAntennas);
    coloradar::copyToGpu(toCudaComplex(config->couplingCalibMatrix), couplingSignature);

    cudaMalloc(&frequencyCalibMatrix, sizeof(cuDoubleComplex) * config->numRangeBins * config->numTxAntennas * config->numRxAntennas);
    coloradar::copyToGpu(toCudaComplex(config->frequencyCalibMatrix), frequencyCalibMatrix);

    cudaMalloc(&phaseCalibMatrix, sizeof(cuDoubleComplex) * config->numTxAntennas * config->numRxAntennas);
    coloradar::copyToGpu(toCudaComplex(config->phaseCalibMatrix), phaseCalibMatrix);

    initWindowFunc(rangeWindowFunc, config->numRangeBins);
    initWindowFunc(dopplerWindowFunc, config->numDopplerBins);
    initWindowFunc(azimuthWindowFunc, config->azimuthApertureLen);
    initWindowFunc(elevationWindowFunc, config->elevationApertureLen);
    initFftPlans();
}

coloradar::RadarProcessor::~RadarProcessor() {
    cudaFree(virtualArrayMap);
    cudaFree(couplingSignature);
    cudaFree(frequencyCalibMatrix);
    cudaFree(phaseCalibMatrix);
    cudaFree(rangeWindowFunc);
    cudaFree(dopplerWindowFunc);
    cudaFree(azimuthWindowFunc);
    cudaFree(elevationWindowFunc);
    cufftDestroy(rangePlan);
    cufftDestroy(dopplerPlan);
    cufftDestroy(anglePlan);
}


std::vector<float> coloradar::RadarProcessor::cubeToHeatmap(
    const std::vector<int16_t>& datacube,
    const bool& applyCollapseDoppler,
    const bool& removeAntennaCoupling,
    const bool& applyPhaseFrequencyCalib
) {
    int16_t* datacubeGpu;
    cudaMalloc(&datacubeGpu, sizeof(int16_t) * datacube.size());
    coloradar::copyToGpu(datacube, datacubeGpu);

    // Range FFT
    cuDoubleComplex* rangeFftData;
    cudaMalloc(&rangeFftData, sizeof(cuDoubleComplex) * config->numRangeBins * config->numDopplerBins * config->numTxAntennas * config->numRxAntennas);
    setFrameData(config->numRangeBins, config->numDopplerBins, config->numTxAntennas, config->numRxAntennas, datacubeGpu, rangeFftData);
    if (applyPhaseFrequencyCalib)
        applyPhaseFreqCal(config->numRangeBins, config->numDopplerBins, config->numTxAntennas, config->numRxAntennas, rangeFftData, frequencyCalibMatrix, phaseCalibMatrix);
    applyWindow(config->numRangeBins, 1, config->numRangeBins, config->numTxAntennas * config->numRxAntennas * config->numDopplerBins, rangeWindowFunc, rangeFftData);
    cufftExecZ2Z(rangePlan, rangeFftData, rangeFftData, CUFFT_FORWARD);
    cudaDeviceSynchronize();
    cudaFree(datacubeGpu);

    // Doppler FFT
    cuDoubleComplex* dopplerFftData;
    cudaMalloc(&dopplerFftData, sizeof(cuDoubleComplex) * config->numPosRangeBins * config->numDopplerBins * config->numTxAntennas * config->numRxAntennas);
    removeNegSpectrum(config->numRangeBins, config->numPosRangeBins, config->numDopplerBins, config->numTxAntennas * config->numRxAntennas, rangeFftData, dopplerFftData);
    if (removeAntennaCoupling)
        removeCoupling(config->numPosRangeBins, config->numDopplerBins, config->numTxAntennas * config->numRxAntennas, dopplerFftData, couplingSignature);
    applyWindow(1, config->numPosRangeBins * config->numTxAntennas * config->numRxAntennas, config->numDopplerBins, config->numTxAntennas * config->numRxAntennas * config->numPosRangeBins, dopplerWindowFunc, dopplerFftData);
    cufftExecZ2Z(dopplerPlan, dopplerFftData, dopplerFftData, CUFFT_FORWARD);
    cudaDeviceSynchronize();
    cudaFree(rangeFftData);

    // Angle FFT
    cuDoubleComplex* angleFftData;
    cudaMalloc(&angleFftData, sizeof(cuDoubleComplex) * config->numPosRangeBins * config->numDopplerBins * config->numAzimuthBeams * config->numElevationBeams);
    cudaMemset(angleFftData, 0, sizeof(cuDoubleComplex) * config->numAzimuthBeams * config->numElevationBeams * config->numPosRangeBins * config->numDopplerBins);
    rearrangeData(config->numPosRangeBins, config->numDopplerBins, config->numTxAntennas, config->numRxAntennas, config->numAzimuthBeams, config->numElevationBeams, config->numVirtualElements, virtualArrayMap, azimuthWindowFunc, elevationWindowFunc, dopplerFftData, angleFftData);
    cufftExecZ2Z(anglePlan, angleFftData, angleFftData, CUFFT_FORWARD);
    cudaDeviceSynchronize();
    cudaFree(dopplerFftData);

    // Rearrange data for creating heatmap, including rearranging the doppler, azimuth, and elevation dimensions so zero frequency is centered (fftshift in Matlab and SciPy)
    float* heatmapGpu;
    std::vector<float> heatmap;
    cudaMalloc(&heatmapGpu, sizeof(float) * config->numPosRangeBins * config->numDopplerBins * config->numAzimuthBeams * config->numElevationBeams);
    assembleMsg(config->numPosRangeBins, config->numDopplerBins, config->numAzimuthBeams, config->numElevationBeams, angleFftData, heatmapGpu);
    cudaFree(angleFftData);

    if (!applyCollapseDoppler) {
        heatmap.resize(config->numAngles * config->numPosRangeBins * config->numDopplerBins);
        cudaMemcpy(&heatmap[0], heatmapGpu, sizeof(float) * config->numPosRangeBins * config->numDopplerBins * config->numAngles, cudaMemcpyDefault);
        cudaFree(heatmapGpu);
    } else {
        float* heatmapCollapsed;
        cudaMalloc(&heatmapCollapsed, sizeof(float) * 2 * config->numPosRangeBins * config->numAzimuthBeams * config->numElevationBeams);
        collapseDoppler(config->numPosRangeBins, config->numDopplerBins, config->numAngles, config->dopplerBinWidth, heatmapGpu, heatmapCollapsed);
        heatmap.resize(2 * config->numAngles * config->numPosRangeBins);
        cudaMemcpy(&heatmap[0], heatmapCollapsed, sizeof(float) * 2 * config->numPosRangeBins * config->numAngles, cudaMemcpyDefault);
        cudaFree(heatmapGpu);
        cudaFree(heatmapCollapsed);
    }
    return heatmap;
}
