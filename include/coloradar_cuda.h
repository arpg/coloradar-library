#ifndef COLORADAR_CUDA_H
#define COLORADAR_CUDA_H

#include "coloradar_tools.h"
#include "cuda_kernels.h"

#include <cuda_runtime.h>
#include <cuComplex.h>
#include <cufft.h>
#include <vector>
#include <complex>


namespace coloradar {

    class RadarProcessor {
    protected:
        const double blackmanA0;
        const double blackmanA1;
        const double blackmanA2;

        int* virtualArrayMap;
        cuDoubleComplex* couplingSignature;
        cuDoubleComplex* frequencyCalibMatrix;
        cuDoubleComplex* phaseCalibMatrix;
        cufftHandle rangePlan;
        cufftHandle dopplerPlan;
        cufftHandle anglePlan;
        double* rangeWindowFunc;
        double* dopplerWindowFunc;
        double* azimuthWindowFunc;
        double* elevationWindowFunc;

        std::vector<cuDoubleComplex> toCudaComplex(const std::vector<std::complex<double>>& array);
        double blackman(const int& idx, const int& size);
        void initWindowFunc(double*& windowFunc, const int& size);
        void initFftPlans();

    public:
        const RadarConfig* config;

        RadarProcessor(
            const RadarConfig* config,
            const double& blackmanParamA0 = 0.42,
            const double& blackmanParamA1 = 0.5,
            const double& blackmanParamA2 = 0.08
        );
        ~RadarProcessor();

        std::vector<float> cubeToHeatmap(
            const std::vector<int16_t>& datacube,
            const bool& applyCollapseDoppler = false,
            const bool& removeAntennaCoupling = false,
            const bool& applyPhaseFrequencyCalib = false
        );
    };

    template<typename T>
    void copyToGpu(const std::vector<T>& source, T*& dest) {
        cudaError_t err = cudaMemcpy(dest, source.data(), sizeof(T) * source.size(), cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            std::cerr << "CUDA memcpy error:  " << cudaGetErrorString(err) << std::endl;
        }
        cudaDeviceSynchronize();
    }
}

#endif
