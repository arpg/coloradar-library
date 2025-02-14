#ifndef RADAR_CONFIGS_H
#define RADAR_CONFIGS_H

#include "internal.h"


namespace coloradar {

class RadarConfig {
protected:
    virtual void init(const std::filesystem::path& calibDir) = 0;
    void initAntennaParams(const std::filesystem::path& antennaCfgFile);
    void initHeatmapParams(const std::filesystem::path& heatmapCfgFile);
    void initWaveformParams(const std::filesystem::path& waveformCfgFile);
    void initCouplingParams(const std::filesystem::path& couplingCfgFile);
    void initPhaseFrequencyParams(const std::filesystem::path& phaseFrequencyCfgFile);
    void initInternalParams();

public:
    static constexpr double c = 299792458; // speed of light in m/s

    // heatmap params
    int numRangeBins;
    int numPosRangeBins;
    int numElevationBins;
    int numAzimuthBins;
    double rangeBinWidth;
    std::vector<float> azimuthBins;
    std::vector<float> elevationBins;

    // antenna params
    double designFrequency;
    int numTxAntennas;
    int numRxAntennas;
    std::vector<pcl::PointXY> txCenters;
    std::vector<pcl::PointXY> rxCenters;

    // waveform params
    int numAdcSamplesPerChirp;
    int numChirpsPerFrame;
    int adcSampleFrequency;
    double startFrequency;
    double idleTime;
    double adcStartTime;
    double rampEndTime;
    double frequencySlope;

    // calibration params
    int numDopplerBins;
    std::vector<std::complex<double>> couplingCalibMatrix;

    // phase frequency params
    int calibAdcSampleFrequency;
    double calibFrequencySlope;
    std::vector<std::complex<double>> frequencyCalibMatrix;
    std::vector<std::complex<double>> phaseCalibMatrix;

    // internal params
    int numAzimuthBeams;
    int numElevationBeams;
    int azimuthApertureLen;
    int elevationApertureLen;
    int numAngles;
    int numVirtualElements;
    std::vector<int> virtualArrayMap;
    std::vector<float> azimuthAngles;
    std::vector<float> elevationAngles;
    double dopplerBinWidth;

    RadarConfig(const int& nAzimuthBeams = 1, const int& nElevationBeams = 1);
    Json::Value toJson() const;

    const int& nRangeBins() const;
    const float& maxRange() const;
    int clipAzimuthMaxBin(const int& azMaxBin);
    int clipElevationMaxBin(const int& elMaxBin);
    float clipRange(const float& range);
};

class SingleChipConfig : public RadarConfig {
public:
    SingleChipConfig() = default;
    SingleChipConfig(const std::filesystem::path& calibDir, const int& nAzimuthBeams = 64, const int& nElevationBeams = 8);

protected:
    void init(const std::filesystem::path& calibDir) override;
};

class CascadeConfig : public RadarConfig {
public:
    CascadeConfig() = default;
    CascadeConfig(const std::filesystem::path& calibDir, const int& nAzimuthBeams = 128, const int& nElevationBeams = 32);

protected:
    void init(const std::filesystem::path& calibDir) override;
};

}

#endif
