#ifndef RADAR_CONFIGS_H
#define RADAR_CONFIGS_H

#include "utils.h"


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

    void setNumRangeBins(const int& num);

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
    bool hasDoppler = true;

    RadarConfig(const int& nAzimuthBeams = 1, const int& nElevationBeams = 1) : numAzimuthBeams(nAzimuthBeams), numElevationBeams(nElevationBeams) {};
    RadarConfig(const RadarConfig& other) = default;
    Json::Value toJson() const;

    const int& nRangeBins() const;
    float maxRange() const;
    int clipAzimuthMaxBin(const int& azMaxBin);
    int clipElevationMaxBin(const int& elMaxBin);
    int clipRangeMaxBin(const int& rangeMaxBin);
    float clipRange(const float& range);
    float azimuthIdxToFovDegrees(const int& azMaxBin);
    float elevationIdxToFovDegrees(const int& elMaxBin);
    float rangeIdxToRange(const int& rangeMaxBin);
    int horizontalFovToAzimuthIdx(const float& horizontalFov);
    int verticalFovToElevationIdx(const float& verticalFov);
    int rangeToRangeIdx(const float& range);

    std::vector<float> clipHeatmap(const std::vector<float>& heatmap, int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin, bool updateConfig = true);
    std::vector<float> clipHeatmap(const std::vector<float>& heatmap, float horizontalFov, float verticalFov, float range, bool updateConfig = true);
    std::vector<float> collapseHeatmapElevation(const std::vector<float>& image, const float& elevationMinMeters = -100.0, const float& elevationMaxMeters = 100.0, bool updateConfig = true);
    std::vector<float> removeDoppler(const std::vector<float>& image, bool updateConfig = true);
    std::vector<float> swapHeatmapDimensions(const std::vector<float>& heatmap);
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

pcl::PointCloud<RadarPoint> heatmapToPointcloud(const std::vector<float>& heatmap, RadarConfig* config, const float& intensityThresholdPercent = 0.0);

}

#endif
