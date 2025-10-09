#ifndef RADAR_CONFIGS_H
#define RADAR_CONFIGS_H

#include "utils/utils.h"


namespace coloradar {


class RadarConfig;
struct HeatmapTransformResult {
    std::shared_ptr<std::vector<float>> heatmap;
    std::shared_ptr<RadarConfig> newConfig;
};


class RadarConfig {
protected:
    // internal params
    int numAzimuthBeams_ = 0;
    int numElevationBeams_ = 0;
    int azimuthApertureLen_ = 0;
    int elevationApertureLen_ = 0;
    int numAngles_ = 0;
    int numVirtualElements_ = 0;
    std::vector<int> virtualArrayMap_ = {};
    std::vector<float> azimuthAngles_ = {};
    std::vector<float> elevationAngles_ = {};
    double dopplerBinWidth_ = 0.0;
    bool hasDoppler_ = true;

    // heatmap params
    int numRangeBins_ = 0;
    int numPosRangeBins_ = 0;
    int numElevationBins_ = 0;
    int numAzimuthBins_ = 0;
    double rangeBinWidth_ = 0;
    std::vector<float> azimuthBins_ = {};
    std::vector<float> elevationBins_ = {};

    // antenna params
    double designFrequency_ = 0.0;
    int numTxAntennas_ = 0;
    int numRxAntennas_ = 0;
    std::vector<pcl::PointXY> txCenters_ = {};
    std::vector<pcl::PointXY> rxCenters_ = {};

    // waveform params
    int numAdcSamplesPerChirp_ = 0;
    int numChirpsPerFrame_ = 0;
    int adcSampleFrequency_ = 0;
    double startFrequency_ = 0.0;
    double idleTime_ = 0.0;
    double adcStartTime_ = 0.0;
    double rampEndTime_ = 0.0;
    double frequencySlope_ = 0.0;

    // calibration params
    int numDopplerBins_ = 0;
    std::vector<std::complex<double>> couplingCalibMatrix_ = {};

    // phase frequency params
    int calibAdcSampleFrequency_ = 0;
    double calibFrequencySlope_ = 0.0;
    std::vector<std::complex<double>> frequencyCalibMatrix_ = {};
    std::vector<std::complex<double>> phaseCalibMatrix_ = {};

    // static variables
    pcl::PointCloud<RadarPoint>::Ptr pointcloudTemplate_ = nullptr;

    // methods
    virtual void init(const std::filesystem::path& calibDir) = 0;
    void initAntennaParams(const std::filesystem::path& antennaCfgFile);
    void initHeatmapParams(const std::filesystem::path& heatmapCfgFile);
    void initWaveformParams(const std::filesystem::path& waveformCfgFile);
    void initCouplingParams(const std::filesystem::path& couplingCfgFile);
    void initPhaseFrequencyParams(const std::filesystem::path& phaseFrequencyCfgFile);
    void initInternalParams();
    void setNumRangeBins(const int& num);  // abstracted as a method because distinction between numRangeBins and numPosRangeBins may be unclear

public:
    static constexpr double c = 299792458; // speed of light in m/s

    virtual std::shared_ptr<RadarConfig> clone() const = 0;

    const int& numAzimuthBeams() const { return numAzimuthBeams_; }
    const int& numElevationBeams() const { return numElevationBeams_; }
    const int& azimuthApertureLen() const { return azimuthApertureLen_; }
    const int& elevationApertureLen() const { return elevationApertureLen_; }
    const int& numAngles() const { return numAngles_; }
    const int& numVirtualElements() const { return numVirtualElements_; }
    const std::vector<int>& virtualArrayMap() const { return virtualArrayMap_; }
    const std::vector<float>& azimuthAngles() const { return azimuthAngles_; }
    const std::vector<float>& elevationAngles() const { return elevationAngles_; }
    const double& dopplerBinWidth() const { return dopplerBinWidth_; }
    const bool& hasDoppler() const { return hasDoppler_; }
    const int& numElevationBins() const { return numElevationBins_; }
    const int& numAzimuthBins() const { return numAzimuthBins_; }
    const int& numRangeBins() const { return numRangeBins_; }
    const int& numPosRangeBins() const { return numPosRangeBins_; }
    const double& rangeBinWidth() const { return rangeBinWidth_; }
    const std::vector<float>& azimuthBins() const { return azimuthBins_; }
    const std::vector<float>& elevationBins() const { return elevationBins_; }
    const double& designFrequency() const { return designFrequency_; }
    const int& numTxAntennas() const { return numTxAntennas_; }
    const int& numRxAntennas() const { return numRxAntennas_; }
    const std::vector<pcl::PointXY>& txCenters() const { return txCenters_; }
    const std::vector<pcl::PointXY>& rxCenters() const { return rxCenters_; }
    const int& numAdcSamplesPerChirp() const { return numAdcSamplesPerChirp_; }
    const int& numChirpsPerFrame() const { return numChirpsPerFrame_; }
    const int& adcSampleFrequency() const { return adcSampleFrequency_; }
    const double& startFrequency() const { return startFrequency_; }
    const double& idleTime() const { return idleTime_; }
    const double& adcStartTime() const { return adcStartTime_; }
    const double& rampEndTime() const { return rampEndTime_; }
    const double& frequencySlope() const { return frequencySlope_; }
    const int& numDopplerBins() const { return numDopplerBins_; }
    const std::vector<std::complex<double>>& couplingCalibMatrix() const { return couplingCalibMatrix_; }
    const int& calibAdcSampleFrequency() const { return calibAdcSampleFrequency_; }
    const double& calibFrequencySlope() const { return calibFrequencySlope_; }
    const std::vector<std::complex<double>>& frequencyCalibMatrix() const { return frequencyCalibMatrix_; }
    const std::vector<std::complex<double>>& phaseCalibMatrix() const { return phaseCalibMatrix_; }
    const pcl::PointCloud<RadarPoint>::Ptr& pointcloudTemplate() const { return pointcloudTemplate_; }

    RadarConfig(const int& nAzimuthBeams = 1, const int& nElevationBeams = 1) : numAzimuthBeams_(nAzimuthBeams), numElevationBeams_(nElevationBeams) {};
    RadarConfig(const RadarConfig& other) = default;
    virtual ~RadarConfig() = default;

    Json::Value toJson() const;
    void fromJson(const Json::Value& root);          
    void fromJson(const std::string& jsonString);

    const int& nRangeBins() const;  // abstracted as a method because distinction between numRangeBins and numPosRangeBins may be unclear
    float maxRange() const;
    const int heatmapSize() const;
    
    int clipAzimuthMaxBin(const int& azMaxBin) const;
    int clipElevationMaxBin(const int& elMaxBin) const;
    int clipRangeMaxBin(const int& rangeMaxBin) const;
    float clipRange(const float& range) const;
    float azimuthIdxToFovDegrees(const int& azMaxBin) const;
    float elevationIdxToFovDegrees(const int& elMaxBin) const;
    float rangeIdxToRange(const int& rangeMaxBin) const;
    int horizontalFovToAzimuthIdx(const float& horizontalFov) const;
    int verticalFovToElevationIdx(const float& verticalFov) const;
    int rangeToRangeIdx(const float& range) const;

    HeatmapTransformResult clipHeatmap(const std::shared_ptr<std::vector<float>>& heatmap, int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin) const;
    HeatmapTransformResult clipHeatmap(const std::shared_ptr<std::vector<float>>& heatmap, float horizontalFov, float verticalFov, float range) const;
    HeatmapTransformResult collapseHeatmapElevation(const std::shared_ptr<std::vector<float>>& image, const float& elevationMinMeters = -100.0, const float& elevationMaxMeters = 100.0) const;
    HeatmapTransformResult removeDoppler(const std::shared_ptr<std::vector<float>>& image) const;
    // HeatmapTransformResult swapHeatmapDimensions(const std::shared_ptr<std::vector<float>>& heatmap) const;
    pcl::PointCloud<RadarPoint>::Ptr heatmapToPointcloud(const std::shared_ptr<std::vector<float>>& heatmap, const double intensityThreshold = 0.0) const;
    void precomputePointcloudTemplate();  // call to ensure correct heatmap->pointcloud conversion if config parameters change
};


class SingleChipConfig : public RadarConfig {
public:
    SingleChipConfig() = default;
    SingleChipConfig(const std::filesystem::path& calibDir, const int& nAzimuthBeams = 64, const int& nElevationBeams = 8);
    explicit SingleChipConfig(const Json::Value& root) { fromJson(root); }
    std::shared_ptr<RadarConfig> clone() const override {
        return std::make_shared<SingleChipConfig>(*this);
    }

protected:
    void init(const std::filesystem::path& calibDir) override;
};


class CascadeConfig : public RadarConfig {
public:
    CascadeConfig() = default;
    CascadeConfig(const std::filesystem::path& calibDir, const int& nAzimuthBeams = 128, const int& nElevationBeams = 32);
    explicit CascadeConfig(const Json::Value& root) { fromJson(root); }
    std::shared_ptr<RadarConfig> clone() const override {
        return std::make_shared<CascadeConfig>(*this);
    }

protected:
    void init(const std::filesystem::path& calibDir) override;
};

}

#endif
