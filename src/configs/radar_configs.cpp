#include "configs/radar_configs.h"


namespace coloradar {


void RadarConfig::precomputePointcloudTemplate() {
    const size_t totalSize = numElevationBins_ * numAzimuthBins_ * nRangeBins();
    pointcloudTemplate_.reset(new pcl::PointCloud<RadarPoint>);
    pointcloudTemplate_->resize(totalSize);

    std::vector<float> cosAzimuths(numAzimuthBins_), sinAzimuths(numAzimuthBins_);
    for (int azIdx = 0; azIdx < numAzimuthBins_; ++azIdx) {
        cosAzimuths[azIdx] = std::cos(azimuthBins_[azIdx]);
        sinAzimuths[azIdx] = std::sin(azimuthBins_[azIdx]);
    }
    std::vector<float> cosElevations(numElevationBins_), sinElevations(numElevationBins_);
    for (int elIdx = 0; elIdx < numElevationBins_; ++elIdx) {
        cosElevations[elIdx] = std::cos(elevationBins_[elIdx]);
        sinElevations[elIdx] = std::sin(elevationBins_[elIdx]);
    }

    size_t pointIdx = 0;
    for (int elIdx = 0; elIdx < numElevationBins_; ++elIdx) {
        for (int azIdx = 0; azIdx < numAzimuthBins_; ++azIdx) {
            for (int rangeIdx = 0; rangeIdx < nRangeBins(); ++rangeIdx, ++pointIdx) {
                float range = rangeIdx * rangeBinWidth_;
                RadarPoint& point = (*pointcloudTemplate_)[pointIdx];
                point.x = range * cosElevations[elIdx] * cosAzimuths[azIdx];
                point.y = range * cosElevations[elIdx] * sinAzimuths[azIdx];
                point.z = range * sinElevations[elIdx];
                point.intensity = 0.0f;
                point.doppler = 0.0f;
            }
        }
    }
}


const int RadarConfig::heatmapSize() const {
    return numElevationBins_ * numAzimuthBins_ * nRangeBins() * (hasDoppler_ ? 2 : 1);
}


pcl::PointCloud<RadarPoint>::Ptr RadarConfig::heatmapToPointcloud(const std::shared_ptr<std::vector<float>>& heatmap, const double intensityThreshold) const {
    const size_t expectedCloudSize = numElevationBins_ * numAzimuthBins_ * nRangeBins();
    if (heatmap->size() != heatmapSize()) throw std::runtime_error("Heatmap size mismatch. Expected: " + std::to_string(heatmapSize()) + ", got: " + std::to_string(heatmap->size()));
    if (!pointcloudTemplate_ || pointcloudTemplate_->size() != expectedCloudSize) throw std::runtime_error("Pointcloud template size mismatch. Expected: " + std::to_string(expectedCloudSize) + ", got: " + std::to_string(pointcloudTemplate_->size()));

    pcl::PointCloud<RadarPoint>::Ptr outputCloud(new pcl::PointCloud<RadarPoint>);
    outputCloud->reserve(pointcloudTemplate_->size());
    size_t heatmapIdx = 0;

    float intensity, doppler = 0.0f;
    for (size_t pointIdx = 0; pointIdx < pointcloudTemplate_->size(); ++pointIdx) {
        intensity = heatmap->at(heatmapIdx++);
        if (hasDoppler_) doppler = heatmap->at(heatmapIdx++);
        if (intensity >= intensityThreshold) {
            RadarPoint point = (*pointcloudTemplate_)[pointIdx];
            point.intensity = intensity;
            point.doppler = doppler;
            outputCloud->push_back(point);
        }
    }
    return outputCloud;
}


HeatmapTransformResult RadarConfig::clipHeatmap(
    const std::shared_ptr<std::vector<float>>& heatmap,
    int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin) const
{
    HeatmapTransformResult result;
    result.heatmap = heatmap;
    result.newConfig = this->clone();
    azimuthMaxBin   = clipAzimuthMaxBin(azimuthMaxBin);
    elevationMaxBin = clipElevationMaxBin(elevationMaxBin);
    rangeMaxBin     = clipRangeMaxBin(rangeMaxBin);
    if (azimuthMaxBin == numAzimuthBins_ / 2 && elevationMaxBin == numElevationBins_ / 2 && rangeMaxBin == nRangeBins() - 1) return result;

    int azimuthBinLimit   = numAzimuthBins_ / 2 - 1;
    int azimuthLeftBin    = azimuthBinLimit - azimuthMaxBin;
    int azimuthRightBin   = azimuthBinLimit + azimuthMaxBin + 1;

    int elevationBinLimit = numElevationBins_ / 2 - 1;
    int elevationLeftBin  = elevationBinLimit - elevationMaxBin;
    int elevationRightBin = elevationBinLimit + elevationMaxBin + 1;

    result.heatmap = std::make_shared<std::vector<float>>();
    for (int e = elevationLeftBin; e <= elevationRightBin; ++e) {
        for (int a = azimuthLeftBin; a <= azimuthRightBin; ++a) {
            for (int r = 0; r <= rangeMaxBin; ++r) {
                int index = ((e * numAzimuthBins_ + a) * nRangeBins() + r) * 2;
                if (hasDoppler_) {
                    for (int n = 0; n < 2; ++n) result.heatmap->push_back((*heatmap)[index + n]);
                }
                else result.heatmap->push_back((*heatmap)[index]);
            }
        }
    }
    result.newConfig->numAzimuthBins_ = azimuthRightBin - azimuthLeftBin + 1;
    result.newConfig->numElevationBins_ = elevationRightBin - elevationLeftBin + 1;
    result.newConfig->setNumRangeBins(rangeMaxBin + 1);
    result.newConfig->azimuthBins_ = std::vector<float>(azimuthBins_.begin() + azimuthLeftBin, azimuthBins_.begin() + azimuthRightBin + 1);
    result.newConfig->elevationBins_ = std::vector<float>(elevationBins_.begin() + elevationLeftBin, elevationBins_.begin() + elevationRightBin + 1);
    return result;
}

HeatmapTransformResult RadarConfig::clipHeatmap(
    const std::shared_ptr<std::vector<float>>& heatmap,
    float horizontalFov, float verticalFov, float range) const
{
    int azMaxBin = horizontalFovToAzimuthIdx(horizontalFov);
    int elMaxBin = verticalFovToElevationIdx(verticalFov);
    int rangeMaxBin = rangeToRangeIdx(range);
    return clipHeatmap(heatmap, azMaxBin, elMaxBin, rangeMaxBin);
}


HeatmapTransformResult RadarConfig::collapseHeatmapElevation(
    const std::shared_ptr<std::vector<float>>& image,
    const float& elevationMinMeters, const float& elevationMaxMeters) const
{
    if (elevationMaxMeters < elevationMinMeters) throw std::out_of_range("elevationMaxMeters must be greater or equal to elevationMinMeters.");
    auto collapsedHeatmap = std::make_shared<std::vector<float>>(numAzimuthBins_ * nRangeBins() * 2);

    for (int a = 0; a < numAzimuthBins_; ++a) {
        for (int r = 0; r < nRangeBins(); ++r) {
            float maxIntensity = -std::numeric_limits<float>::infinity();
            float maxDoppler   = 0.0f;
            for (int e = 0; e < elevationBins_.size(); ++e) {
                float z = r * std::sin(elevationBins_[e]);
                if (z >= elevationMinMeters && z <= elevationMaxMeters) {
                    int index = (((e * numAzimuthBins_ + a) * nRangeBins() + r) * 2);
                    if ((*image)[index] > maxIntensity) {
                        maxIntensity = (*image)[index];
                        maxDoppler   = (*image)[index + 1];
                    }
                }
            }
            collapsedHeatmap->push_back(maxIntensity);
            collapsedHeatmap->push_back(maxDoppler);
        }
    }
    auto newConfig = this->clone();
    newConfig->numElevationBins_ = 0;
    newConfig->elevationBins_ = {};
    HeatmapTransformResult result;
    result.heatmap = collapsedHeatmap;
    result.newConfig = newConfig;
    return result;
}

HeatmapTransformResult RadarConfig::removeDoppler(const std::shared_ptr<std::vector<float>>& image) const {
    auto intensityImage = std::make_shared<std::vector<float>>(image->size() / 2);
    for (size_t i = 0; i < image->size(); i += 2) {
        (*intensityImage)[i] = (*image)[i];
    }
    auto newConfig = this->clone();
    newConfig->hasDoppler_ = false;
    HeatmapTransformResult result;
    result.heatmap = intensityImage;
    result.newConfig = newConfig;
    return result;
}

// std::shared_ptr<std::vector<float>> RadarConfig::swapHeatmapDimensions(const std::shared_ptr<std::vector<float>>& heatmap) {
//     const size_t numDims = hasDoppler ? 2 : 1;
//     const size_t expectedSize = numElevationBins * numAzimuthBins * nRangeBins() * numDims;
//     if (heatmap->size() != expectedSize) {
//         throw std::runtime_error(
//             "RadarConfig::swapHeatmapDimensions(): heatmap size does not match the expected dimensions. Expected size: " +
//             std::to_string(expectedSize) + ", Actual size: " + std::to_string(heatmap->size())
//         );
//     }

//     auto reorganizedHeatmap = std::make_shared<std::vector<float>>(expectedSize);
//     for (int el = 0; el < numElevationBins; ++el) {
//         for (int az = 0; az < numAzimuthBins; ++az) {
//             for (int r = 0; r < nRangeBins(); ++r) {
//                 for (int d = 0; d < numDims; ++d) {
//                     size_t originalIndex    = (((el * numAzimuthBins + az) * nRangeBins()) + r) * numDims + d;
//                     size_t reorganizedIndex = (((az * nRangeBins() + r) * numElevationBins) + el) * numDims + d;
//                     (*reorganizedHeatmap)[reorganizedIndex] = (*heatmap)[originalIndex];
//                 }
//             }
//         }
//     }
//     return reorganizedHeatmap;
// }


Json::Value RadarConfig::toJson() const {
    Json::Value jsonConfig;
    constexpr double MIN_THRESHOLD = 1e-12;

    auto validateParam = [](const std::string& name, double value) {
        if (std::abs(value) < MIN_THRESHOLD) {
            // std::cerr << "Warning: Parameter '" << name << "' has very small absolute value: " << value << std::endl;
            return 0.0;
        }
        return value;
    };

    // Heatmap parameters
    jsonConfig["heatmap"]["numRangeBins"]     = numRangeBins_;     
    jsonConfig["heatmap"]["numPosRangeBins"]  = numPosRangeBins_; 
    jsonConfig["heatmap"]["numElevationBins"] = numElevationBins_;
    jsonConfig["heatmap"]["numAzimuthBins"]   = numAzimuthBins_;
    jsonConfig["heatmap"]["rangeBinWidth"]    = validateParam("rangeBinWidth", rangeBinWidth_);
    jsonConfig["heatmap"]["azimuthBins"]      = Json::arrayValue;
    for (size_t i = 0; i < azimuthBins_.size(); ++i) {
        jsonConfig["heatmap"]["azimuthBins"].append(validateParam("azimuthBins[" + std::to_string(i) + "]", azimuthBins_[i]));
    }
    jsonConfig["heatmap"]["elevationBins"]    = Json::arrayValue;
    for (size_t i = 0; i < elevationBins_.size(); ++i) {
        jsonConfig["heatmap"]["elevationBins"].append(validateParam("elevationBins[" + std::to_string(i) + "]", elevationBins_[i]));
    }

    // Antenna parameters
    jsonConfig["antenna"]["designFrequency"] = validateParam("designFrequency", designFrequency_);
    jsonConfig["antenna"]["numTxAntennas"]   = numTxAntennas_;
    jsonConfig["antenna"]["numRxAntennas"]   = numRxAntennas_;
    jsonConfig["antenna"]["txCenters"]       = Json::arrayValue;
    for (size_t i = 0; i < txCenters_.size(); ++i) {
        Json::Value p;
        p["x"] = validateParam("txCenters[" + std::to_string(i) + "].x", txCenters_[i].x);
        p["y"] = validateParam("txCenters[" + std::to_string(i) + "].y", txCenters_[i].y);
        jsonConfig["antenna"]["txCenters"].append(p);
    }
    jsonConfig["antenna"]["rxCenters"] = Json::arrayValue;
    for (size_t i = 0; i < rxCenters_.size(); ++i) {
        Json::Value p;
        p["x"] = validateParam("rxCenters[" + std::to_string(i) + "].x", rxCenters_[i].x);
        p["y"] = validateParam("rxCenters[" + std::to_string(i) + "].y", rxCenters_[i].y);
        jsonConfig["antenna"]["rxCenters"].append(p);
    }

    // Waveform parameters
    jsonConfig["waveform"]["numAdcSamplesPerChirp"] = numAdcSamplesPerChirp_;
    jsonConfig["waveform"]["numChirpsPerFrame"]     = numChirpsPerFrame_;
    jsonConfig["waveform"]["adcSampleFrequency"]    = adcSampleFrequency_;
    jsonConfig["waveform"]["startFrequency"]        = validateParam("startFrequency", startFrequency_);
    jsonConfig["waveform"]["idleTime"]              = validateParam("idleTime", idleTime_);
    jsonConfig["waveform"]["adcStartTime"]          = validateParam("adcStartTime", adcStartTime_);
    jsonConfig["waveform"]["rampEndTime"]           = validateParam("rampEndTime", rampEndTime_);
    jsonConfig["waveform"]["frequencySlope"]        = validateParam("frequencySlope", frequencySlope_);

    // Calibration parameters
    jsonConfig["calibration"]["numDopplerBins"]     = numDopplerBins_;
    jsonConfig["calibration"]["couplingCalibMatrix"] = Json::arrayValue;
    for (size_t i = 0; i < couplingCalibMatrix_.size(); ++i) {
        Json::Value c;
        c["real"] = validateParam("couplingCalibMatrix[" + std::to_string(i) + "].real", couplingCalibMatrix_[i].real());
        c["imag"] = validateParam("couplingCalibMatrix[" + std::to_string(i) + "].imag", couplingCalibMatrix_[i].imag());
        jsonConfig["calibration"]["couplingCalibMatrix"].append(c);
    }

    // Phase frequency parameters
    jsonConfig["phaseFrequency"]["calibAdcSampleFrequency"] = calibAdcSampleFrequency_;
    jsonConfig["phaseFrequency"]["calibFrequencySlope"]     = validateParam("calibFrequencySlope", calibFrequencySlope_);
    jsonConfig["phaseFrequency"]["frequencyCalibMatrix"]    = Json::arrayValue;
    for (size_t i = 0; i < frequencyCalibMatrix_.size(); ++i) {
        Json::Value c;
        c["real"] = validateParam("frequencyCalibMatrix[" + std::to_string(i) + "].real", frequencyCalibMatrix_[i].real());
        c["imag"] = validateParam("frequencyCalibMatrix[" + std::to_string(i) + "].imag", frequencyCalibMatrix_[i].imag());
        jsonConfig["phaseFrequency"]["frequencyCalibMatrix"].append(c);
    }
    jsonConfig["phaseFrequency"]["phaseCalibMatrix"] = Json::arrayValue;
    for (size_t i = 0; i < phaseCalibMatrix_.size(); ++i) {
        Json::Value c;
        c["real"] = validateParam("phaseCalibMatrix[" + std::to_string(i) + "].real", phaseCalibMatrix_[i].real());
        c["imag"] = validateParam("phaseCalibMatrix[" + std::to_string(i) + "].imag", phaseCalibMatrix_[i].imag());
        jsonConfig["phaseFrequency"]["phaseCalibMatrix"].append(c);
    }

    // Internal parameters
    jsonConfig["internal"]["numAzimuthBeams"]     = numAzimuthBeams_;
    jsonConfig["internal"]["numElevationBeams"]   = numElevationBeams_;
    jsonConfig["internal"]["azimuthApertureLen"]  = azimuthApertureLen_;
    jsonConfig["internal"]["elevationApertureLen"]= elevationApertureLen_;
    jsonConfig["internal"]["numAngles"]           = numAngles_;
    jsonConfig["internal"]["numVirtualElements"]  = numVirtualElements_;
    jsonConfig["internal"]["virtualArrayMap"]     = Json::arrayValue;
    for (const auto& v : virtualArrayMap_) jsonConfig["internal"]["virtualArrayMap"].append(v);
    jsonConfig["internal"]["azimuthAngles"]       = Json::arrayValue;
    for (size_t i = 0; i < azimuthAngles_.size(); ++i) {
        jsonConfig["internal"]["azimuthAngles"].append(validateParam("azimuthAngles[" + std::to_string(i) + "]", azimuthAngles_[i]));
    }
    jsonConfig["internal"]["elevationAngles"]     = Json::arrayValue;
    for (size_t i = 0; i < elevationAngles_.size(); ++i) {
        jsonConfig["internal"]["elevationAngles"].append(validateParam("elevationAngles[" + std::to_string(i) + "]", elevationAngles_[i]));
    }
    jsonConfig["internal"]["hasDoppler"]          = hasDoppler_;
    jsonConfig["internal"]["dopplerBinWidth"]     = validateParam("dopplerBinWidth", dopplerBinWidth_);

    return jsonConfig;
}


void RadarConfig::fromJson(const std::string& jsonString) {
    Json::CharReaderBuilder b;
    std::unique_ptr<Json::CharReader> reader(b.newCharReader());
    Json::Value root;
    std::string errs;
    const char* begin = jsonString.data();
    const char* end = begin + jsonString.size();
    if (!reader->parse(begin, end, &root, &errs)) {
        throw std::invalid_argument("RadarConfig::fromJson: parse error: " + errs);
    }
    fromJson(root);
}


void RadarConfig::fromJson(const Json::Value& root) {
    if (!root.isObject()) {
        throw std::invalid_argument("RadarConfig::fromJson: root must be an object.");
    }

    // ---- heatmap
    if (!root.isMember("heatmap") || !root["heatmap"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'heatmap' object.");
    const auto& H = root["heatmap"];

    if (!H.isMember("numRangeBins") || !H["numRangeBins"].isInt())
        throw std::invalid_argument("RadarConfig::fromJson: heatmap.numRangeBins missing or not int.");
    // if (!H.isMember("numPosRangeBins") || !H["numPosRangeBins"].isInt())
    //     throw std::invalid_argument("RadarConfig::fromJson: heatmap.numPosRangeBins missing or not int.");
    if (!H.isMember("numElevationBins") || !H["numElevationBins"].isInt())
        throw std::invalid_argument("RadarConfig::fromJson: heatmap.numElevationBins missing or not int.");
    if (!H.isMember("numAzimuthBins") || !H["numAzimuthBins"].isInt())
        throw std::invalid_argument("RadarConfig::fromJson: heatmap.numAzimuthBins missing or not int.");
    if (!H.isMember("rangeBinWidth") || !H["rangeBinWidth"].isNumeric())
        throw std::invalid_argument("RadarConfig::fromJson: heatmap.rangeBinWidth missing or not number.");
    if (!H.isMember("azimuthBins") || !H["azimuthBins"].isArray())
        throw std::invalid_argument("RadarConfig::fromJson: heatmap.azimuthBins missing or not array.");
    if (!H.isMember("elevationBins") || !H["elevationBins"].isArray())
        throw std::invalid_argument("RadarConfig::fromJson: heatmap.elevationBins missing or not array.");

    numRangeBins_     = H["numRangeBins"].asInt();
    numPosRangeBins_  = H["numPosRangeBins"].asInt();
    numElevationBins_ = H["numElevationBins"].asInt();
    numAzimuthBins_   = H["numAzimuthBins"].asInt();
    rangeBinWidth_    = H["rangeBinWidth"].asDouble();

    azimuthBins_.clear();
    azimuthBins_.reserve(H["azimuthBins"].size());
    for (const auto& v : H["azimuthBins"]) {
        if (!v.isNumeric()) throw std::invalid_argument("heatmap.azimuthBins: all values must be numbers.");
        azimuthBins_.push_back(static_cast<float>(v.asDouble()));
    }
    elevationBins_.clear();
    elevationBins_.reserve(H["elevationBins"].size());
    for (const auto& v : H["elevationBins"]) {
        if (!v.isNumeric()) throw std::invalid_argument("heatmap.elevationBins: all values must be numbers.");
        elevationBins_.push_back(static_cast<float>(v.asDouble()));
    }

    // ---- antenna
    if (!root.isMember("antenna") || !root["antenna"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'antenna' object.");
    const auto& A = root["antenna"];

    if (!A.isMember("designFrequency") || !A["designFrequency"].isNumeric())
        throw std::invalid_argument("RadarConfig::fromJson: antenna.designFrequency missing or not number.");
    if (!A.isMember("numTxAntennas") || !A["numTxAntennas"].isInt())
        throw std::invalid_argument("RadarConfig::fromJson: antenna.numTxAntennas missing or not int.");
    if (!A.isMember("numRxAntennas") || !A["numRxAntennas"].isInt())
        throw std::invalid_argument("RadarConfig::fromJson: antenna.numRxAntennas missing or not int.");
    if (!A.isMember("txCenters") || !A["txCenters"].isArray())
        throw std::invalid_argument("RadarConfig::fromJson: antenna.txCenters missing or not array.");
    if (!A.isMember("rxCenters") || !A["rxCenters"].isArray())
        throw std::invalid_argument("RadarConfig::fromJson: antenna.rxCenters missing or not array.");

    designFrequency_ = A["designFrequency"].asDouble();
    numTxAntennas_   = A["numTxAntennas"].asInt();
    numRxAntennas_   = A["numRxAntennas"].asInt();

    txCenters_.clear();
    txCenters_.reserve(A["txCenters"].size());
    for (const auto& p : A["txCenters"]) {
        if (!p.isObject() || !p.isMember("x") || !p.isMember("y") || !p["x"].isNumeric() || !p["y"].isNumeric())
            throw std::invalid_argument("antenna.txCenters: each entry must be {x:number, y:number}.");
        pcl::PointXY pt; pt.x = static_cast<float>(p["x"].asDouble()); pt.y = static_cast<float>(p["y"].asDouble());
        txCenters_.push_back(pt);
    }
    rxCenters_.clear();
    rxCenters_.reserve(A["rxCenters"].size());
    for (const auto& p : A["rxCenters"]) {
        if (!p.isObject() || !p.isMember("x") || !p.isMember("y") || !p["x"].isNumeric() || !p["y"].isNumeric())
            throw std::invalid_argument("antenna.rxCenters: each entry must be {x:number, y:number}.");
        pcl::PointXY pt; pt.x = static_cast<float>(p["x"].asDouble()); pt.y = static_cast<float>(p["y"].asDouble());
        rxCenters_.push_back(pt);
    }

    // ---- waveform
    if (!root.isMember("waveform") || !root["waveform"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'waveform' object.");
    const auto& W = root["waveform"];

    if (!W.isMember("numAdcSamplesPerChirp") || !W["numAdcSamplesPerChirp"].isInt())
        throw std::invalid_argument("waveform.numAdcSamplesPerChirp missing or not int.");
    if (!W.isMember("numChirpsPerFrame") || !W["numChirpsPerFrame"].isInt())
        throw std::invalid_argument("waveform.numChirpsPerFrame missing or not int.");
    if (!W.isMember("adcSampleFrequency") || !W["adcSampleFrequency"].isInt())
        throw std::invalid_argument("waveform.adcSampleFrequency missing or not int.");
    if (!W.isMember("startFrequency") || !W["startFrequency"].isNumeric())
        throw std::invalid_argument("waveform.startFrequency missing or not number.");
    if (!W.isMember("idleTime") || !W["idleTime"].isNumeric())
        throw std::invalid_argument("waveform.idleTime missing or not number.");
    if (!W.isMember("adcStartTime") || !W["adcStartTime"].isNumeric())
        throw std::invalid_argument("waveform.adcStartTime missing or not number.");
    if (!W.isMember("rampEndTime") || !W["rampEndTime"].isNumeric())
        throw std::invalid_argument("waveform.rampEndTime missing or not number.");
    if (!W.isMember("frequencySlope") || !W["frequencySlope"].isNumeric())
        throw std::invalid_argument("waveform.frequencySlope missing or not number.");

    numAdcSamplesPerChirp_ = W["numAdcSamplesPerChirp"].asInt();
    numChirpsPerFrame_     = W["numChirpsPerFrame"].asInt();
    adcSampleFrequency_    = W["adcSampleFrequency"].asInt();
    startFrequency_        = W["startFrequency"].asDouble();
    idleTime_              = W["idleTime"].asDouble();
    adcStartTime_          = W["adcStartTime"].asDouble();
    rampEndTime_           = W["rampEndTime"].asDouble();
    frequencySlope_        = W["frequencySlope"].asDouble();

    // ---- calibration
    if (!root.isMember("calibration") || !root["calibration"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'calibration' object.");
    const auto& C = root["calibration"];

    if (!C.isMember("numDopplerBins") || !C["numDopplerBins"].isInt())
        throw std::invalid_argument("calibration.numDopplerBins missing or not int.");
    if (!C.isMember("couplingCalibMatrix") || !C["couplingCalibMatrix"].isArray())
        throw std::invalid_argument("calibration.couplingCalibMatrix missing or not array.");

    numDopplerBins_ = C["numDopplerBins"].asInt();
    couplingCalibMatrix_.clear();
    couplingCalibMatrix_.reserve(C["couplingCalibMatrix"].size());
    for (const auto& z : C["couplingCalibMatrix"]) {
        if (!z.isObject() || !z.isMember("real") || !z.isMember("imag") ||
            !z["real"].isNumeric() || !z["imag"].isNumeric())
            throw std::invalid_argument("calibration.couplingCalibMatrix elements must be {real,imag} numbers.");
        couplingCalibMatrix_.emplace_back(z["real"].asDouble(), z["imag"].asDouble());
    }

    // ---- phaseFrequency
    if (!root.isMember("phaseFrequency") || !root["phaseFrequency"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'phaseFrequency' object.");
    const auto& P = root["phaseFrequency"];

    if (!P.isMember("calibAdcSampleFrequency") || !P["calibAdcSampleFrequency"].isInt())
        throw std::invalid_argument("phaseFrequency.calibAdcSampleFrequency missing or not int.");
    if (!P.isMember("calibFrequencySlope") || !P["calibFrequencySlope"].isNumeric())
        throw std::invalid_argument("phaseFrequency.calibFrequencySlope missing or not number.");
    if (!P.isMember("frequencyCalibMatrix") || !P["frequencyCalibMatrix"].isArray())
        throw std::invalid_argument("phaseFrequency.frequencyCalibMatrix missing or not array.");
    if (!P.isMember("phaseCalibMatrix") || !P["phaseCalibMatrix"].isArray())
        throw std::invalid_argument("phaseFrequency.phaseCalibMatrix missing or not array.");

    calibAdcSampleFrequency_ = P["calibAdcSampleFrequency"].asInt();
    calibFrequencySlope_     = P["calibFrequencySlope"].asDouble();

    frequencyCalibMatrix_.clear();
    frequencyCalibMatrix_.reserve(P["frequencyCalibMatrix"].size());
    for (const auto& z : P["frequencyCalibMatrix"]) {
        if (!z.isObject() || !z.isMember("real") || !z.isMember("imag") ||
            !z["real"].isNumeric() || !z["imag"].isNumeric())
            throw std::invalid_argument("phaseFrequency.frequencyCalibMatrix elements must be {real,imag} numbers.");
        frequencyCalibMatrix_.emplace_back(z["real"].asDouble(), z["imag"].asDouble());
    }
    phaseCalibMatrix_.clear();
    phaseCalibMatrix_.reserve(P["phaseCalibMatrix"].size());
    for (const auto& z : P["phaseCalibMatrix"]) {
        if (!z.isObject() || !z.isMember("real") || !z.isMember("imag") ||
            !z["real"].isNumeric() || !z["imag"].isNumeric())
            throw std::invalid_argument("phaseFrequency.phaseCalibMatrix elements must be {real,imag} numbers.");
        phaseCalibMatrix_.emplace_back(z["real"].asDouble(), z["imag"].asDouble());
    }

    // ---- internal
    if (!root.isMember("internal") || !root["internal"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'internal' object.");
    const auto& I = root["internal"];

    if (!I.isMember("numAzimuthBeams") || !I["numAzimuthBeams"].isInt())
        throw std::invalid_argument("internal.numAzimuthBeams missing or not int.");
    if (!I.isMember("numElevationBeams") || !I["numElevationBeams"].isInt())
        throw std::invalid_argument("internal.numElevationBeams missing or not int.");
    if (!I.isMember("azimuthApertureLen") || !I["azimuthApertureLen"].isInt())
        throw std::invalid_argument("internal.azimuthApertureLen missing or not int.");
    if (!I.isMember("elevationApertureLen") || !I["elevationApertureLen"].isInt())
        throw std::invalid_argument("internal.elevationApertureLen missing or not int.");
    if (!I.isMember("numAngles") || !I["numAngles"].isInt())
        throw std::invalid_argument("internal.numAngles missing or not int.");
    if (!I.isMember("numVirtualElements") || !I["numVirtualElements"].isInt())
        throw std::invalid_argument("internal.numVirtualElements missing or not int.");
    if (!I.isMember("virtualArrayMap") || !I["virtualArrayMap"].isArray())
        throw std::invalid_argument("internal.virtualArrayMap missing or not array.");
    if (!I.isMember("azimuthAngles") || !I["azimuthAngles"].isArray())
        throw std::invalid_argument("internal.azimuthAngles missing or not array.");
    if (!I.isMember("elevationAngles") || !I["elevationAngles"].isArray())
        throw std::invalid_argument("internal.elevationAngles missing or not array.");
    if (!I.isMember("hasDoppler") || !I["hasDoppler"].isBool())
        throw std::invalid_argument("internal.hasDoppler missing or not bool.");
    if (!I.isMember("dopplerBinWidth") || !I["dopplerBinWidth"].isNumeric())
        throw std::invalid_argument("internal.dopplerBinWidth missing or not number.");

    numAzimuthBeams_     = I["numAzimuthBeams"].asInt();
    numElevationBeams_   = I["numElevationBeams"].asInt();
    azimuthApertureLen_  = I["azimuthApertureLen"].asInt();
    elevationApertureLen_= I["elevationApertureLen"].asInt();
    numAngles_           = I["numAngles"].asInt();
    numVirtualElements_  = I["numVirtualElements"].asInt();

    virtualArrayMap_.clear();
    virtualArrayMap_.reserve(I["virtualArrayMap"].size());
    for (const auto& v : I["virtualArrayMap"]) {
        if (!v.isInt()) throw std::invalid_argument("internal.virtualArrayMap: all values must be ints.");
        virtualArrayMap_.push_back(v.asInt());
    }

    azimuthAngles_.clear();
    azimuthAngles_.reserve(I["azimuthAngles"].size());
    for (const auto& a : I["azimuthAngles"]) {
        if (!a.isNumeric()) throw std::invalid_argument("internal.azimuthAngles: all values must be numbers.");
        azimuthAngles_.push_back(static_cast<float>(a.asDouble()));
    }
    elevationAngles_.clear();
    elevationAngles_.reserve(I["elevationAngles"].size());
    for (const auto& a : I["elevationAngles"]) {
        if (!a.isNumeric()) throw std::invalid_argument("internal.elevationAngles: all values must be numbers.");
        elevationAngles_.push_back(static_cast<float>(a.asDouble()));
    }

    hasDoppler_      = I["hasDoppler"].asBool();
    dopplerBinWidth_ = I["dopplerBinWidth"].asDouble();

    precomputePointcloudTemplate();
}


}


std::map<std::string, std::vector<std::string>> readConfig(const std::filesystem::path& configFile) {
    coloradar::internal::checkPathExists(configFile);
    std::ifstream infile(configFile);
    if (!infile.is_open()) {
        throw std::runtime_error("Unable to open config file: " + configFile.string());
    }
    std::map<std::string, std::vector<std::string>> configMap;
    std::string line;
    std::regex keyValueRegex(R"(\s*([^#\s]+)\s*[:=\s]\s*(.*))");
    const size_t maxKeySearchLength = 256;

    while (std::getline(infile, line)) {
        if (line.empty() || line.find_first_not_of(" \t") == std::string::npos || line.find('#') == 0) {
            continue;
        }
        std::string lineHead = line.substr(0, maxKeySearchLength);
        std::smatch match;
        if (std::regex_match(lineHead, match, keyValueRegex)) {
            std::string key = match[1];
            size_t valueStartPos = match.position(2);
            std::string valueSection = line.substr(valueStartPos);
            std::istringstream valuesStream(valueSection);
            std::string value;
            std::vector<std::string> values;
            while (valuesStream >> value) {
                values.push_back(value);
            }
            configMap[key] = values;
        }
    }
    infile.close();
    return configMap;
}

Json::Value readJsonConfig(const std::filesystem::path& configFile) {
    if (!std::filesystem::exists(configFile)) {
        throw std::runtime_error("Config file does not exist: " + configFile.string());
    }
    std::ifstream file(configFile);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open config file: " + configFile.string());
    }
    Json::Value root;
    file >> root;
    return root;
}


void coloradar::RadarConfig::initAntennaParams(const std::filesystem::path& antennaCfgFile) {
    auto configMap = readConfig(antennaCfgFile);
    bool hasNumRx = false, hasNumTx = false, hasDesignF = false;
    rxCenters_.clear();
    txCenters_.clear();

    std::fstream cfg_file(antennaCfgFile, std::iostream::in);
    std::string line;
    int rx_count = 0;
    int tx_count = 0;
    while (std::getline(cfg_file,line))
    {
      std::stringstream ss(line);
      std::string token;
      std::getline(ss, token, ' ');
      if (token.compare("num_rx") == 0)
      {
        std::getline(ss, token, ' ');
        numRxAntennas_ = std::stoi(token);
        rxCenters_.resize(numRxAntennas_);
        hasNumRx = true;
      }
      if (token.compare("num_tx") == 0)
      {
        std::getline(ss, token, ' ');
        numTxAntennas_ = std::stoi(token);
        txCenters_.resize(numTxAntennas_);
        hasNumTx = true;
      }
      if (token.compare("F_design") == 0)
      {
        std::getline(ss, token, ' ');
        designFrequency_ = std::stoi(token) * 1e9;  // convert from GHz to Hz
        hasDesignF = true;
      }
      if (token.compare("rx") == 0)
      {
        std::string token;
        std::getline(ss, token, ' ');
        int idx = std::stoi(token);
        std::getline(ss, token, ' ');
        double x = std::stod(token);
        std::getline(ss, token, ' ');
        double y = std::stod(token);
        pcl::PointXY rx_center (x, y);
        rxCenters_[idx] = rx_center;
        rx_count++;
      }
      if (token.compare("tx") == 0)
      {
        std::string token;
        std::getline(ss, token, ' ');
        int idx = std::stoi(token);
        std::getline(ss, token, ' ');
        double x = std::stod(token);
        std::getline(ss, token, ' ');
        double y = std::stod(token);
        pcl::PointXY tx_center (x, y);
        txCenters_[idx] = tx_center;
        tx_count++;
      }
    }
    if (!hasNumRx || !hasNumTx || !hasDesignF) {
        throw std::runtime_error("Missing num_rx or num_tx or F_design in antenna config.");
    }
    if (rx_count != rxCenters_.size()) {
      throw std::runtime_error("antenna config specified num_rx = " + std::to_string(rxCenters_.size()) + " but only " + std::to_string(rx_count) + " rx positions found.");
    }
    if (tx_count != txCenters_.size()) {
      throw std::runtime_error("antenna config specified num_tx = " + std::to_string(txCenters_.size()) + " but only " + std::to_string(tx_count) + " tx positions found.");
    }
}

void coloradar::RadarConfig::initHeatmapParams(const std::filesystem::path& heatmapCfgFile) {
    auto configMap = readConfig(heatmapCfgFile);
    azimuthBins_.clear();
    elevationBins_.clear();

    auto it = configMap.find("num_range_bins");
    if (it != configMap.end()) {
        numPosRangeBins_ = std::stoi(it->second[0]);
        numRangeBins_ = numPosRangeBins_ * 2;
    } else {
        throw std::runtime_error("Missing num_range_bins in heatmap config.");
    }
    it = configMap.find("num_elevation_bins");
    if (it != configMap.end()) {
        numElevationBins_ = std::stoi(it->second[0]);
    }
    it = configMap.find("num_azimuth_bins");
    if (it != configMap.end()) {
        numAzimuthBins_ = std::stoi(it->second[0]);
    }
    it = configMap.find("range_bin_width");
    if (it != configMap.end()) {
        rangeBinWidth_ = std::stod(it->second[0]);
    }
    it = configMap.find("azimuth_bins");
    if (it != configMap.end()) {
        for (const auto& bin : it->second) {
            azimuthBins_.push_back(std::stof(bin));
        }
    }
    it = configMap.find("elevation_bins");
    if (it != configMap.end()) {
        for (const auto& bin : it->second) {
            elevationBins_.push_back(std::stof(bin));
        }
    }
}


void coloradar::RadarConfig::initWaveformParams(const std::filesystem::path& waveformCfgFile) {
    auto configMap = readConfig(waveformCfgFile);

    auto it = configMap.find("num_adc_samples_per_chirp");
    if (it != configMap.end()) {
        numAdcSamplesPerChirp_ = std::stoi(it->second[0]);
    } else {
        throw std::runtime_error("Missing num_adc_samples_per_chirp in waveform config.");
    }
    it = configMap.find("num_chirps_per_frame");
    if (it != configMap.end()) {
        numChirpsPerFrame_ = std::stoi(it->second[0]);
    }
    it = configMap.find("adc_sample_frequency");
    if (it != configMap.end()) {
        adcSampleFrequency_ = std::stod(it->second[0]);
    }
    it = configMap.find("start_frequency");
    if (it != configMap.end()) {
        startFrequency_ = std::stod(it->second[0]);
    }
    it = configMap.find("idle_time");
    if (it != configMap.end()) {
        idleTime_ = std::stod(it->second[0]);
    }
    it = configMap.find("adc_start_time");
    if (it != configMap.end()) {
        adcStartTime_ = std::stod(it->second[0]);
    }
    it = configMap.find("ramp_end_time");
    if (it != configMap.end()) {
        rampEndTime_ = std::stod(it->second[0]);
    }
    it = configMap.find("frequency_slope");
    if (it != configMap.end()) {
        frequencySlope_ = std::stod(it->second[0]);
    }
}

void coloradar::RadarConfig::initCouplingParams(const std::filesystem::path& couplingCfgFile) {
    std::ifstream file(couplingCfgFile);
    if (!file.is_open())
        throw std::runtime_error("Unable to open coupling config file: " + couplingCfgFile.string());
    std::string line, token;
    std::stringstream ss;
    couplingCalibMatrix_.clear();

    while (std::getline(file, line)) {
        ss.clear();
        ss.str(line);
        std::getline(ss, token, ':');
        if (token == "num_doppler_bins") {
            std::getline(ss, token);
            numDopplerBins_ = std::stoi(token);
        }
        if (token == "data") {
            std::vector<double> values;
            while (std::getline(ss, token, ','))
                values.push_back(std::stod(token));
            size_t expectedSize = numTxAntennas_ * numRxAntennas_ * numPosRangeBins_ * 2;
            if (values.size() != expectedSize)
                throw std::runtime_error("Mismatch in the size of the coupling calibration matrix. Expected: " + std::to_string(expectedSize) + ", Found: " + std::to_string(values.size()));
            couplingCalibMatrix_.resize(values.size() / 2);
            for (size_t i = 0; i < values.size(); i += 2) {
                couplingCalibMatrix_[i / 2] = std::complex<double>(values[i], values[i + 1]);
            }
        }
    }
    file.close();
    if (couplingCalibMatrix_.empty())
        throw std::runtime_error("Coupling calibration data not found in config.");
}


void coloradar::RadarConfig::initPhaseFrequencyParams(const std::filesystem::path& phaseFrequencyCfgFile) {
    const Json::Value& configMap = readJsonConfig(phaseFrequencyCfgFile);
    if (!configMap.isMember("antennaCalib")) {
        throw std::runtime_error("Missing antennaCalib in phase frequency config.");
    }
    const Json::Value& config = configMap["antennaCalib"];

    std::vector<double> freqData(numTxAntennas_ * numRxAntennas_);
    std::vector<std::complex<double>> phaseData(numTxAntennas_ * numRxAntennas_);

    if (config.isMember("frequencySlope")) {
        calibFrequencySlope_ = config["frequencySlope"].asDouble();
    } else {
        throw std::runtime_error("Missing frequencySlope in phase frequency config.");
    }
    if (config.isMember("samplingRate")) {
        calibAdcSampleFrequency_ = config["samplingRate"].asInt();
    } else {
        throw std::runtime_error("Missing samplingRate in phase frequency config.");
    }

    if (config.isMember("frequencyCalibrationMatrix")) {
        const Json::Value& frequencyMatrix = config["frequencyCalibrationMatrix"];
        if (frequencyMatrix.size() != numTxAntennas_ * numRxAntennas_) {
            throw std::runtime_error("Invalid frequency calibration array: expected " + std::to_string(numRxAntennas_ * numTxAntennas_) + " elements, got " + std::to_string(frequencyMatrix.size()));
        }
        int count = 0;
        for (const auto& value : frequencyMatrix) {
            freqData[count] = value.asDouble();
            count++;
        }
    } else {
        throw std::runtime_error("Missing frequencyCalibrationMatrix in phase frequency config.");
    }

    if (config.isMember("phaseCalibrationMatrix")) {
        const Json::Value& phaseMatrix = config["phaseCalibrationMatrix"];
        if (phaseMatrix.size() % 2 != 0) {
            throw std::runtime_error("Invalid phaseCalibrationMatrix: Expecting pairs of real and imaginary values.");
        }
        if (phaseMatrix.size() / 2 != numTxAntennas_ * numRxAntennas_) {
            throw std::runtime_error("Invalid phase calibration array: expected " + std::to_string(numRxAntennas_ * numTxAntennas_) + " elements, got " + std::to_string(phaseMatrix.size()));
        }
        for (Json::ArrayIndex i = 0; i < phaseMatrix.size(); i += 2) {
            double real = phaseMatrix[i].asDouble();
            double imag = phaseMatrix[i + 1].asDouble();
            phaseData[i / 2] = std::complex<double>(real, imag);
        }
    } else {
        throw std::runtime_error("Missing phaseCalibrationMatrix in phase frequency config.");
    }

    frequencyCalibMatrix_.clear();
    frequencyCalibMatrix_.resize(numTxAntennas_ * numRxAntennas_ * numRangeBins_);
    phaseCalibMatrix_.clear();
    phaseCalibMatrix_.resize(numTxAntennas_ * numRxAntennas_);

    for (int tx_idx = 0; tx_idx < numTxAntennas_; tx_idx++) {
        for (int rx_idx = 0; rx_idx < numRxAntennas_; rx_idx++) {
            int idx = rx_idx + (tx_idx * numRxAntennas_);
            double delta_p = freqData[idx] - freqData[0];
            double freq_calib = 2.0 * M_PI * delta_p / numRangeBins_ * (frequencySlope_ / calibFrequencySlope_) * (adcSampleFrequency_ / calibAdcSampleFrequency_);
            for (int sample_idx = 0; sample_idx < numRangeBins_; sample_idx++) {
                int cal_idx = sample_idx + numRangeBins_ * (rx_idx + numRxAntennas_ * tx_idx);
                frequencyCalibMatrix_[cal_idx] = std::exp(std::complex<double>(0.0, -1.0) * std::complex<double>(freq_calib, 0.0) * std::complex<double>(sample_idx, 0.0));
            }
        }
    }
    std::complex<double> phase_ref = phaseData[0];
    for (int tx_idx = 0; tx_idx < numTxAntennas_; tx_idx++) {
        for (int rx_idx = 0; rx_idx < numRxAntennas_; rx_idx++) {
            int idx = rx_idx + (tx_idx * numRxAntennas_);
            phaseCalibMatrix_[idx] = phase_ref / phaseData[idx];
        }
    }
}


void coloradar::RadarConfig::initInternalParams() {
    azimuthApertureLen_ = 0;
    elevationApertureLen_ = 0;
    numVirtualElements_ = 0;
    virtualArrayMap_.clear();
    azimuthAngles_.clear();
    elevationAngles_.clear();
    azimuthAngles_.resize(numAzimuthBeams_);
    elevationAngles_.resize(numElevationBeams_);
    numAngles_ = numAzimuthBeams_ * numElevationBeams_;

    for (int tx_idx = 0; tx_idx < numTxAntennas_; tx_idx++)
    {
      for (int rx_idx = 0; rx_idx < numRxAntennas_; rx_idx++)
      {
        int virtual_x = rxCenters_[rx_idx].x + txCenters_[tx_idx].x;
        int virtual_y = rxCenters_[rx_idx].y + txCenters_[tx_idx].y;
        // check to ensure this antenna pair doesn't map to the same virtual
        // location as a previously evaluated antenna pair
        bool redundant = false;
        for (int i = 0; i < numVirtualElements_; i++)
        {
          int idx = i * 4;
          if (virtualArrayMap_[idx] == virtual_x
            && virtualArrayMap_[idx+1] == virtual_y)
            redundant = true;
        }
        // record mapping from antenna pair index to virtual antenna location
        // stored in vector with entries grouped into 4-tuples of
        // [azimuth_location, elevation_location, rx_index, tx_index]
        if (!redundant)
        {
          if (virtual_x + 1 > azimuthApertureLen_)
            azimuthApertureLen_ = virtual_x + 1;
          if (virtual_y + 1 > elevationApertureLen_)
            elevationApertureLen_ = virtual_y + 1;

          virtualArrayMap_.push_back(virtual_x);
          virtualArrayMap_.push_back(virtual_y);
          virtualArrayMap_.push_back(rx_idx);
          virtualArrayMap_.push_back(tx_idx);
          numVirtualElements_++;
        }
      }
    }
    double wavelength = c / (startFrequency_ + adcStartTime_ * frequencySlope_);
    double chirp_time = idleTime_ + rampEndTime_;
    double v_max = wavelength / (4.0 * numTxAntennas_ * chirp_time);
    dopplerBinWidth_ = v_max / numDopplerBins_;

    double center_frequency = startFrequency_ + numRangeBins_ / adcSampleFrequency_ * frequencySlope_ / 2.0;
    double d = 0.5 * center_frequency / designFrequency_;
    double az_d_phase = (2. * M_PI) / numAzimuthBeams_;
    double phase_dif = (az_d_phase / 2.) - M_PI;
    for (int i = 0; i < numAzimuthBeams_; i++) {
      azimuthAngles_[i] = asin(phase_dif / (2. * M_PI * d));
      phase_dif += az_d_phase;
    }
    double el_d_phase = (2.*M_PI) / numElevationBeams_;
    phase_dif = (el_d_phase / 2.) - M_PI;
    for (int i = 0; i < numElevationBeams_; i++) {
        elevationAngles_[i] = asin(phase_dif / (2. * M_PI * d));
        phase_dif += el_d_phase;
    }
}

coloradar::SingleChipConfig::SingleChipConfig(const std::filesystem::path& calibDir, const int& nAzimuthBeams, const int& nElevationBeams) : coloradar::RadarConfig(nAzimuthBeams, nElevationBeams) {
    coloradar::internal::checkPathExists(calibDir);
    init(calibDir);
}

void coloradar::SingleChipConfig::init(const std::filesystem::path& calibDir) {
    std::filesystem::path configDir = calibDir / "single_chip";
    coloradar::internal::checkPathExists(configDir);
    std::filesystem::path antennaConfigFilePath = configDir / "antenna_cfg.txt";
    std::filesystem::path heatmapConfigFilePath = configDir / "heatmap_cfg.txt";
    std::filesystem::path waveformConfigFilePath = configDir / "waveform_cfg.txt";
    std::filesystem::path couplingConfigFilePath = configDir / "coupling_calib.txt";
    initAntennaParams(antennaConfigFilePath);
    initHeatmapParams(heatmapConfigFilePath);
    initWaveformParams(waveformConfigFilePath);
    initCouplingParams(couplingConfigFilePath);
    initInternalParams();
    precomputePointcloudTemplate();
}

coloradar::CascadeConfig::CascadeConfig(const std::filesystem::path& calibDir, const int& nAzimuthBeams, const int& nElevationBeams) : coloradar::RadarConfig(nAzimuthBeams, nElevationBeams) {
    coloradar::internal::checkPathExists(calibDir);
    init(calibDir);
}

void coloradar::CascadeConfig::init(const std::filesystem::path& calibDir) {
    std::filesystem::path configDir = calibDir / "cascade";
    coloradar::internal::checkPathExists(configDir);
    std::filesystem::path antennaConfigFilePath = configDir / "antenna_cfg.txt";
    std::filesystem::path heatmapConfigFilePath = configDir / "heatmap_cfg.txt";
    std::filesystem::path waveformConfigFilePath = configDir / "waveform_cfg.txt";
    std::filesystem::path couplingConfigFilePath = configDir / "coupling_calib.txt";
    std::filesystem::path phaseFrequencyConfigFilePath = configDir / "phase_frequency_calib.txt";
    initAntennaParams(antennaConfigFilePath);
    initHeatmapParams(heatmapConfigFilePath);
    initWaveformParams(waveformConfigFilePath);
    initCouplingParams(couplingConfigFilePath);
    initPhaseFrequencyParams(phaseFrequencyConfigFilePath);
    initInternalParams();
    precomputePointcloudTemplate();
}


void coloradar::RadarConfig::setNumRangeBins(const int& num) { numPosRangeBins_ = num; }
const int& coloradar::RadarConfig::nRangeBins() const { return numPosRangeBins_; }
float coloradar::RadarConfig::maxRange() const { return std::ceil(nRangeBins() * rangeBinWidth_ * 100.0f) / 100.0f; }

int coloradar::RadarConfig::clipAzimuthMaxBin(const int& azMaxBin) const { 
    return azMaxBin >= 0 && (azMaxBin + 1) * 2 < numAzimuthBins_ ? azMaxBin : numAzimuthBins_ / 2 - 1;
}
int coloradar::RadarConfig::clipElevationMaxBin(const int& elMaxBin) const { 
    return elMaxBin >= 0 && (elMaxBin + 1) * 2 < numElevationBins_ ? elMaxBin : numElevationBins_ / 2 - 1;
}
int coloradar::RadarConfig::clipRangeMaxBin(const int& rangeMaxBin) const {
    return rangeMaxBin >= 0 && rangeMaxBin < nRangeBins() ? rangeMaxBin : nRangeBins() - 1;
}
float coloradar::RadarConfig::clipRange(const float& range) const { 
    return range > 0 && range <= maxRange() ? range : maxRange(); 
}
float coloradar::RadarConfig::azimuthIdxToFovDegrees(const int& azMaxBin) const {
    if (azMaxBin < 0 || (azMaxBin + 1) * 2 > numAzimuthBins_) {
        throw std::runtime_error("Invalid azimuth max bin: expected value in [0; " + std::to_string(numAzimuthBins_ / 2) + "), got " + std::to_string(azMaxBin));
    }
    return azimuthBins_[numAzimuthBins_ / 2 + azMaxBin] * 2 * 180.0f / M_PI;
}
float coloradar::RadarConfig::elevationIdxToFovDegrees(const int& elMaxBin) const {
    if (elMaxBin < 0 || (elMaxBin + 1) * 2 > numElevationBins_) {
        throw std::runtime_error("Invalid elevation max bin: expected value in [0; " + std::to_string(numElevationBins_ / 2) + "), got " + std::to_string(elMaxBin));
    }
    return elevationBins_[numElevationBins_ / 2 + elMaxBin] * 2 * 180.0f / M_PI;
}
float coloradar::RadarConfig::rangeIdxToRange(const int& rangeMaxBin) const {
    if (rangeMaxBin < 0 || rangeMaxBin >= nRangeBins()) {
        throw std::runtime_error("Invalid range max bin: expected value in [0; " + std::to_string(nRangeBins()) + "), got " + std::to_string(rangeMaxBin));
    }
    return (rangeMaxBin + 1) * rangeBinWidth_;
}
int coloradar::RadarConfig::horizontalFovToAzimuthIdx(const float& horizontalFov) const {
    if (horizontalFov <= 0 || horizontalFov > 360) {
        throw std::runtime_error("Invalid horizontal FOV value: expected 0 < FOV <= 360, got " + std::to_string(horizontalFov));
    }
    float horizontalHalfFovRad = horizontalFov / 2 * M_PI / 180.0f;
    auto it = std::lower_bound(azimuthBins_.begin(), azimuthBins_.end(), -horizontalHalfFovRad);
    int binIdx = std::distance(azimuthBins_.begin(), --it);
    return numAzimuthBins_ / 2 - binIdx - 1;
}
int coloradar::RadarConfig::verticalFovToElevationIdx(const float& verticalFov) const {
    if (verticalFov <= 0 || verticalFov > 180) {
        throw std::runtime_error("Invalid vertical FOV value: expected 0 < FOV <= 180, got " + std::to_string(verticalFov));
    }
    float verticalHalfFovRad = verticalFov / 2 * M_PI / 180.0f;
    auto it = std::lower_bound(elevationBins_.begin(), elevationBins_.end(), -verticalHalfFovRad);
    int binIdx = std::distance(elevationBins_.begin(), --it);
    return numElevationBins_ / 2 - binIdx - 1;
}
int coloradar::RadarConfig::rangeToRangeIdx(const float& range) const {
    if (range <= 0) {
        throw std::runtime_error("RadarConfig::rangeToRangeIdx(): Invalid max range value: expected R > 0, got " + std::to_string(range));
    }
    return static_cast<int>(std::ceil(range / rangeBinWidth_) - 1);
}
