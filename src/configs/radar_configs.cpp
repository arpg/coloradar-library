#include "configs/radar_configs.h"


namespace coloradar {


void RadarConfig::precomputePointcloudTemplate() {
    const size_t totalSize = numElevationBins * numAzimuthBins * nRangeBins();
    pointcloudTemplate.reset(new pcl::PointCloud<RadarPoint>);
    pointcloudTemplate->resize(totalSize);

    std::vector<float> cosAzimuths(numAzimuthBins), sinAzimuths(numAzimuthBins);
    for (int azIdx = 0; azIdx < numAzimuthBins; ++azIdx) {
        cosAzimuths[azIdx] = std::cos(azimuthBins[azIdx]);
        sinAzimuths[azIdx] = std::sin(azimuthBins[azIdx]);
    }
    std::vector<float> cosElevations(numElevationBins), sinElevations(numElevationBins);
    for (int elIdx = 0; elIdx < numElevationBins; ++elIdx) {
        cosElevations[elIdx] = std::cos(elevationBins[elIdx]);
        sinElevations[elIdx] = std::sin(elevationBins[elIdx]);
    }

    size_t pointIdx = 0;
    for (int elIdx = 0; elIdx < numElevationBins; ++elIdx) {
        for (int azIdx = 0; azIdx < numAzimuthBins; ++azIdx) {
            for (int rangeIdx = 0; rangeIdx < nRangeBins(); ++rangeIdx, ++pointIdx) {
                float range = rangeIdx * rangeBinWidth;
                RadarPoint& point = (*pointcloudTemplate)[pointIdx];
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
    return numElevationBins * numAzimuthBins * nRangeBins() * (hasDoppler ? 2 : 1);
}


pcl::PointCloud<RadarPoint>::Ptr RadarConfig::heatmapToPointcloud(const std::shared_ptr<std::vector<float>>& heatmap, const double intensityThreshold) const {
    const size_t expectedCloudSize = numElevationBins * numAzimuthBins * nRangeBins();
    if (heatmap->size() != heatmapSize()) throw std::runtime_error("Heatmap size mismatch. Expected: " + std::to_string(heatmapSize()) + ", got: " + std::to_string(heatmap->size()));
    if (!pointcloudTemplate || pointcloudTemplate->size() != expectedCloudSize) throw std::runtime_error("Pointcloud template size mismatch. Expected: " + std::to_string(expectedCloudSize) + ", got: " + std::to_string(pointcloudTemplate->size()));

    pcl::PointCloud<RadarPoint>::Ptr outputCloud(new pcl::PointCloud<RadarPoint>);
    outputCloud->reserve(pointcloudTemplate->size());
    size_t heatmapIdx = 0;

    for (size_t pointIdx = 0; pointIdx < pointcloudTemplate->size(); ++pointIdx) {
        const float intensity = heatmap->at(heatmapIdx++);
        const float doppler   = heatmap->at(heatmapIdx++);
        if (intensity >= intensityThreshold) {
            RadarPoint point = (*pointcloudTemplate)[pointIdx];
            point.intensity = intensity;
            point.doppler = doppler;
            outputCloud->push_back(point);
        }
    }
    return outputCloud;
}


std::shared_ptr<std::vector<float>> RadarConfig::clipHeatmap(
    const std::shared_ptr<std::vector<float>>& heatmap,
    int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin, bool updateConfig)
{
    azimuthMaxBin   = clipAzimuthMaxBin(azimuthMaxBin);
    elevationMaxBin = clipElevationMaxBin(elevationMaxBin);
    rangeMaxBin     = clipRangeMaxBin(rangeMaxBin);
    if (azimuthMaxBin == numAzimuthBins / 2 && elevationMaxBin == numElevationBins / 2 && rangeMaxBin == nRangeBins() - 1) return heatmap;

    int azimuthBinLimit   = numAzimuthBins / 2 - 1;
    int azimuthLeftBin    = azimuthBinLimit - azimuthMaxBin;
    int azimuthRightBin   = azimuthBinLimit + azimuthMaxBin + 1;

    int elevationBinLimit = numElevationBins / 2 - 1;
    int elevationLeftBin  = elevationBinLimit - elevationMaxBin;
    int elevationRightBin = elevationBinLimit + elevationMaxBin + 1;

    auto clipped = std::make_shared<std::vector<float>>();
    for (int e = elevationLeftBin; e <= elevationRightBin; ++e) {
        for (int a = azimuthLeftBin; a <= azimuthRightBin; ++a) {
            for (int r = 0; r <= rangeMaxBin; ++r) {
                int index = ((e * numAzimuthBins + a) * nRangeBins() + r) * 2;
                if (hasDoppler) {
                    for (int n = 0; n < 2; ++n) clipped->push_back((*heatmap)[index + n]);
                }
                else clipped->push_back((*heatmap)[index]);
            }
        }
    }
    if (updateConfig) {
        numAzimuthBins = azimuthRightBin - azimuthLeftBin + 1;
        numElevationBins = elevationRightBin - elevationLeftBin + 1;
        setNumRangeBins(rangeMaxBin + 1);
        azimuthBins = std::vector<float>(azimuthBins.begin() + azimuthLeftBin,   azimuthBins.begin() + azimuthRightBin);
        elevationBins = std::vector<float>(elevationBins.begin() + elevationLeftBin, elevationBins.begin() + elevationRightBin);
    }
    // std::cout << "clipHeatmap(): clipped->size() = " << clipped->size() << ", nRangeBins(): " << nRangeBins() << std::endl;
    // std::cout << "elevationLeftBin: " << elevationLeftBin << ", elevationRightBin: " << elevationRightBin << ", numElevationBins: " << numElevationBins << std::endl;
    // std::cout << "azimuthLeftBin: " << azimuthLeftBin << ", azimuthRightBin: " << azimuthRightBin << ", numAzimuthBins: " << numAzimuthBins << std::endl;
    return clipped;
}

std::shared_ptr<std::vector<float>> RadarConfig::clipHeatmap(
    const std::shared_ptr<std::vector<float>>& heatmap,
    float horizontalFov, float verticalFov, float range, bool updateConfig)
{
    int azMaxBin = horizontalFovToAzimuthIdx(horizontalFov);
    int elMaxBin = verticalFovToElevationIdx(verticalFov);
    int rangeMaxBin = rangeToRangeIdx(range);
    return clipHeatmap(heatmap, azMaxBin, elMaxBin, rangeMaxBin, updateConfig);
}


std::shared_ptr<std::vector<float>> RadarConfig::collapseHeatmapElevation(
    const std::shared_ptr<std::vector<float>>& image,
    const float& elevationMinMeters, const float& elevationMaxMeters, bool updateConfig)
{
    if (elevationMaxMeters < elevationMinMeters) throw std::out_of_range("elevationMaxMeters must be greater or equal to elevationMinMeters.");
    auto collapsedHeatmap = std::make_shared<std::vector<float>>(numAzimuthBins * nRangeBins() * 2);

    for (int a = 0; a < numAzimuthBins; ++a) {
        for (int r = 0; r < nRangeBins(); ++r) {
            float maxIntensity = -std::numeric_limits<float>::infinity();
            float maxDoppler   = 0.0f;
            for (int e = 0; e < elevationBins.size(); ++e) {
                float z = r * std::sin(elevationBins[e]);
                if (z >= elevationMinMeters && z <= elevationMaxMeters) {
                    int index = (((e * numAzimuthBins + a) * nRangeBins() + r) * 2);
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
    if (updateConfig) {
        numElevationBins = 0;
        elevationBins = {};
    }
    return collapsedHeatmap;
}

std::shared_ptr<std::vector<float>> RadarConfig::removeDoppler(const std::shared_ptr<std::vector<float>>& image, bool updateConfig) {
    auto intensityImage = std::make_shared<std::vector<float>>(image->size() / 2);
    for (size_t i = 0; i < image->size(); i += 2) {
        (*intensityImage)[i] = (*image)[i];
    }
    if (updateConfig) {
        hasDoppler = false;
    }
    return intensityImage;
}

std::shared_ptr<std::vector<float>> RadarConfig::swapHeatmapDimensions(const std::shared_ptr<std::vector<float>>& heatmap) {
    const size_t numDims = hasDoppler ? 2 : 1;
    const size_t expectedSize = numElevationBins * numAzimuthBins * nRangeBins() * numDims;
    if (heatmap->size() != expectedSize) {
        throw std::runtime_error(
            "RadarConfig::swapHeatmapDimensions(): heatmap size does not match the expected dimensions. Expected size: " +
            std::to_string(expectedSize) + ", Actual size: " + std::to_string(heatmap->size())
        );
    }

    auto reorganizedHeatmap = std::make_shared<std::vector<float>>(expectedSize);
    for (int el = 0; el < numElevationBins; ++el) {
        for (int az = 0; az < numAzimuthBins; ++az) {
            for (int r = 0; r < nRangeBins(); ++r) {
                for (int d = 0; d < numDims; ++d) {
                    size_t originalIndex    = (((el * numAzimuthBins + az) * nRangeBins()) + r) * numDims + d;
                    size_t reorganizedIndex = (((az * nRangeBins() + r) * numElevationBins) + el) * numDims + d;
                    (*reorganizedHeatmap)[reorganizedIndex] = (*heatmap)[originalIndex];
                }
            }
        }
    }
    return reorganizedHeatmap;
}


Json::Value RadarConfig::toJson() const {
    Json::Value jsonConfig;

    // Heatmap parameters
    jsonConfig["heatmap"]["numRangeBins"]     = numRangeBins;     
    jsonConfig["heatmap"]["numPosRangeBins"]  = numPosRangeBins; 
    jsonConfig["heatmap"]["numElevationBins"] = numElevationBins;
    jsonConfig["heatmap"]["numAzimuthBins"]   = numAzimuthBins;
    jsonConfig["heatmap"]["rangeBinWidth"]    = rangeBinWidth;
    jsonConfig["heatmap"]["azimuthBins"]      = Json::arrayValue;
    for (const auto& bin : azimuthBins) jsonConfig["heatmap"]["azimuthBins"].append(bin);
    jsonConfig["heatmap"]["elevationBins"]    = Json::arrayValue;
    for (const auto& bin : elevationBins) jsonConfig["heatmap"]["elevationBins"].append(bin);

    // Antenna parameters
    jsonConfig["antenna"]["designFrequency"] = designFrequency;
    jsonConfig["antenna"]["numTxAntennas"]   = numTxAntennas;
    jsonConfig["antenna"]["numRxAntennas"]   = numRxAntennas;
    jsonConfig["antenna"]["txCenters"]       = Json::arrayValue;
    for (const auto& center : txCenters) {
        Json::Value p; p["x"] = center.x; p["y"] = center.y;
        jsonConfig["antenna"]["txCenters"].append(p);
    }
    jsonConfig["antenna"]["rxCenters"] = Json::arrayValue;
    for (const auto& center : rxCenters) {
        Json::Value p; p["x"] = center.x; p["y"] = center.y;
        jsonConfig["antenna"]["rxCenters"].append(p);
    }

    // Waveform parameters
    jsonConfig["waveform"]["numAdcSamplesPerChirp"] = numAdcSamplesPerChirp;
    jsonConfig["waveform"]["numChirpsPerFrame"]     = numChirpsPerFrame;
    jsonConfig["waveform"]["adcSampleFrequency"]    = adcSampleFrequency;
    jsonConfig["waveform"]["startFrequency"]        = startFrequency;
    jsonConfig["waveform"]["idleTime"]              = idleTime;
    jsonConfig["waveform"]["adcStartTime"]          = adcStartTime;
    jsonConfig["waveform"]["rampEndTime"]           = rampEndTime;
    jsonConfig["waveform"]["frequencySlope"]        = frequencySlope;

    // Calibration parameters
    jsonConfig["calibration"]["numDopplerBins"]     = numDopplerBins;
    jsonConfig["calibration"]["couplingCalibMatrix"] = Json::arrayValue;
    for (const auto& z : couplingCalibMatrix) {
        Json::Value c; c["real"] = z.real(); c["imag"] = z.imag();
        jsonConfig["calibration"]["couplingCalibMatrix"].append(c);
    }

    // Phase frequency parameters
    jsonConfig["phaseFrequency"]["calibAdcSampleFrequency"] = calibAdcSampleFrequency;
    jsonConfig["phaseFrequency"]["calibFrequencySlope"]     = calibFrequencySlope;
    jsonConfig["phaseFrequency"]["frequencyCalibMatrix"]    = Json::arrayValue;
    for (const auto& z : frequencyCalibMatrix) {
        Json::Value c; c["real"] = z.real(); c["imag"] = z.imag();
        jsonConfig["phaseFrequency"]["frequencyCalibMatrix"].append(c);
    }
    jsonConfig["phaseFrequency"]["phaseCalibMatrix"] = Json::arrayValue;
    for (const auto& z : phaseCalibMatrix) {
        Json::Value c; c["real"] = z.real(); c["imag"] = z.imag();
        jsonConfig["phaseFrequency"]["phaseCalibMatrix"].append(c);
    }

    // Internal parameters
    jsonConfig["internal"]["numAzimuthBeams"]     = numAzimuthBeams;
    jsonConfig["internal"]["numElevationBeams"]   = numElevationBeams;
    jsonConfig["internal"]["azimuthApertureLen"]  = azimuthApertureLen;
    jsonConfig["internal"]["elevationApertureLen"]= elevationApertureLen;
    jsonConfig["internal"]["numAngles"]           = numAngles;
    jsonConfig["internal"]["numVirtualElements"]  = numVirtualElements;
    jsonConfig["internal"]["virtualArrayMap"]     = Json::arrayValue;
    for (const auto& v : virtualArrayMap) jsonConfig["internal"]["virtualArrayMap"].append(v);
    jsonConfig["internal"]["azimuthAngles"]       = Json::arrayValue;
    for (const auto& a : azimuthAngles) jsonConfig["internal"]["azimuthAngles"].append(a);
    jsonConfig["internal"]["elevationAngles"]     = Json::arrayValue;
    for (const auto& a : elevationAngles) jsonConfig["internal"]["elevationAngles"].append(a);
    jsonConfig["internal"]["hasDoppler"]          = hasDoppler;
    jsonConfig["internal"]["dopplerBinWidth"]     = dopplerBinWidth;

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

    numRangeBins     = H["numRangeBins"].asInt();
    numPosRangeBins  = H["numPosRangeBins"].asInt();
    // numPosRangeBins  = H["numRangeBins"].asInt();
    // numRangeBins     = numPosRangeBins * 2;
    numElevationBins = H["numElevationBins"].asInt();
    numAzimuthBins   = H["numAzimuthBins"].asInt();
    rangeBinWidth    = H["rangeBinWidth"].asDouble();

    azimuthBins.clear();
    azimuthBins.reserve(H["azimuthBins"].size());
    for (const auto& v : H["azimuthBins"]) {
        if (!v.isNumeric()) throw std::invalid_argument("heatmap.azimuthBins: all values must be numbers.");
        azimuthBins.push_back(static_cast<float>(v.asDouble()));
    }
    elevationBins.clear();
    elevationBins.reserve(H["elevationBins"].size());
    for (const auto& v : H["elevationBins"]) {
        if (!v.isNumeric()) throw std::invalid_argument("heatmap.elevationBins: all values must be numbers.");
        elevationBins.push_back(static_cast<float>(v.asDouble()));
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

    designFrequency = A["designFrequency"].asDouble();
    numTxAntennas   = A["numTxAntennas"].asInt();
    numRxAntennas   = A["numRxAntennas"].asInt();

    txCenters.clear();
    txCenters.reserve(A["txCenters"].size());
    for (const auto& p : A["txCenters"]) {
        if (!p.isObject() || !p.isMember("x") || !p.isMember("y") || !p["x"].isNumeric() || !p["y"].isNumeric())
            throw std::invalid_argument("antenna.txCenters: each entry must be {x:number, y:number}.");
        pcl::PointXY pt; pt.x = static_cast<float>(p["x"].asDouble()); pt.y = static_cast<float>(p["y"].asDouble());
        txCenters.push_back(pt);
    }
    rxCenters.clear();
    rxCenters.reserve(A["rxCenters"].size());
    for (const auto& p : A["rxCenters"]) {
        if (!p.isObject() || !p.isMember("x") || !p.isMember("y") || !p["x"].isNumeric() || !p["y"].isNumeric())
            throw std::invalid_argument("antenna.rxCenters: each entry must be {x:number, y:number}.");
        pcl::PointXY pt; pt.x = static_cast<float>(p["x"].asDouble()); pt.y = static_cast<float>(p["y"].asDouble());
        rxCenters.push_back(pt);
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

    numAdcSamplesPerChirp = W["numAdcSamplesPerChirp"].asInt();
    numChirpsPerFrame     = W["numChirpsPerFrame"].asInt();
    adcSampleFrequency    = W["adcSampleFrequency"].asInt();
    startFrequency        = W["startFrequency"].asDouble();
    idleTime              = W["idleTime"].asDouble();
    adcStartTime          = W["adcStartTime"].asDouble();
    rampEndTime           = W["rampEndTime"].asDouble();
    frequencySlope        = W["frequencySlope"].asDouble();

    // ---- calibration
    if (!root.isMember("calibration") || !root["calibration"].isObject())
        throw std::invalid_argument("RadarConfig::fromJson: missing 'calibration' object.");
    const auto& C = root["calibration"];

    if (!C.isMember("numDopplerBins") || !C["numDopplerBins"].isInt())
        throw std::invalid_argument("calibration.numDopplerBins missing or not int.");
    if (!C.isMember("couplingCalibMatrix") || !C["couplingCalibMatrix"].isArray())
        throw std::invalid_argument("calibration.couplingCalibMatrix missing or not array.");

    numDopplerBins = C["numDopplerBins"].asInt();
    couplingCalibMatrix.clear();
    couplingCalibMatrix.reserve(C["couplingCalibMatrix"].size());
    for (const auto& z : C["couplingCalibMatrix"]) {
        if (!z.isObject() || !z.isMember("real") || !z.isMember("imag") ||
            !z["real"].isNumeric() || !z["imag"].isNumeric())
            throw std::invalid_argument("calibration.couplingCalibMatrix elements must be {real,imag} numbers.");
        couplingCalibMatrix.emplace_back(z["real"].asDouble(), z["imag"].asDouble());
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

    calibAdcSampleFrequency = P["calibAdcSampleFrequency"].asInt();
    calibFrequencySlope     = P["calibFrequencySlope"].asDouble();

    frequencyCalibMatrix.clear();
    frequencyCalibMatrix.reserve(P["frequencyCalibMatrix"].size());
    for (const auto& z : P["frequencyCalibMatrix"]) {
        if (!z.isObject() || !z.isMember("real") || !z.isMember("imag") ||
            !z["real"].isNumeric() || !z["imag"].isNumeric())
            throw std::invalid_argument("phaseFrequency.frequencyCalibMatrix elements must be {real,imag} numbers.");
        frequencyCalibMatrix.emplace_back(z["real"].asDouble(), z["imag"].asDouble());
    }
    phaseCalibMatrix.clear();
    phaseCalibMatrix.reserve(P["phaseCalibMatrix"].size());
    for (const auto& z : P["phaseCalibMatrix"]) {
        if (!z.isObject() || !z.isMember("real") || !z.isMember("imag") ||
            !z["real"].isNumeric() || !z["imag"].isNumeric())
            throw std::invalid_argument("phaseFrequency.phaseCalibMatrix elements must be {real,imag} numbers.");
        phaseCalibMatrix.emplace_back(z["real"].asDouble(), z["imag"].asDouble());
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

    numAzimuthBeams     = I["numAzimuthBeams"].asInt();
    numElevationBeams   = I["numElevationBeams"].asInt();
    azimuthApertureLen  = I["azimuthApertureLen"].asInt();
    elevationApertureLen= I["elevationApertureLen"].asInt();
    numAngles           = I["numAngles"].asInt();
    numVirtualElements  = I["numVirtualElements"].asInt();

    virtualArrayMap.clear();
    virtualArrayMap.reserve(I["virtualArrayMap"].size());
    for (const auto& v : I["virtualArrayMap"]) {
        if (!v.isInt()) throw std::invalid_argument("internal.virtualArrayMap: all values must be ints.");
        virtualArrayMap.push_back(v.asInt());
    }

    azimuthAngles.clear();
    azimuthAngles.reserve(I["azimuthAngles"].size());
    for (const auto& a : I["azimuthAngles"]) {
        if (!a.isNumeric()) throw std::invalid_argument("internal.azimuthAngles: all values must be numbers.");
        azimuthAngles.push_back(static_cast<float>(a.asDouble()));
    }
    elevationAngles.clear();
    elevationAngles.reserve(I["elevationAngles"].size());
    for (const auto& a : I["elevationAngles"]) {
        if (!a.isNumeric()) throw std::invalid_argument("internal.elevationAngles: all values must be numbers.");
        elevationAngles.push_back(static_cast<float>(a.asDouble()));
    }

    hasDoppler      = I["hasDoppler"].asBool();
    dopplerBinWidth = I["dopplerBinWidth"].asDouble();

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
    rxCenters.clear();
    txCenters.clear();

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
        numRxAntennas = std::stoi(token);
        rxCenters.resize(numRxAntennas);
        hasNumRx = true;
      }
      if (token.compare("num_tx") == 0)
      {
        std::getline(ss, token, ' ');
        numTxAntennas = std::stoi(token);
        txCenters.resize(numTxAntennas);
        hasNumTx = true;
      }
      if (token.compare("F_design") == 0)
      {
        std::getline(ss, token, ' ');
        designFrequency = std::stoi(token) * 1e9;  // convert from GHz to Hz
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
        rxCenters[idx] = rx_center;
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
        txCenters[idx] = tx_center;
        tx_count++;
      }
    }
    if (!hasNumRx || !hasNumTx || !hasDesignF) {
        throw std::runtime_error("Missing num_rx or num_tx or F_design in antenna config.");
    }
    if (rx_count != rxCenters.size())
    {
      throw std::runtime_error("antenna config specified num_rx = " + std::to_string(rxCenters.size()) + " but only " + std::to_string(rx_count) + " rx positions found.");
    }
    if (tx_count != txCenters.size())
    {
      throw std::runtime_error("antenna config specified num_tx = " + std::to_string(txCenters.size()) + " but only " + std::to_string(tx_count) + " tx positions found.");
    }
}

void coloradar::RadarConfig::initHeatmapParams(const std::filesystem::path& heatmapCfgFile) {
    auto configMap = readConfig(heatmapCfgFile);
    azimuthBins.clear();
    elevationBins.clear();

    auto it = configMap.find("num_range_bins");
    if (it != configMap.end()) {
        numPosRangeBins = std::stoi(it->second[0]);
        numRangeBins = numPosRangeBins * 2;
    } else {
        throw std::runtime_error("Missing num_range_bins in heatmap config.");
    }
    it = configMap.find("num_elevation_bins");
    if (it != configMap.end()) {
        numElevationBins = std::stoi(it->second[0]);
    }
    it = configMap.find("num_azimuth_bins");
    if (it != configMap.end()) {
        numAzimuthBins = std::stoi(it->second[0]);
    }
    it = configMap.find("range_bin_width");
    if (it != configMap.end()) {
        rangeBinWidth = std::stod(it->second[0]);
    }
    it = configMap.find("azimuth_bins");
    if (it != configMap.end()) {
        for (const auto& bin : it->second) {
            azimuthBins.push_back(std::stof(bin));
        }
    }
    it = configMap.find("elevation_bins");
    if (it != configMap.end()) {
        for (const auto& bin : it->second) {
            elevationBins.push_back(std::stof(bin));
        }
    }
}


void coloradar::RadarConfig::initWaveformParams(const std::filesystem::path& waveformCfgFile) {
    auto configMap = readConfig(waveformCfgFile);

    auto it = configMap.find("num_adc_samples_per_chirp");
    if (it != configMap.end()) {
        numAdcSamplesPerChirp = std::stoi(it->second[0]);
    } else {
        throw std::runtime_error("Missing num_adc_samples_per_chirp in waveform config.");
    }
    it = configMap.find("num_chirps_per_frame");
    if (it != configMap.end()) {
        numChirpsPerFrame = std::stoi(it->second[0]);
    }
    it = configMap.find("adc_sample_frequency");
    if (it != configMap.end()) {
        adcSampleFrequency = std::stod(it->second[0]);
    }
    it = configMap.find("start_frequency");
    if (it != configMap.end()) {
        startFrequency = std::stod(it->second[0]);
    }
    it = configMap.find("idle_time");
    if (it != configMap.end()) {
        idleTime = std::stod(it->second[0]);
    }
    it = configMap.find("adc_start_time");
    if (it != configMap.end()) {
        adcStartTime = std::stod(it->second[0]);
    }
    it = configMap.find("ramp_end_time");
    if (it != configMap.end()) {
        rampEndTime = std::stod(it->second[0]);
    }
    it = configMap.find("frequency_slope");
    if (it != configMap.end()) {
        frequencySlope = std::stod(it->second[0]);
    }
}

void coloradar::RadarConfig::initCouplingParams(const std::filesystem::path& couplingCfgFile) {
    std::ifstream file(couplingCfgFile);
    if (!file.is_open())
        throw std::runtime_error("Unable to open coupling config file: " + couplingCfgFile.string());
    std::string line, token;
    std::stringstream ss;
    couplingCalibMatrix.clear();

    while (std::getline(file, line)) {
        ss.clear();
        ss.str(line);
        std::getline(ss, token, ':');
        if (token == "num_doppler_bins") {
            std::getline(ss, token);
            numDopplerBins = std::stoi(token);
        }
        if (token == "data") {
            std::vector<double> values;
            while (std::getline(ss, token, ','))
                values.push_back(std::stod(token));
            size_t expectedSize = numTxAntennas * numRxAntennas * numPosRangeBins * 2;
            if (values.size() != expectedSize)
                throw std::runtime_error("Mismatch in the size of the coupling calibration matrix. Expected: " + std::to_string(expectedSize) + ", Found: " + std::to_string(values.size()));
            couplingCalibMatrix.resize(values.size() / 2);
            for (size_t i = 0; i < values.size(); i += 2) {
                couplingCalibMatrix[i / 2] = std::complex<double>(values[i], values[i + 1]);
            }
        }
    }
    file.close();
    if (couplingCalibMatrix.empty())
        throw std::runtime_error("Coupling calibration data not found in config.");
}


void coloradar::RadarConfig::initPhaseFrequencyParams(const std::filesystem::path& phaseFrequencyCfgFile) {
    const Json::Value& configMap = readJsonConfig(phaseFrequencyCfgFile);
    if (!configMap.isMember("antennaCalib")) {
        throw std::runtime_error("Missing antennaCalib in phase frequency config.");
    }
    const Json::Value& config = configMap["antennaCalib"];

    std::vector<double> freqData(numTxAntennas * numRxAntennas);
    std::vector<std::complex<double>> phaseData(numTxAntennas * numRxAntennas);

    if (config.isMember("frequencySlope")) {
        calibFrequencySlope = config["frequencySlope"].asDouble();
    } else {
        throw std::runtime_error("Missing frequencySlope in phase frequency config.");
    }
    if (config.isMember("samplingRate")) {
        calibAdcSampleFrequency = config["samplingRate"].asInt();
    } else {
        throw std::runtime_error("Missing samplingRate in phase frequency config.");
    }

    if (config.isMember("frequencyCalibrationMatrix")) {
        const Json::Value& frequencyMatrix = config["frequencyCalibrationMatrix"];
        if (frequencyMatrix.size() != numTxAntennas * numRxAntennas) {
            throw std::runtime_error("Invalid frequency calibration array: expected " + std::to_string(numRxAntennas * numTxAntennas) + " elements, got " + std::to_string(frequencyMatrix.size()));
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
        if (phaseMatrix.size() / 2 != numTxAntennas * numRxAntennas) {
            throw std::runtime_error("Invalid phase calibration array: expected " + std::to_string(numRxAntennas * numTxAntennas) + " elements, got " + std::to_string(phaseMatrix.size()));
        }
        for (Json::ArrayIndex i = 0; i < phaseMatrix.size(); i += 2) {
            double real = phaseMatrix[i].asDouble();
            double imag = phaseMatrix[i + 1].asDouble();
            phaseData[i / 2] = std::complex<double>(real, imag);
        }
    } else {
        throw std::runtime_error("Missing phaseCalibrationMatrix in phase frequency config.");
    }

    frequencyCalibMatrix.clear();
    frequencyCalibMatrix.resize(numTxAntennas * numRxAntennas * numRangeBins);
    phaseCalibMatrix.clear();
    phaseCalibMatrix.resize(numTxAntennas * numRxAntennas);

    for (int tx_idx = 0; tx_idx < numTxAntennas; tx_idx++) {
        for (int rx_idx = 0; rx_idx < numRxAntennas; rx_idx++) {
            int idx = rx_idx + (tx_idx * numRxAntennas);
            double delta_p = freqData[idx] - freqData[0];
            double freq_calib = 2.0 * M_PI * delta_p / numRangeBins * (frequencySlope / calibFrequencySlope) * (adcSampleFrequency / calibAdcSampleFrequency);
            for (int sample_idx = 0; sample_idx < numRangeBins; sample_idx++) {
                int cal_idx = sample_idx + numRangeBins * (rx_idx + numRxAntennas * tx_idx);
                frequencyCalibMatrix[cal_idx] = std::exp(std::complex<double>(0.0, -1.0) * std::complex<double>(freq_calib, 0.0) * std::complex<double>(sample_idx, 0.0));
            }
        }
    }
    std::complex<double> phase_ref = phaseData[0];
    for (int tx_idx = 0; tx_idx < numTxAntennas; tx_idx++) {
        for (int rx_idx = 0; rx_idx < numRxAntennas; rx_idx++) {
            int idx = rx_idx + (tx_idx * numRxAntennas);
            phaseCalibMatrix[idx] = phase_ref / phaseData[idx];
        }
    }
}


void coloradar::RadarConfig::initInternalParams() {
    azimuthApertureLen = 0;
    elevationApertureLen = 0;
    numVirtualElements = 0;
    virtualArrayMap.clear();
    azimuthAngles.clear();
    elevationAngles.clear();
    azimuthAngles.resize(numAzimuthBeams);
    elevationAngles.resize(numElevationBeams);
    numAngles = numAzimuthBeams * numElevationBeams;

    for (int tx_idx = 0; tx_idx < numTxAntennas; tx_idx++)
    {
      for (int rx_idx = 0; rx_idx < numRxAntennas; rx_idx++)
      {
        int virtual_x = rxCenters[rx_idx].x + txCenters[tx_idx].x;
        int virtual_y = rxCenters[rx_idx].y + txCenters[tx_idx].y;
        // check to ensure this antenna pair doesn't map to the same virtual
        // location as a previously evaluated antenna pair
        bool redundant = false;
        for (int i = 0; i < numVirtualElements; i++)
        {
          int idx = i * 4;
          if (virtualArrayMap[idx] == virtual_x
            && virtualArrayMap[idx+1] == virtual_y)
            redundant = true;
        }
        // record mapping from antenna pair index to virtual antenna location
        // stored in vector with entries grouped into 4-tuples of
        // [azimuth_location, elevation_location, rx_index, tx_index]
        if (!redundant)
        {
          if (virtual_x + 1 > azimuthApertureLen)
            azimuthApertureLen = virtual_x + 1;
          if (virtual_y + 1 > elevationApertureLen)
            elevationApertureLen = virtual_y + 1;

          virtualArrayMap.push_back(virtual_x);
          virtualArrayMap.push_back(virtual_y);
          virtualArrayMap.push_back(rx_idx);
          virtualArrayMap.push_back(tx_idx);
          numVirtualElements++;
        }
      }
    }
    double wavelength = c / (startFrequency + adcStartTime * frequencySlope);
    double chirp_time = idleTime + rampEndTime;
    double v_max = wavelength / (4.0 * numTxAntennas * chirp_time);
    dopplerBinWidth = v_max / numDopplerBins;

    double center_frequency = startFrequency + numRangeBins / adcSampleFrequency * frequencySlope / 2.0;
    double d = 0.5 * center_frequency / designFrequency;
    double az_d_phase = (2. * M_PI) / numAzimuthBeams;
    double phase_dif = (az_d_phase / 2.) - M_PI;
    for (int i = 0; i < numAzimuthBeams; i++) {
      azimuthAngles[i] = asin(phase_dif / (2. * M_PI * d));
      phase_dif += az_d_phase;
    }
    double el_d_phase = (2.*M_PI) / numElevationBeams;
    phase_dif = (el_d_phase / 2.) - M_PI;
    for (int i = 0; i < numElevationBeams; i++) {
        elevationAngles[i] = asin(phase_dif / (2. * M_PI * d));
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


void coloradar::RadarConfig::setNumRangeBins(const int& num) { numPosRangeBins = num; }
const int& coloradar::RadarConfig::nRangeBins() const { return numPosRangeBins; }
float coloradar::RadarConfig::maxRange() const { return std::ceil(nRangeBins() * rangeBinWidth * 100.0f) / 100.0f; }

int coloradar::RadarConfig::clipAzimuthMaxBin(const int& azMaxBin) { 
    return azMaxBin >= 0 && (azMaxBin + 1) * 2 < numAzimuthBins ? azMaxBin : numAzimuthBins / 2 - 1;
}
int coloradar::RadarConfig::clipElevationMaxBin(const int& elMaxBin) { 
    return elMaxBin >= 0 && (elMaxBin + 1) * 2 < numElevationBins ? elMaxBin : numElevationBins / 2 - 1;
}
int coloradar::RadarConfig::clipRangeMaxBin(const int& rangeMaxBin) {
    return rangeMaxBin >= 0 && rangeMaxBin < nRangeBins() ? rangeMaxBin : nRangeBins() - 1;
}
float coloradar::RadarConfig::clipRange(const float& range) { 
    return range > 0 && range <= maxRange() ? range : maxRange(); 
}
float coloradar::RadarConfig::azimuthIdxToFovDegrees(const int& azMaxBin) {
    if (azMaxBin < 0 || (azMaxBin + 1) * 2 > numAzimuthBins) {
        throw std::runtime_error("Invalid azimuth max bin: expected value in [0; " + std::to_string(numAzimuthBins / 2) + "), got " + std::to_string(azMaxBin));
    }
    return azimuthBins[numAzimuthBins / 2 + azMaxBin] * 2 * 180.0f / M_PI;
}
float coloradar::RadarConfig::elevationIdxToFovDegrees(const int& elMaxBin) {
    if (elMaxBin < 0 || (elMaxBin + 1) * 2 > numElevationBins) {
        throw std::runtime_error("Invalid elevation max bin: expected value in [0; " + std::to_string(numElevationBins / 2) + "), got " + std::to_string(elMaxBin));
    }
    return elevationBins[numElevationBins / 2 + elMaxBin] * 2 * 180.0f / M_PI;
}
float coloradar::RadarConfig::rangeIdxToRange(const int& rangeMaxBin) {
    if (rangeMaxBin < 0 || rangeMaxBin >= nRangeBins()) {
        throw std::runtime_error("Invalid range max bin: expected value in [0; " + std::to_string(nRangeBins()) + "), got " + std::to_string(rangeMaxBin));
    }
    return (rangeMaxBin + 1) * rangeBinWidth;
}
int coloradar::RadarConfig::horizontalFovToAzimuthIdx(const float& horizontalFov) {
    if (horizontalFov <= 0 || horizontalFov > 360) {
        throw std::runtime_error("Invalid horizontal FOV value: expected 0 < FOV <= 360, got " + std::to_string(horizontalFov));
    }
    float horizontalHalfFovRad = horizontalFov / 2 * M_PI / 180.0f;
    auto it = std::lower_bound(azimuthBins.begin(), azimuthBins.end(), -horizontalHalfFovRad);
    int binIdx = std::distance(azimuthBins.begin(), --it);
    return numAzimuthBins / 2 - binIdx - 1;
}
int coloradar::RadarConfig::verticalFovToElevationIdx(const float& verticalFov) {
    if (verticalFov <= 0 || verticalFov > 180) {
        throw std::runtime_error("Invalid vertical FOV value: expected 0 < FOV <= 180, got " + std::to_string(verticalFov));
    }
    float verticalHalfFovRad = verticalFov / 2 * M_PI / 180.0f;
    auto it = std::lower_bound(elevationBins.begin(), elevationBins.end(), -verticalHalfFovRad);
    int binIdx = std::distance(elevationBins.begin(), --it);
    return numElevationBins / 2 - binIdx - 1;
}
int coloradar::RadarConfig::rangeToRangeIdx(const float& range) {
    if (range <= 0) {
        throw std::runtime_error("RadarConfig::rangeToRangeIdx(): Invalid max range value: expected R > 0, got " + std::to_string(range));
    }
    return static_cast<int>(std::ceil(range / rangeBinWidth) - 1);
}
