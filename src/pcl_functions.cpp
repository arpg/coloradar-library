#include "utils.h"

#include <unordered_map>
#include <tuple>
#include <functional>


POINT_CLOUD_REGISTER_POINT_STRUCT (coloradar::RadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, doppler, doppler))


template <>
struct std::hash<std::tuple<float, float, float>> {
    std::size_t operator()(const std::tuple<float, float, float>& key) const {
        auto roundToPrecision = [](float value, float precision) {
            return std::round(value / precision) * precision;
        };
        std::size_t h1 = std::hash<float>{}(roundToPrecision(std::get<0>(key), 0.01f));
        std::size_t h2 = std::hash<float>{}(roundToPrecision(std::get<1>(key), 0.01f));
        std::size_t h3 = std::hash<float>{}(roundToPrecision(std::get<2>(key), 0.01f));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
inline std::tuple<float, float, float> makeKey(float x, float y, float z) {
    return std::make_tuple(x, y, z);
}


pcl::PointCloud<coloradar::RadarPoint> coloradar::heatmapToPointcloud(
    const std::vector<float>& heatmap,
    coloradar::RadarConfig* config,
    const float& intensityThresholdPercent)
{
    if (intensityThresholdPercent < 0 || intensityThresholdPercent >= 100) {
        throw std::runtime_error("Invalid intensityThresholdPercent: expected value in [0; 100), got " + std::to_string(intensityThresholdPercent));
    }
    float maxIntensity = 0.0f;
    pcl::PointCloud<coloradar::RadarPoint> cloud;

    std::vector<float> cosAzimuths(config->numAzimuthBeams);
    std::vector<float> sinAzimuths(config->numAzimuthBeams);
    std::vector<float> cosElevations(config->numElevationBeams);
    std::vector<float> sinElevations(config->numElevationBeams);
    for (int azIdx = 0; azIdx < config->numAzimuthBeams; ++azIdx) {
        cosAzimuths[azIdx] = std::cos(config->azimuthAngles[azIdx]);
        sinAzimuths[azIdx] = std::sin(config->azimuthAngles[azIdx]);
    }
    for (int elIdx = 0; elIdx < config->numElevationBeams; ++elIdx) {
        cosElevations[elIdx] = std::cos(config->elevationAngles[elIdx]);
        sinElevations[elIdx] = std::sin(config->elevationAngles[elIdx]);
    }

    std::unordered_map<std::tuple<float, float, float>, RadarPoint, std::hash<std::tuple<float, float, float>>> pointMap;

    // Iterate through all azimuth, range, and elevation bins
    for (int azIdx = 0; azIdx < config->numAzimuthBeams; ++azIdx) {
        for (int rangeIdx = 10; rangeIdx < config->numPosRangeBins; ++rangeIdx) {
            float range = rangeIdx * config->rangeBinWidth;

            for (int elIdx = 0; elIdx < config->numElevationBeams; ++elIdx) {
                // Compute the index in the heatmap
                int outIdx = 2 * (elIdx + config->numElevationBeams * (rangeIdx + config->numPosRangeBins * azIdx));
                float intensity = heatmap[outIdx];
                float doppler = heatmap[outIdx + 1];

                if (intensity > maxIntensity) {
                    maxIntensity = intensity;
                }

                // Compute Cartesian coordinates
                float x = range * cosElevations[elIdx] * sinAzimuths[azIdx];
                float y = range * cosElevations[elIdx] * cosAzimuths[azIdx];
                float z = range * sinElevations[elIdx];

                // Create the radar point
                RadarPoint point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity;
                point.doppler = doppler;

                // Use hash map to keep the most intense point per coordinate
                auto key = makeKey(point.x, point.y, point.z);
                auto it = pointMap.find(key);
                if (it != pointMap.end()) {
                    if (it->second.intensity < point.intensity) {
                        it->second = point;
                    }
                } else {
                    pointMap[key] = point;
                }
            }
        }
    }

    // Apply intensity threshold
    float intensityThreshold = maxIntensity * intensityThresholdPercent / 100;
    for (const auto& kv : pointMap) {
        if (kv.second.intensity >= intensityThreshold) {
            cloud.push_back(kv.second);
        }
    }

    return cloud;
}


void coloradar::convertRadarBinsToFov(int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin, const RadarConfig* config, float& horizontalFov, float& verticalFov, float& range) {
    azimuthMaxBin = azimuthMaxBin >= 0 && azimuthMaxBin < config->numAzimuthBins / 2 ? azimuthMaxBin : config->numAzimuthBins / 2 - 1;
    elevationMaxBin = elevationMaxBin >= 0 && elevationMaxBin < config->numElevationBins / 2 ? elevationMaxBin : config->numElevationBins / 2 - 1;
    rangeMaxBin = rangeMaxBin >= 0 && rangeMaxBin < config->numRangeBins ? rangeMaxBin : config->numRangeBins - 1;
    int azimuthRightBin = config->numAzimuthBins / 2 + azimuthMaxBin;
    int elevationRightBin = config->numElevationBins / 2 + elevationMaxBin;
    horizontalFov = config->azimuthBins[azimuthRightBin] * 2 * 180.0f / M_PI;
    verticalFov = config->elevationBins[elevationRightBin] * 2 * 180.0f / M_PI;
    range = (rangeMaxBin + 1) * config->rangeBinWidth;
}

void coloradar::convertFovToRadarBins(const float& horizontalFov, const float& verticalFov, const float& range, const coloradar::RadarConfig* config, int& azimuthMaxBin, int& elevationMaxBin, int& rangeMaxBin) {
    if (horizontalFov <= 0 || horizontalFov > 360) {
        throw std::runtime_error("Invalid horizontal FOV value: expected 0 < FOV <= 360, got " + std::to_string(horizontalFov));
    }
    if (verticalFov <= 0 || verticalFov > 180) {
        throw std::runtime_error("Invalid vertical FOV value: expected 0 < FOV <= 180, got " + std::to_string(verticalFov));
    }
    if (range <= 0) {
        throw std::runtime_error("Invalid max range value: expected R > 0, got " + std::to_string(range));
    }
    float horizontalHalfFovRad = horizontalFov / 2 * M_PI / 180.0f;
    float verticalHalfFovRad = verticalFov / 2 * M_PI / 180.0f;

    auto it = std::lower_bound(config->azimuthBins.begin(), config->azimuthBins.end(), -horizontalHalfFovRad);
    int binIdx = std::distance(config->azimuthBins.begin(), --it);
    azimuthMaxBin = config->numAzimuthBins - binIdx - 1;

    it = std::lower_bound(config->elevationBins.begin(), config->elevationBins.end(), -verticalHalfFovRad);
    binIdx = std::distance(config->elevationBins.begin(), --it);
    elevationMaxBin = config->numElevationBins - binIdx - 1;

    rangeMaxBin = static_cast<int>(std::ceil(range / config->rangeBinWidth));
}

std::vector<float> coloradar::clipHeatmapImage(const std::vector<float>& image, int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin, const coloradar::RadarConfig* config) {
    int azimuthBinLimit = config->numAzimuthBins / 2 - 1;
    int elevationBinLimit = config->numElevationBins / 2 - 1;
    int rangeBinLimit = config->numPosRangeBins - 1;
    azimuthMaxBin = azimuthMaxBin >= 0 && azimuthMaxBin <= azimuthBinLimit ? azimuthMaxBin : azimuthBinLimit;
    elevationMaxBin = elevationMaxBin >= 0 && elevationMaxBin <= elevationBinLimit ? elevationMaxBin : elevationBinLimit;
    rangeMaxBin = rangeMaxBin >= 0 && rangeMaxBin <= rangeBinLimit ? rangeMaxBin : rangeBinLimit;
    if (azimuthMaxBin == azimuthBinLimit && elevationMaxBin == elevationBinLimit && rangeMaxBin == rangeBinLimit) {
        return image;
    }
    int azimuthLeftBin = azimuthBinLimit - azimuthMaxBin;
    int azimuthRightBin = azimuthBinLimit + azimuthMaxBin + 1;
    int elevationLeftBin = elevationBinLimit - elevationMaxBin;
    int elevationRightBin = elevationBinLimit + elevationMaxBin + 1;
    std::vector<float> clipped;
    for (int e = elevationLeftBin; e <= elevationRightBin; ++e) {
        for (int a = azimuthLeftBin; a <= azimuthRightBin; ++a) {
            for (int r = 0; r <= rangeMaxBin; ++r) {
                for (int n = 0; n < 2; ++n) {
                    int index = (((e * config->numAzimuthBins + a) * config->numPosRangeBins + r) * 2) + n;
                    clipped.push_back(image[index]);
                }
            }
        }
    }
    return clipped;
}

std::vector<float> coloradar::clipHeatmapImage(const std::vector<float>& image, const float& horizontalFov, const float& verticalFov, const float& range, const coloradar::RadarConfig* config) {
    int azimuthMaxBin, elevationMaxBin, rangeMaxBin;
    coloradar::convertFovToRadarBins(horizontalFov, verticalFov, range, config, azimuthMaxBin, elevationMaxBin, rangeMaxBin);
    return coloradar::clipHeatmapImage(image, azimuthMaxBin, elevationMaxBin, rangeMaxBin, config);
}


void coloradar::convertBinsToElevationRange(int elevationMinBin, int elevationMaxBin, const std::vector<float>& elevationBins, float& elevationMinDeg, float& elevationMaxDeg) {
    int numElevationBins = elevationBins.size();
    elevationMinBin = elevationMinBin >= 0 && elevationMinBin < numElevationBins ? elevationMinBin : 0;
    elevationMaxBin = elevationMaxBin >= 0 && elevationMaxBin < numElevationBins ? elevationMaxBin : numElevationBins - 1;
    float elevation_min_rad = elevationBins[elevationMinBin];
    float elevation_max_rad = elevationBins[elevationMaxBin];
    elevationMinDeg = (elevation_min_rad * 180.0f / M_PI) + 90.0f;
    elevationMaxDeg = (elevation_max_rad * 180.0f / M_PI) + 90.0f;
}

void coloradar::convertElevationRangeToBins(float elevationMinDeg, float elevationMaxDeg, const std::vector<float>& elevationBins, int& elevationMinBin, int& elevationMaxBin) {
    if (elevationMinDeg < -90 || elevationMaxDeg > 90 || elevationMinDeg >= elevationMaxDeg) {
        throw std::runtime_error("Invalid elevation range: expected -90 <= min < max <= 90");
    }
    float elevation_min_rad = elevationMinDeg * M_PI / 180.0f;
    float elevation_max_rad = elevationMaxDeg * M_PI / 180.0f;

    auto min_it = std::lower_bound(elevationBins.begin(), elevationBins.end(), elevation_min_rad);
    elevationMinBin = std::distance(elevationBins.begin(), min_it);

    auto max_it = std::upper_bound(elevationBins.begin(), elevationBins.end(), elevation_max_rad);
    elevationMaxBin = std::distance(elevationBins.begin(), max_it) - 1;
}


//std::vector<float> coloradar::collapseHeatmapElevation(const std::vector<float>& image, int elevationMinBin, int elevationMaxBin, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins) {
//    elevationMinBin = elevationMinBin >= 0 && elevationMinBin < numElevationBins ? elevationMinBin : 0;
//    elevationMaxBin = elevationMaxBin >= 0 && elevationMaxBin < numElevationBins ? elevationMaxBin : numElevationBins - 1;
//    if (elevationMinBin == 0 && elevationMaxBin == numElevationBins)
//        return image;
//    if (elevationMinBin > elevationMaxBin)
//        throw std::out_of_range("Invalid elevation bins selected.");
//
//    std::vector<float> collapsed_image;
//    collapsed_image.reserve(numAzimuthBins * numRangeBins * 2);
//    for (int a = 0; a < numAzimuthBins; ++a) {
//        for (int r = 0; r < numRangeBins; ++r) {
//            float maxIntensity = -std::numeric_limits<float>::infinity(), maxDoppler;
//            for (int e = elevationMinBin; e <= elevationMaxBin; ++e) {
//                int index = (((e * numAzimuthBins + a) * numRangeBins + r) * 2);
//                if (image[index] > maxIntensity) {
//                    maxIntensity = image[index];
//                    maxDoppler = image[index + 1];
//                }
//            }
//            collapsed_image.push_back(maxIntensity);
//            collapsed_image.push_back(maxDoppler);
//        }
//    }
//    return collapsed_image;
//}

std::vector<float> coloradar::collapseHeatmapElevation(const std::vector<float>& image, const float& elevationMinMeters, const float& elevationMaxMeters, const std::vector<float>& elevationBins, const int& numAzimuthBins, const int& numRangeBins) {
    if (elevationMinMeters > elevationMaxMeters) {
        throw std::out_of_range("Invalid elevation range: elevationMinMeters must be less or equal to elevationMaxMeters.");
    }
    std::vector<float> collapsed_image;
    collapsed_image.reserve(numAzimuthBins * numRangeBins * 2);
    for (int a = 0; a < numAzimuthBins; ++a) {
        for (int r = 0; r < numRangeBins; ++r) {
            float maxIntensity = -std::numeric_limits<float>::infinity();
            float maxDoppler = 0.0f;
            for (int e = 0; e < elevationBins.size(); ++e) {
                float z = r * std::sin(elevationBins[e]);
                if (z >= elevationMinMeters && z <= elevationMaxMeters) {
                    int index = (((e * numAzimuthBins + a) * numRangeBins + r) * 2);
                    if (image[index] > maxIntensity) {
                        maxIntensity = image[index];
                        maxDoppler = image[index + 1];
                    }
                }
            }
            collapsed_image.push_back(maxIntensity);
            collapsed_image.push_back(maxDoppler);
        }
    }
    return collapsed_image;
}


//std::vector<float> coloradar::collapseHeatmapElevation(const std::vector<float>& image, float elevationMinDeg, float elevationMaxDeg, const std::vector<float>& elevationBins, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins) {
//    int elevationMinBin, elevationMaxBin;
//    coloradar::convertElevationRangeToBins(elevationMinDeg, elevationMaxDeg, elevationBins, elevationMinBin, elevationMaxBin);
//    return coloradar::collapseHeatmapElevation(image, elevationMinBin, elevationMaxBin, numAzimuthBins, numElevationBins, numRangeBins);
//}

std::vector<float> coloradar::removeDoppler(const std::vector<float>& image) {
    std::vector<float> intensityImage;
    intensityImage.reserve(image.size() / 2);
    for (size_t i = 0; i < image.size(); i += 2) {
        intensityImage.push_back(image[i]);
    }
    return intensityImage;
}
