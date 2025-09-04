#include "utils/utils.h"


namespace coloradar {

const int findClosestTimestampIndex(const double targetTimestamp, const std::vector<double>& timestamps, const std::string& preference) {
    if (timestamps.empty()) {
        throw std::runtime_error("Timestamps vector is empty.");
    }

    if (preference != "none" && preference != "before" && preference != "after") {
        throw std::invalid_argument("Invalid preference: must be 'none', 'before', or 'after'.");
    }

    auto it = std::lower_bound(timestamps.begin(), timestamps.end(), targetTimestamp);
    size_t afterIdx = static_cast<size_t>(it - timestamps.begin());

    if (afterIdx < timestamps.size() && timestamps[afterIdx] == targetTimestamp) {
        return static_cast<int>(afterIdx);
    }

    bool hasAfter = afterIdx < timestamps.size();
    bool hasBefore = afterIdx > 0;
    size_t beforeIdx = hasBefore ? afterIdx - 1 : 0;

    if (preference == "none") {
        double beforeDiff = hasBefore ? std::abs(timestamps[beforeIdx] - targetTimestamp)
                                      : std::numeric_limits<double>::max();
        double afterDiff  = hasAfter  ? std::abs(timestamps[afterIdx]  - targetTimestamp)
                                      : std::numeric_limits<double>::max();

        return (beforeDiff <= afterDiff) ? static_cast<int>(beforeIdx) : static_cast<int>(afterIdx);
    }

    if (preference == "before") {
        if (hasBefore) return static_cast<int>(beforeIdx);
        if (hasAfter)  return static_cast<int>(afterIdx);
    }

    if (preference == "after") {
        if (hasAfter)  return static_cast<int>(afterIdx);
        if (hasBefore) return static_cast<int>(beforeIdx);
    }

    throw std::runtime_error("No valid timestamp found.");
}

}