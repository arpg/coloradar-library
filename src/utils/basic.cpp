#include "utils.h"


namespace coloradar {

const int findClosestTimestampIndex(
    const double targetTimestamp,
    const std::vector<double>& timestamps,
    const bool beforeAllowed,
    const bool afterAllowed
) {
    if (timestamps.empty()) throw std::runtime_error("Timestamps vector is empty.");
    if (!beforeAllowed && !afterAllowed) throw std::invalid_argument("At least one of beforeAllowed or afterAllowed must be true.");

    auto it = std::lower_bound(timestamps.begin(), timestamps.end(), targetTimestamp);
    size_t afterIdx = static_cast<size_t>(it - timestamps.begin());
    size_t beforeIdx = (afterIdx == 0) ? 0 : afterIdx - 1;

    double beforeDiff = beforeAllowed && afterIdx > 0
        ? std::abs(timestamps[beforeIdx] - targetTimestamp)
        : std::numeric_limits<double>::max();

    double afterDiff = afterAllowed && afterIdx < timestamps.size()
        ? std::abs(timestamps[afterIdx] - targetTimestamp)
        : std::numeric_limits<double>::max();

    if (beforeDiff == std::numeric_limits<double>::max() && afterDiff == std::numeric_limits<double>::max()) {
        throw std::runtime_error("No valid timestamp found that satisfies before/after constraints.");
    }
    return (beforeDiff <= afterDiff) ? static_cast<int>(beforeIdx) : static_cast<int>(afterIdx);
}

}