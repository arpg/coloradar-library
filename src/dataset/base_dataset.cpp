#include "dataset/base_dataset.h"


namespace coloradar {

std::vector<std::string> Dataset::listRuns() const {
    std::vector<std::string> keys;
    keys.reserve(runs_.size());
    for (const auto& kv : runs_) {
        keys.push_back(kv.first);
    }
    return keys;
}

std::vector<std::shared_ptr<Run>> Dataset::getRuns() const {
    std::vector<std::shared_ptr<Run>> values;
    values.reserve(runs_.size());
    for (const auto& kv : runs_) {
        values.push_back(kv.second);
    }
    return values;
}

std::shared_ptr<Run> Dataset::getRun(const std::string& runName) const {
    auto it = runs_.find(runName);
    if (it != runs_.end()) {
        return it->second;
    }
    throw std::invalid_argument("Run '" + runName + "' not found in dataset.");
}
    

}