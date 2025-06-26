#include "dataset.h"


namespace coloradar {

// PUBLIC METHODS

const Eigen::Affine3f& ColoradarDataset::singleChipTransform() const { return singleChipTransform_; }
const RadarConfig* ColoradarDataset::singleChipConfig() const { return singleChipConfig_; }

ColoradarPlusRun* ColoradarDataset::getRun(const std::string& runName) {
    return new ColoradarRun(runsDirPath_ / runName, cascadeConfig_, singleChipConfig_);
}

}
