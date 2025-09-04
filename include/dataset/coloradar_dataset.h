#ifndef COLORADAR_DATASET_H
#define COLORADAR_DATASET_H

#include "dataset/coloradar_plus_dataset.h"
#include "run/coloradar_run.h"


namespace coloradar {

class ColoradarDataset : public ColoradarPlusDataset {
protected:
    // ATTRIBUTES
    Eigen::Affine3f singleChipTransform_;
    std::shared_ptr<RadarConfig> singleChipConfig_;
    std::unique_ptr<SingleChipDevice> single_chip_;

    // src/dataset/coloradar_dataset.cpp
    void postInit();

public:
    const Eigen::Affine3f& singleChipTransform() const { return singleChipTransform_; }
    const std::shared_ptr<RadarConfig> singleChipConfig() const { return singleChipConfig_; }

    // src/dataset/coloradar_dataset.cpp
    ColoradarDataset(const std::filesystem::path& pathToDataset);
    ColoradarDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir);
};


}

#endif
