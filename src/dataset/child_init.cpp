#include "dataset.h"


namespace coloradar {

// PUBLIC METHODS

ColoradarDataset::ColoradarDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    postInit();
}

ColoradarDataset::ColoradarDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir) {
    init(pathToRunsDir, pathToCalibDir);
    postInit();
}


// PROTECTED METHODS

void ColoradarDataset::postInit() {
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_cascade.txt");
    singleChipTransform_ = loadTransform(transformsDirPath_ / "base_to_single_chip.txt");

    single_chip_ = std::make_unique<SingleChipDevice>();
    singleChipConfig_ = std::make_shared<coloradar::SingleChipConfig>(calibDirPath_);
}

}
