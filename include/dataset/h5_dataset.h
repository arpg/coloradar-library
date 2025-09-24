#ifndef H5_DATASET_H
#define H5_DATASET_H

#include "dataset/base_dataset.h"
#include "run/h5_run.h"


namespace coloradar {


class H5Dataset : public Dataset {
protected:
    // ATTRIBUTES
    std::filesystem::path h5SourceFilePath_;
    std::shared_ptr<RadarConfig> heatmapConfig_;

    // METHODS
    H5Dataset() = default;

public:
    // src/dataset/h5_dataset.cpp
    explicit H5Dataset(const std::filesystem::path& pathToH5File);
    H5Dataset(const H5Dataset&) = delete;
    H5Dataset& operator=(const H5Dataset&) = delete;
    H5Dataset(H5Dataset&&) noexcept = default;
    H5Dataset& operator=(H5Dataset&&) noexcept = default;
    virtual ~H5Dataset() = default;
    void summary() const;

    const std::shared_ptr<RadarConfig> heatmapConfig() const { return heatmapConfig_; }
};

}

#endif
