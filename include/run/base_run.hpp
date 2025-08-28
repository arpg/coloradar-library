#ifndef BASE_RUN_HPP
#define BASE_RUN_HPP


namespace coloradar {

template<PoseType PoseT>
std::vector<PoseT> Run::getPoses() const {
    if (poses_.empty()) throw std::runtime_error("No poses found in run " + name_);
    std::vector<PoseT> outPoses;
    outPoses.reserve(poses_.size());
    for (const auto& pose : poses_) {
        outPoses.push_back(coloradar::internal::fromEigenPose<PoseT>(pose));
    }
    return outPoses;
}

}

#endif
