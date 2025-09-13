#include "utils/utils.h"


void coloradar::OctoPointcloud::transform(const Eigen::Affine3f& transformMatrix) {
    Eigen::Quaternionf rotation(transformMatrix.rotation());
    octomath::Pose6D transformPose(
        octomath::Vector3(transformMatrix.translation().x(), transformMatrix.translation().y(), transformMatrix.translation().z()),
        octomath::Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z())
    );
    this->transform(transformPose);
}

void coloradar::OctoPointcloud::filterFov(const float& horizontalFov, const float& verticalFov, const float& range) {
    auto self = shared_from_this();
    coloradar::internal::filterFov<octomap::point3d, coloradar::OctoPointcloud>(self, horizontalFov, verticalFov, range);
}
