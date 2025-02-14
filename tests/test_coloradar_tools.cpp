#include "coloradar_tools.h"

#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>


bool generateRandomEmptySpace(float probability) {
    static std::mt19937 gen(42);  // seed
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return dis(gen) < probability;
}

octomap::Pointcloud generateSpherePointCloud(float radius, float step, float emptySpaceProbability) {
    octomap::Pointcloud cloud;

    for (float x = -radius; x <= radius; x += step) {
        for (float y = -radius; y <= radius; y += step) {
            for (float z = -radius; z <= radius; z += step) {
                if (x*x + y*y + z*z <= radius * radius) {
                    if (!generateRandomEmptySpace(emptySpaceProbability)) {
                        cloud.push_back(octomap::point3d(x, y, z));
                    }
                }
            }
        }
    }
    return cloud;
}


TEST(ColoradarConcepts, PointTypeTests) {
    static_assert(coloradar::Pcl3dPointType<pcl::PointXYZ>, "pcl::PointXYZ should be a Pcl3dPointType");
    ASSERT_TRUE(coloradar::Pcl3dPointType<pcl::PointXYZ>);

    static_assert(coloradar::Pcl4dPointType<pcl::PointXYZI>, "pcl::PointXYZI should be a Pcl4dPointType");
    ASSERT_TRUE(coloradar::Pcl4dPointType<pcl::PointXYZI>);

    static_assert(!coloradar::Pcl4dPointType<pcl::PointXYZ>, "pcl::PointXYZ should not be a Pcl4dPointType");
    ASSERT_FALSE(coloradar::Pcl4dPointType<pcl::PointXYZ>);

    static_assert(coloradar::PclPointType<pcl::PointXYZ>, "pcl::PointXYZ should be a PclPointType");
    ASSERT_TRUE(coloradar::PclPointType<pcl::PointXYZ>);

    static_assert(coloradar::PclPointType<pcl::PointXYZI>, "pcl::PointXYZI should be a PclPointType");
    ASSERT_TRUE(coloradar::PclPointType<pcl::PointXYZI>);

    static_assert(coloradar::OctomapPointType<octomap::point3d>, "octomap::point3d should be an OctomapPointType");
    ASSERT_TRUE(coloradar::OctomapPointType<octomap::point3d>);

    static_assert(!coloradar::OctomapPointType<pcl::PointXYZI>, "pcl::PointXYZI should not be an OctomapPointType");
    ASSERT_FALSE(coloradar::OctomapPointType<pcl::PointXYZI>);
}

TEST(ColoradarConcepts, CloudTypeTests) {
    // Test if pcl::PointCloud<pcl::PointXYZ> matches PclCloudType
    static_assert(coloradar::PclCloudType<pcl::PointCloud<pcl::PointXYZ>>, "pcl::PointCloud<pcl::PointXYZ> should be a PclCloudType");
    ASSERT_TRUE(coloradar::PclCloudType<pcl::PointCloud<pcl::PointXYZ>>);

    // Test if pcl::PointCloud<pcl::PointXYZI> matches PclCloudType
    static_assert(coloradar::PclCloudType<pcl::PointCloud<pcl::PointXYZI>>, "pcl::PointCloud<pcl::PointXYZI> should be a PclCloudType");
    ASSERT_TRUE(coloradar::PclCloudType<pcl::PointCloud<pcl::PointXYZI>>);

    // Test if octomap::Pointcloud matches OctomapCloudType
    static_assert(coloradar::OctomapCloudType<octomap::Pointcloud>, "octomap::Pointcloud should be an OctomapCloudType");
    ASSERT_TRUE(coloradar::OctomapCloudType<octomap::Pointcloud>);

    // Test if pcl::PointCloud<pcl::PointXYZI> does not match OctomapCloudType
    static_assert(!coloradar::OctomapCloudType<pcl::PointCloud<pcl::PointXYZI>>, "pcl::PointCloud<pcl::PointXYZI> should not be an OctomapCloudType");
    ASSERT_FALSE(coloradar::OctomapCloudType<pcl::PointCloud<pcl::PointXYZI>>);

    // Test if pcl::PointCloud<pcl::PointXYZ> matches CloudType
    static_assert(coloradar::CloudType<pcl::PointCloud<pcl::PointXYZ>>, "pcl::PointCloud<pcl::PointXYZ> should be a CloudType");
    ASSERT_TRUE(coloradar::CloudType<pcl::PointCloud<pcl::PointXYZ>>);

    // Test if octomap::Pointcloud matches CloudType
    static_assert(coloradar::CloudType<octomap::Pointcloud>, "octomap::Pointcloud should be a CloudType");
    ASSERT_TRUE(coloradar::CloudType<octomap::Pointcloud>);
}

// Test suite for general PointType and CloudType
TEST(ColoradarConcepts, GeneralConcepts) {
    // Test if pcl::PointXYZ matches PointType
    static_assert(coloradar::PointType<pcl::PointXYZ>, "pcl::PointXYZ should be a PointType");
    ASSERT_TRUE(coloradar::PointType<pcl::PointXYZ>);

    // Test if pcl::PointXYZI matches PointType
    static_assert(coloradar::PointType<pcl::PointXYZI>, "pcl::PointXYZI should be a PointType");
    ASSERT_TRUE(coloradar::PointType<pcl::PointXYZI>);

    // Test if octomap::point3d matches PointType
    static_assert(coloradar::PointType<octomap::point3d>, "octomap::point3d should be a PointType");
    ASSERT_TRUE(coloradar::PointType<octomap::point3d>);
}


class CompileTest : public ::testing::Test {
public:
    CompileTest() : gen(randomSeed) {}

    pcl::PointCloud<pcl::PointXYZ> create3dPcl() {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.points.push_back(pcl::PointXYZ(0.f, 0.f, 0.f));
        cloud.points.push_back(pcl::PointXYZ(1.f, 1.f, 1.f));
        cloud.points.push_back(pcl::PointXYZ(2.f, 1.f, 0.f));
        cloud.points.push_back(pcl::PointXYZ(1.f, 5.f, 0.f));
        cloud.points.push_back(pcl::PointXYZ(0.f, 1.f, 2.f));
        cloud.points.push_back(pcl::PointXYZ(-2.f, 1.f, 3.f));
        cloud.points.push_back(pcl::PointXYZ(10.f, 10.f, 10.f));
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZI> create4dPcl() {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.points.push_back(pcl::PointXYZI(0.f, 0.f, 0.f, generateIntensity()));
        cloud.points.push_back(pcl::PointXYZI(1.f, 1.f, 1.f, generateIntensity()));
        cloud.points.push_back(pcl::PointXYZI(2.f, 1.f, 0.f, generateIntensity()));
        cloud.points.push_back(pcl::PointXYZI(1.f, 5.f, 0.f, generateIntensity()));
        cloud.points.push_back(pcl::PointXYZI(0.f, 1.f, 2.f, generateIntensity()));
        cloud.points.push_back(pcl::PointXYZI(10.f, 10.f, 10.f, generateIntensity()));
        cloud.points.push_back(pcl::PointXYZI(-2.f, 1.f, 3.f, generateIntensity()));
        return cloud;
    }

protected:
    const float treeResolution = 0.25;
    const int randomSeed = 42;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis{-1.0, 1.0};
    float generateIntensity() { return dis(gen) * 10; }
};


TEST_F(CompileTest, BasicFunctions) {
    octomap::OcTree tree(treeResolution);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    coloradar::octreeToPcl(tree, cloud);
    coloradar::filterFov(cloud, 360, 180, 10);

//    Compile Error
//    pcl::PointCloud<pcl::PointXYZ> cloud3d;
//    coloradar::octreeToPcl(tree, cloud3d);
//    coloradar::filterFov(cloud3d, 360, 180, 10);
}

TEST_F(CompileTest, OctoPointcloud) {
    auto cloud3d = create3dPcl();
    coloradar::OctoPointcloud octoCloud(cloud3d);
    octoCloud.filterFov(360, 180, 10);
    auto otherCloud = octoCloud.toPcl<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud4d = create4dPcl();
    octoCloud = coloradar::OctoPointcloud(cloud4d);
}

//class PclFilterTest : public ::testing::Test {
//protected:
//    double nearEps = 0.01;
//};
//
//TEST_F(PclFilterTest, RangeTest) {
//    pcl::PointCloud<pcl::PointXYZ> cloud = createPcl();
//    std::cout << "Cloud before: " << cloud.size() << " points,";
//    for (size_t i = 0; i < cloud.size(); ++i) {
//        std::cout << " " << cloud.points[i];
//    }
//    std::cout << std::endl;
//
//    coloradar::filterFov(cloud, 360, 360, 10);
//    std::cout << "Cloud after range: " << cloud.size() << " points,";
//    for (size_t i = 0; i < cloud.size(); ++i) {
//        std::cout << " " << cloud.points[i];
//    }
//    std::cout << std::endl;
//
//    coloradar::filterFov(cloud, 90, 360, 0);
//    std::cout << "Cloud after azimuth:";
//    for (size_t i = 0; i < cloud.size(); ++i) {
//        std::cout << " " << cloud.points[i];
//    }
//    std::cout << std::endl;
//
//    coloradar::filterFov(cloud, 360, 90, 0);
//    std::cout << "Cloud after elevation:";
//    for (size_t i = 0; i < cloud.size(); ++i) {
//        std::cout << " " << cloud.points[i];
//    }
//    std::cout << std::endl;
//
//    cloud = createPcl();
//    coloradar::filterFov(cloud, 90, 90, 10);
//    std::cout << "Cloud filtered:";
//    for (size_t i = 0; i < cloud.size(); ++i) {
//        std::cout << " " << cloud.points[i];
//    }
//    std::cout << std::endl;
//}


//void savePointCloudToPCD(const std::string& filename, const octomap::Pointcloud& cloud) {
//    pcl::PointCloud<pcl::PointXYZ> pclCloud;
//    for (size_t i = 0; i < cloud.size(); ++i) {
//        const octomap::point3d& point = cloud.getPoint(i);
//        pclCloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
//    }
//    std::string homeDir = getenv("HOME");
//    pcl::io::savePCDFile(homeDir + "/" + filename + ".pcd", pclCloud);
//}
//
//TEST(FilterFovTest, FilterPointCloud) {
//    float horizontalFov = 90;
//    float verticalFov = 30;
//    float maxRange = 5;
//
//    float pclRadius = 10;
//    float pclStep = 0.5;
//    float emptySpaceProbability = 0.5;
//    OctoPointcloud cloud = generateSpherePointCloud(pclRadius, pclStep, emptySpaceProbability);
//    savePointCloudToPCD("original_cloud", cloud);
//
//    float horizontalFovTan = processFov(horizontalFov);
//    float verticalFovTan = processFov(verticalFov);
//    float range = processRange(maxRange);
//    filterFov(horizontalFovTan, verticalFovTan, range);
//    savePointCloudToPCD("filtered_cloud", filteredCloud);
//}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
