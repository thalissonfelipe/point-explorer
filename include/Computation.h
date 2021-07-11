#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

#ifndef COMPUTATION_H_
#define COMPUTATION_H_

struct PointAnalysis
{
    pcl::Normal normal;
    pcl::PrincipalCurvatures principalCurvatures;
    float shapeIndex;
    float gaussianCurvature;
};

class Computation {
public:
    void static normalComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string radiusOrKSearch,
        float radiusOrK,
        pcl::PointCloud<pcl::Normal>::Ptr outputNormalCloud);

    void static removeNonExistingIndices(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
        std::vector<int> indicesToKeep);

    void static removeNonExistingIndices(
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &inputCloud,
        std::vector<int> indicesToKeep);

    void static principalCurvaturesComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        pcl::PointCloud<pcl::Normal>::Ptr normalInputCloud,
        std::string radiusOrKSearch,
        int radiusOrK,
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud);
    
    void static shapeIndexComputation(
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud,
        std::vector<float> &outputShapeIndexes,
        std::vector<int> &notNaNIndices);

    PointAnalysis static getPointAnalysis(
        pcl::PointXYZ point,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
        pcl::PointCloud<pcl::Normal>::Ptr &normalCloud,
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &principalCurvaturesCloud,
        std::vector<float> shapeIndexes);
};

#endif /* COMPUTATION_H_ */
