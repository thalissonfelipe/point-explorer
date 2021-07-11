#include <string>

#include "../include/Computation.h"
#include "../include/GeometricFeatures.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

void Computation::normalComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    std::string radiusOrKSearch,
    float radiusOrK,
    pcl::PointCloud<pcl::Normal>::Ptr outputNormalCloud
)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    normalEstimation.setInputCloud(inputCloud);
    normalEstimation.setSearchMethod(tree);

    if (radiusOrKSearch.compare("radius") == 0)
    {
        normalEstimation.setRadiusSearch(radiusOrK);
    }
    else if (radiusOrKSearch.compare("k") == 0)
    {
        normalEstimation.setKSearch(radiusOrK);
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in normalComputation");
    }

    normalEstimation.compute(*outputNormalCloud);
}

void Computation::removeNonExistingIndices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    std::vector<int> indicesToKeep
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        tempCloud->points.push_back(inputCloud->points[i]);
    }

    inputCloud->points.clear();

    for (int i = 0; i < indicesToKeep.size(); i++)
    {
        inputCloud->points.push_back(tempCloud->points[indicesToKeep[i]]);
    }
}

void Computation::removeNonExistingIndices(
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &inputCloud,
    std::vector<int> indicesToKeep
)
{
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr tempCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        tempCloud->points.push_back(inputCloud->points[i]);
    }

    inputCloud->points.clear();

    for (int i = 0; i < indicesToKeep.size(); i++)
    {
        inputCloud->points.push_back(tempCloud->points[indicesToKeep[i]]);
    }
}

void Computation::principalCurvaturesComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::Normal>::Ptr normalInputCloud,
    std::string radiusOrKSearch,
    int radiusOrK,
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud
)
{
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    principalCurvaturesEstimation.setInputCloud(inputCloud);
    principalCurvaturesEstimation.setInputNormals(normalInputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    principalCurvaturesEstimation.setSearchMethod(tree);

    if (radiusOrKSearch.compare("radius") == 0)
    {
        principalCurvaturesEstimation.setRadiusSearch(radiusOrK);
    }
    else if (radiusOrKSearch.compare("k") == 0)
    {
        principalCurvaturesEstimation.setKSearch(radiusOrK);
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in principalCurvaturesComputation");
    }

    principalCurvaturesEstimation.compute(*outputPrincipalCurvaturesCloud);
}

void Computation::shapeIndexComputation(
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud,
    std::vector<float> &outputShapeIndexes,
    std::vector<int> &notNaNIndices
)
{
    float shapeIndex;
    float k1;
    float k2;
    float atg;

    for (int i = 0; i < principalCurvaturesCloud->points.size(); i++)
    {
        k1 = principalCurvaturesCloud->points[i].pc1;
        k2 = principalCurvaturesCloud->points[i].pc2;

        if (k1 >= k2)
        {
            atg = atan((k2 + k1) / (k2 - k1));
            shapeIndex = (2 / M_PI) * (atg);
        }
        else
        {
            atg = atan((k1 + k2) / (k1 - k2));
            shapeIndex = (2 / M_PI) * (atg);
        }

        if (!std::isnan(shapeIndex))
        {
            outputShapeIndexes.push_back(shapeIndex);
            notNaNIndices.push_back(i);
        }
    }
}

PointAnalysis Computation::getPointAnalysis(
    pcl::PointXYZ point,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    pcl::PointCloud<pcl::Normal>::Ptr &normalCloud,
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &principalCurvaturesCloud,
    std::vector<float> shapeIndexes
)
{
    int index = -1;

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        pcl::PointXYZ p = inputCloud->points[i];
        if (point.x == p.x && point.y == p.y && point.z == p.z)
        {
            index = i;
            break;
        }
    }

    if (index == -1)
    {
        PointAnalysis pointAnalysis;
        pointAnalysis.shapeIndex = -10;
        return pointAnalysis;
    }

    pcl::Normal normal = normalCloud->points[index];
    pcl::PrincipalCurvatures principalCurvatures = principalCurvaturesCloud->points[index];
    float shapeIndex = shapeIndexes[index];
    float gaussianCurvature = principalCurvatures.pc1 * principalCurvatures.pc2;

    PointAnalysis pa = {normal, principalCurvatures, shapeIndex, gaussianCurvature};
    return pa;
}
