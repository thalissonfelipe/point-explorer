#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>
#include <chrono>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include "../include/Computation.h"
#include "../include/GeometricFeatures.h"

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

PointAnalysis process(CloudXYZ::Ptr &cloud, pcl::PointXYZ point, std::string method, float methodValue)
{
    CloudXYZ::Ptr filteredCloud(new CloudXYZ);
    
    // Removing NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

    CloudNormal::Ptr normalCloud(new CloudNormal);
    Computation::normalComputation(filteredCloud, method, methodValue, normalCloud);

    CloudNormal::Ptr filteredNormalCloud(new CloudNormal);

    // We will not use 'indices' vector, clearing it to re-use the variable.
    indices.clear();
    pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);

    /*
    * After removing NaNs from Normal Cloud, maybe the size of the XYZ Cloud isn't the same size of Normal Cloud,
    * so we keep in the XYZ Cloud only the indices which are present in Normal Cloud.
    */
    Computation::removeNonExistingIndices(filteredCloud, indices);

    CloudPC::Ptr principalCurvaturesCloud(new CloudPC);
    Computation::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, method, methodValue, principalCurvaturesCloud);

    std::vector<int> notNaNIndicesOfShapeIndexes;
    std::vector<float> shapeIndexes;
    
    Computation::shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, notNaNIndicesOfShapeIndexes);
    if (notNaNIndicesOfShapeIndexes.size() != filteredCloud->points.size() || notNaNIndicesOfShapeIndexes.size() != principalCurvaturesCloud->points.size())
    {
        Computation::removeNonExistingIndices(filteredCloud, notNaNIndicesOfShapeIndexes);
        Computation::removeNonExistingIndices(principalCurvaturesCloud, notNaNIndicesOfShapeIndexes);
    }

    PointAnalysis pointAnalysis = Computation::getPointAnalysis(point, filteredCloud, filteredNormalCloud, principalCurvaturesCloud, shapeIndexes);

    return pointAnalysis;
}

void extractShapeIndexAndGaussianCurvature(std::string outputFilename, std::string type) {
    const int nFolders = 104;

    // std::ios_base::app (append mode)
    std::ofstream myfile;
    myfile.open(outputFilename);

    myfile << "cloud,individuo,";
    myfile << "radius_10_si,radius_10_cg,radius_11_si,radius_11_cf,radius_12_si,radius_12_cg,radius_13_si,radius_13_cg,radius_14_si,radius_14_cg,";
    myfile << "k_100_si,k_100_cg,k_150_si,k_150_cg,k_200_si,k_200_cg,k_250_si,k_250_cg,k_300_si,k_300_cg" << std::endl;

    for (int i = 0; i <= nFolders; i++)
    {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << i;
        std::string folder = "bs" + oss.str();

        std::string landmarksPath = "/home/thalisson/Documents/landmarks/landmarks/" + type + "/" + folder;
        std::string originalCloudsFolder = "/media/thalisson/Seagate Expansion Drive/BD Faces/Bosphorus_Original_PCD/" + folder + "/";

        std::cout << "Folder [" << folder << "]" << std::endl;

        for (const auto & entry : fs::directory_iterator(landmarksPath))
        {
            std::string filename = entry.path();
            std::string originalFilename = fs::path(filename).filename();
            std::string originalCloudPath = originalCloudsFolder + originalFilename;

            if (has_suffix(filename, ".pcd"))
            {
                myfile << originalFilename << "," << i;

                CloudXYZ::Ptr cloud(new CloudXYZ);
                if (pcl::io::loadPCDFile(filename, *cloud) == -1)
                {
                    myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null" << std::endl;
                    continue;
                }

                CloudXYZ::Ptr originalCloud(new CloudXYZ);

                if (pcl::io::loadPCDFile(originalCloudPath, *originalCloud) == -1)
                {
                    myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null" << std::endl;
                    continue;
                }

                // radius
                bool canContinue = true;
                for (int j = 10; j <= 14; j++) {
                    PointAnalysis pa = process(originalCloud, cloud->points[0], "radius", j);

                    if (pa.shapeIndex != -10)
                    {
                        myfile << "," << pa.shapeIndex << "," << pa.gaussianCurvature;
                    } else {
                        // point not found
                        myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null";
                        canContinue = false;
                        break;
                    }
                }

                // neighbors
                if (canContinue)
                {
                    for (int k = 100; k <= 300; k+=50) {
                        PointAnalysis pa = process(originalCloud, cloud->points[0], "k", k);
                        myfile << "," << pa.shapeIndex << "," << pa.gaussianCurvature;
                    }
                }

                myfile << std::endl;
            }
        }
    }

    myfile.close();
}

// 0 - K neighbors
// 1 - Radius
void extractGeometricFeatures(int method, std::string outputFilename, std::string type) {
    const int nFolders = 104;

    std::ofstream myfile;
    myfile.open(outputFilename);

    myfile << "cloud,";
    // K neighbors
    if (method == 0)
    {
        myfile << "gf01_k_100,gf02_k_100,gf03_k_100,gf04_k_100,gf05_k_100,gf06_k_100,gf07_k_100,gf08_k_100,gf09_k_100,";
        myfile << "gf01_k_150,gf02_k_150,gf03_k_150,gf04_k_150,gf05_k_150,gf06_k_150,gf07_k_150,gf08_k_150,gf09_k_150,";
        myfile << "gf01_k_200,gf02_k_200,gf03_k_200,gf04_k_200,gf05_k_200,gf06_k_200,gf07_k_200,gf08_k_200,gf09_k_200,";
        myfile << "gf01_k_250,gf02_k_250,gf03_k_250,gf04_k_250,gf05_k_250,gf06_k_250,gf07_k_250,gf08_k_250,gf09_k_250,";
        myfile << "gf01_k_300,gf02_k_300,gf03_k_300,gf04_k_300,gf05_k_300,gf06_k_300,gf07_k_300,gf08_k_300,gf09_k_300" << std::endl;
    }
    
    // Radius
    if (method == 1)
    {
        myfile << "gf01_r_10,gf02_r_10,gf03_r_10,gf04_r_10,gf05_r_10,gf06_r_10,gf07_r_10,gf08_r_10,gf09_r_10,";
        myfile << "gf01_r_15,gf02_r_15,gf03_r_15,gf04_r_15,gf05_r_15,gf06_r_15,gf07_r_15,gf08_r_15,gf09_r_15,";
        myfile << "gf01_r_20,gf02_r_20,gf03_r_20,gf04_r_20,gf05_r_20,gf06_r_20,gf07_r_20,gf08_r_20,gf09_r_20,";
        myfile << "gf01_r_25,gf02_r_25,gf03_r_25,gf04_r_25,gf05_r_25,gf06_r_25,gf07_r_25,gf08_r_25,gf09_r_25,";
        myfile << "gf01_r_30,gf02_r_30,gf03_r_30,gf04_r_30,gf05_r_30,gf06_r_30,gf07_r_30,gf08_r_30,gf09_r_30" << std::endl;
    }
    
    for (int i = 0; i <= nFolders; i++)
    {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << i;
        std::string folder = "bs" + oss.str();

        std::string landmarksPath = "/home/thalisson/Documents/landmarks/landmarks/" + type + "/" + folder;
        std::string originalCloudsFolder = "/media/thalisson/Seagate Expansion Drive/BD Faces/Bosphorus_Original_PCD/" + folder + "/";

        std::cout << "Folder [" << folder << "]" << std::endl;

        for (const auto & entry : fs::directory_iterator(landmarksPath))
        {
            std::string filename = entry.path();
            std::string originalFilename = fs::path(filename).filename();
            std::string originalCloudPath = originalCloudsFolder + originalFilename;

            if (has_suffix(filename, ".pcd"))
            {
                myfile << originalFilename;

                CloudXYZ::Ptr landmarkCloud(new CloudXYZ);
                if (pcl::io::loadPCDFile(filename, *landmarkCloud) == -1)
                {
                    myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null";
                    myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null" << std::endl;
                    continue;
                }

                CloudXYZ::Ptr originalCloud(new CloudXYZ);

                if (pcl::io::loadPCDFile(originalCloudPath, *originalCloud) == -1)
                {
                    myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null";
                    myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null" << std::endl;
                    continue;
                }

                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(originalCloud);

                // K Neighbors
                if (method == 0)
                {
                    for (int k = 100; k <= 300; k+=50)
                    {
                        std::vector<int> pointIdxKNNSearch(k);
                        std::vector<float> pointKNNSquaredDistance(k);

                        if (kdtree.nearestKSearch(landmarkCloud->points[0], k, pointIdxKNNSearch, pointKNNSquaredDistance) <= 0)
                        {
                            myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null";
                            myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null" << std::endl;
                            continue;
                        }

                        GeometricFeatures gf;
                        CloudXYZ::Ptr filteredCloud(new CloudXYZ);

                        pcl::copyPointCloud(*originalCloud, pointIdxKNNSearch, *filteredCloud);

                        GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf);

                        myfile << GeometricFeaturesComputation::printGeometricFeatures(&gf);
                    }

                    myfile << std::endl;

                    continue;
                }

                // Radius
                for (int r = 10; r <= 30; r+=5)
                {
                    std::vector<int> pointIdxKNNSearch;
                    std::vector<float> pointKNNSquaredDistance;

                    if (kdtree.radiusSearch(landmarkCloud->points[0], r, pointIdxKNNSearch, pointKNNSquaredDistance) <= 0)
                    {
                        myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null";
                        myfile << ",null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null" << std::endl;
                        continue;
                    }

                    GeometricFeatures gf;
                    CloudXYZ::Ptr filteredCloud(new CloudXYZ);

                    pcl::copyPointCloud(*originalCloud, pointIdxKNNSearch, *filteredCloud);

                    GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf);

                    myfile << GeometricFeaturesComputation::printGeometricFeatures(&gf);
                }

                myfile << std::endl;
            }
        }
    }

    myfile.close();
}

int main(int n, char **argv) {
  // std::cout << "Extracting Anistropy for K neighbors" << std::endl;
  // extractGeometricFeatures(0, "anisotropy_k_neighbors_re.txt");
//   std::cout << "Extracting Covariance Features - Left eye" << std::endl;
//   extractGeometricFeatures(1, "testeeeeeeeeee.txt", "left-eye");
//   std::cout << "Extracting Covariance Features - Right eye" << std::endl;
//   extractGeometricFeatures(1, "covariance_features_right_eye_v2.txt", "right-eye");
//   std::cout << "Extracting Covariance Features - Nosetip" << std::endl;
//   extractGeometricFeatures(1, "covariance_features_nose_tip_v2.txt", "nose-tip");
    std::cout << "Extracting shape index and gaussian curvature" << std::endl;
    extractShapeIndexAndGaussianCurvature("testeeeeeeeeeeeeee.txt", "nose-tip");

  // if (n != 7) {
  //   throw std::runtime_error("number of args insufficient: should be 7");
  // }

  // std::string filename = argv[1];
  // std::string method = argv[2];
  // float methodValue = std::stof(argv[3]);
  // float x = std::stof(argv[4]);
  // float y = std::stof(argv[5]);
  // float z = std::stof(argv[6]);
  // int index = -1;

  // std::cout << x << " " << y << " " << z << std::endl;

  // CloudXYZ::Ptr cloud(new CloudXYZ);

  // if (pcl::io::loadPCDFile(filename, *cloud) == -1)
  // {
  //   throw std::runtime_error("Couldn't read PCD file");
  // }

  // std::cout << cloud->points.size() << std::endl;
}
