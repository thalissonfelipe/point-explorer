#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#ifndef GEOMETRIC_FEATURES_H_
#define GEOMETRIC_FEATURES_H_

/**
 * Geometric Features:
 * gf01: Sum
 * gf02: Omnivariance
 * gf03: Eigenentropy
 * gf04: Anisotropy
 * gf05: Planarity
 * gf06: Linearity
 * gf07: Surface Variation
 * gf08: Sphericity
 * gf09: Verticality
 */
struct GeometricFeatures {
    double gf01 = 0;
    double gf02 = 0;
    double gf03 = 0;
    double gf04 = 0;
    double gf05 = 0;
    double gf06 = 0;
    double gf07 = 0;
    double gf08 = 0;
    double gf09 = 0;
};

class GeometricFeaturesComputation {
public:
    void static geometricFeatures(pcl::PointCloud<pcl::PointXYZ> cloud_in, GeometricFeatures* gf);
    std::string static printGeometricFeatures(GeometricFeatures* gf);
};

#endif /* GEOMETRIC_FEATURES_H_ */
