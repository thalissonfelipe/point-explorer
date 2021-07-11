#include <string>

#include <pcl/point_cloud.h>

#include "../include/GeometricFeatures.h"

void GeometricFeaturesComputation::geometricFeatures(pcl::PointCloud<pcl::PointXYZ> cloud_in, GeometricFeatures* gf)
{
	pcl::PCA<pcl::PointXYZ> pca;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_in, *cloud);
	pca.setInputCloud(cloud);
	Eigen::Vector3f evCloud = pca.getEigenValues();
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
	Eigen::Vector3f v3(0,0,1);

	// Sum
	gf->gf01 = evCloud.sum();

	// Omnivariance : Dá sempre 1
	gf->gf02 = pow(evCloud.prod(),1/3);

	// Eigenentropy
	gf->gf03 = -evCloud[0]*log(evCloud[0]) -evCloud[1]*log(evCloud[1]) -evCloud[2]*log(evCloud[2]);

	// Anisotropy
	gf->gf04 = (evCloud[0]-evCloud[2])/evCloud[0];

	// Planarity
	gf->gf05 = (evCloud[1]-evCloud[2])/evCloud[0];

	// Linearity
	gf->gf06 = (evCloud[0]-evCloud[1])/evCloud[0];

	// Surface Variation
	gf->gf07 = evCloud[2]/(evCloud.sum());

	// Sphericity
	gf->gf08 = evCloud[2]/evCloud[0];

	// Verticality
	gf->gf09 = 1 -abs(v3.dot(eigen_vectors.col(2)));
}

std::string GeometricFeaturesComputation::printGeometricFeatures(GeometricFeatures* gf)
{
	char aux[3000];
	sprintf(aux, ",%f,%f,%f,%f,%f,%f,%f,%f,%f", gf->gf01, gf->gf02, gf->gf03,
			gf->gf04, gf->gf05, gf->gf06, gf->gf07, gf->gf08, gf->gf09);
	return aux;
}
