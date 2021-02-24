#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>
#include <chrono>

#include <nan.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

struct PointAnalysis
{
  pcl::Normal normal;
  pcl::PrincipalCurvatures principalCurvatures;
  float shapeIndex;
  float gaussianCurvature;
};

void normalComputation(
    CloudXYZ::Ptr inputCloud,
    std::string radiusOrKSearch,
    float radiusOrK,
    CloudNormal::Ptr outputNormalCloud)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  normalEstimation.setInputCloud(inputCloud);
  normalEstimation.setSearchMethod(tree);

  if (radiusOrKSearch == "radius")
  {
    normalEstimation.setRadiusSearch(radiusOrK);
  }
  else
  {
    if (radiusOrKSearch == "k")
    {
      normalEstimation.setKSearch(radiusOrK);
    }
    else
    {
      throw std::runtime_error("Use 'radius' or 'k' in normalComputation");
    }
  }

  normalEstimation.compute(*outputNormalCloud);
}

void removeNonExistingIndices(CloudXYZ::Ptr &inputCloud, std::vector<int> indicesToKeep)
{
  CloudXYZ::Ptr tempCloud(new CloudXYZ);

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

void removeNonExistingIndices(CloudPC::Ptr &inputCloud, std::vector<int> indicesToKeep)
{
  CloudPC::Ptr tempCloud(new CloudPC);

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

void principalCurvaturesComputation(
    CloudXYZ::Ptr inputCloud,
    CloudNormal::Ptr normalInputCloud,
    std::string radiusOrKSearch,
    int radiusOrK,
    CloudPC::Ptr outputPrincipalCurvaturesCloud)
{
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

  principalCurvaturesEstimation.setInputCloud(inputCloud);
  principalCurvaturesEstimation.setInputNormals(normalInputCloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  principalCurvaturesEstimation.setSearchMethod(tree);

  if (radiusOrKSearch == "radius")
  {
    principalCurvaturesEstimation.setRadiusSearch(radiusOrK);
  }
  else
  {
    if (radiusOrKSearch == "k")
    {
      principalCurvaturesEstimation.setKSearch(radiusOrK);
    }
    else
    {
      throw std::runtime_error("Use 'radius' or 'k' in principalCurvaturesComputation");
    }
  }

  principalCurvaturesEstimation.compute(*outputPrincipalCurvaturesCloud);
}

void shapeIndexComputation(
    CloudPC::Ptr principalCurvaturesCloud,
    std::vector<float> &outputShapeIndexes,
    std::vector<int> &notNaNIndices)
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

PointAnalysis getPointAnalysis(pcl::PointXYZ point, CloudXYZ::Ptr &inputCloud, CloudNormal::Ptr &normalCloud, CloudPC::Ptr &principalCurvaturesCloud, std::vector<float> shapeIndexes)
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
    throw std::runtime_error("Could not find provided point for analysis after NaN filters. Maybe you selected a NaN point or a nonexistent index.");
  }

  pcl::Normal normal = normalCloud->points[index];
  pcl::PrincipalCurvatures principalCurvatures = principalCurvaturesCloud->points[index];
  float shapeIndex = shapeIndexes[index];
  float gaussianCurvature = principalCurvatures.pc1 * principalCurvatures.pc2;

  struct PointAnalysis pa = {normal, principalCurvatures, shapeIndex, gaussianCurvature};
  return pa;
}

NAN_METHOD(GetPointAnalysis)
{
  try
  {
    std::string filename(*Nan::Utf8String(info[0]));
    std::string computationMethod(*Nan::Utf8String(info[1]));
    int computationSize = info[2]->NumberValue();
    int pointIndexToAnalyze = info[3]->NumberValue();

    CloudXYZ::Ptr cloud(new CloudXYZ);

    if (pcl::io::loadPCDFile(filename, *cloud) == -1)
    {
      throw std::runtime_error("Couldn't read PCD file");
    }

    pcl::PointXYZ pointToAnalyze = cloud->points[pointIndexToAnalyze];

    CloudXYZ::Ptr filteredCloud(new CloudXYZ);

    //Removing NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

    CloudNormal::Ptr normalCloud(new CloudNormal);
    normalComputation(filteredCloud, computationMethod, computationSize, normalCloud);

    CloudNormal::Ptr filteredNormalCloud(new CloudNormal);

    //We will not use 'indices' vector, clearing it to re-use the variable.
    indices.clear();
    pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);

    /*
    * After removing NaNs from Normal Cloud, maybe the size of the XYZ Cloud isn't the same size of Normal Cloud,
    * so we keep in the XYZ Cloud only the indices which are present in Normal Cloud.
    */
    removeNonExistingIndices(filteredCloud, indices);

    CloudPC::Ptr principalCurvaturesCloud(new CloudPC);
    principalCurvaturesComputation(filteredCloud, filteredNormalCloud, computationMethod, computationSize, principalCurvaturesCloud);

    std::vector<int> notNaNIndicesOfShapeIndexes;
    std::vector<float> shapeIndexes;
    shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, notNaNIndicesOfShapeIndexes);
    if (notNaNIndicesOfShapeIndexes.size() != filteredCloud->points.size() || notNaNIndicesOfShapeIndexes.size() != principalCurvaturesCloud->points.size())
    {
      removeNonExistingIndices(filteredCloud, notNaNIndicesOfShapeIndexes);
      removeNonExistingIndices(principalCurvaturesCloud, notNaNIndicesOfShapeIndexes);
    }

    v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();

    struct PointAnalysis pointAnalysis = getPointAnalysis(pointToAnalyze, filteredCloud, filteredNormalCloud, principalCurvaturesCloud, shapeIndexes);
    v8::Local<v8::Object> normalV8Object = Nan::New<v8::Object>();
    normalV8Object->Set(Nan::New("x").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.normal.normal_x));
    normalV8Object->Set(Nan::New("y").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.normal.normal_y));
    normalV8Object->Set(Nan::New("z").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.normal.normal_z));

    v8::Local<v8::Object> principalCurvaturesV8Object = Nan::New<v8::Object>();
    principalCurvaturesV8Object->Set(Nan::New("k1").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.principalCurvatures.pc1));
    principalCurvaturesV8Object->Set(Nan::New("k2").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.principalCurvatures.pc2));

    v8::Local<v8::Object> pointAnalysisV8Object = Nan::New<v8::Object>();
    pointAnalysisV8Object->Set(Nan::New("normal").ToLocalChecked(), normalV8Object);
    pointAnalysisV8Object->Set(Nan::New("principal_curvatures").ToLocalChecked(), principalCurvaturesV8Object);
    pointAnalysisV8Object->Set(Nan::New("gaussian_curvature").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.gaussianCurvature));
    pointAnalysisV8Object->Set(Nan::New("shape_index").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.shapeIndex));

    moduleResponse->Set(Nan::New("point_analysis").ToLocalChecked(), pointAnalysisV8Object);

    info.GetReturnValue().Set(moduleResponse);
  }
  catch (std::exception &e)
  {
    v8::Isolate *isolate = v8::Isolate::GetCurrent();
    isolate->ThrowException(v8::String::NewFromUtf8(isolate, e.what()));
  }
}

using Nan::GetFunction;
using Nan::New;
using Nan::Set;
using v8::Boolean;
using v8::FunctionTemplate;
using v8::Handle;
using v8::Number;
using v8::Object;
using v8::String;

NAN_MODULE_INIT(init)
{
  Set(target, New<String>("getPointAnalysis").ToLocalChecked(),
      GetFunction(New<FunctionTemplate>(GetPointAnalysis)).ToLocalChecked());
}

NODE_MODULE(point_analysis, init)