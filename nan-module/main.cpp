#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>
#include <chrono>

#include <nan.h>

#include "../include/GeometricFeatures.h"
#include "../include/Computation.h"

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

v8::Local<v8::Array> parseCloudToV8Array(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int cloudSize = cloud->points.size();
    v8::Local<v8::Array> response = Nan::New<v8::Array>(cloudSize);

    for (int i = 0; i < cloudSize; i++)
    {
        pcl::PointXYZ point;
        v8::Local<v8::Object> obj = Nan::New<v8::Object>();

        point = cloud->points[i];

        obj->Set(Nan::New("x").ToLocalChecked(), Nan::New<v8::Number>(point.x));
        obj->Set(Nan::New("y").ToLocalChecked(), Nan::New<v8::Number>(point.y));
        obj->Set(Nan::New("z").ToLocalChecked(), Nan::New<v8::Number>(point.z));
        Nan::Set(response, i, obj);
    }

    return response;
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

        // Removing NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

        CloudNormal::Ptr normalCloud(new CloudNormal);
        Computation::normalComputation(filteredCloud, computationMethod, computationSize, normalCloud);

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
        Computation::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, computationMethod, computationSize, principalCurvaturesCloud);

        std::vector<int> notNaNIndicesOfShapeIndexes;
        std::vector<float> shapeIndexes;

        Computation::shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, notNaNIndicesOfShapeIndexes);

        if (notNaNIndicesOfShapeIndexes.size() != filteredCloud->points.size() || notNaNIndicesOfShapeIndexes.size() != principalCurvaturesCloud->points.size())
        {
            Computation::removeNonExistingIndices(filteredCloud, notNaNIndicesOfShapeIndexes);
            Computation::removeNonExistingIndices(principalCurvaturesCloud, notNaNIndicesOfShapeIndexes);
        }

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();

        PointAnalysis pointAnalysis = Computation::getPointAnalysis(pointToAnalyze, filteredCloud, filteredNormalCloud, principalCurvaturesCloud, shapeIndexes);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        std::vector<int> pointIdxKNNSearch;
        std::vector<float> pointKNNSquaredDistance;

        if (computationMethod.compare("radius") == 0)
        {
            kdtree.radiusSearch(pointToAnalyze, computationSize, pointIdxKNNSearch, pointKNNSquaredDistance);
        }
        else if (computationMethod.compare("k") == 0)
        {
            kdtree.nearestKSearch(pointToAnalyze, computationSize, pointIdxKNNSearch, pointKNNSquaredDistance);
        }

        CloudXYZ::Ptr gfCloud(new CloudXYZ);
        pcl::copyPointCloud(*cloud, pointIdxKNNSearch, *gfCloud);

        GeometricFeatures gf;

        GeometricFeaturesComputation::geometricFeatures(*gfCloud, &gf);

        v8::Local<v8::Object> normalV8Object = Nan::New<v8::Object>();
        normalV8Object->Set(Nan::New("x").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.normal.normal_x));
        normalV8Object->Set(Nan::New("y").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.normal.normal_y));
        normalV8Object->Set(Nan::New("z").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.normal.normal_z));

        v8::Local<v8::Object> gfV8Object = Nan::New<v8::Object>();
        gfV8Object->Set(Nan::New("sum").ToLocalChecked(), Nan::New<v8::Number>(gf.gf01));
        gfV8Object->Set(Nan::New("omnivariance").ToLocalChecked(), Nan::New<v8::Number>(gf.gf02));
        gfV8Object->Set(Nan::New("eigenentropy").ToLocalChecked(), Nan::New<v8::Number>(gf.gf03));
        gfV8Object->Set(Nan::New("anisotropy").ToLocalChecked(), Nan::New<v8::Number>(gf.gf04));
        gfV8Object->Set(Nan::New("planarity").ToLocalChecked(), Nan::New<v8::Number>(gf.gf05));
        gfV8Object->Set(Nan::New("linearity").ToLocalChecked(), Nan::New<v8::Number>(gf.gf06));
        gfV8Object->Set(Nan::New("surfaceVariation").ToLocalChecked(), Nan::New<v8::Number>(gf.gf07));
        gfV8Object->Set(Nan::New("sphericity").ToLocalChecked(), Nan::New<v8::Number>(gf.gf08));
        gfV8Object->Set(Nan::New("verticality").ToLocalChecked(), Nan::New<v8::Number>(gf.gf09));

        v8::Local<v8::Object> principalCurvaturesV8Object = Nan::New<v8::Object>();
        principalCurvaturesV8Object->Set(Nan::New("k1").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.principalCurvatures.pc1));
        principalCurvaturesV8Object->Set(Nan::New("k2").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.principalCurvatures.pc2));

        v8::Local<v8::Object> pointAnalysisV8Object = Nan::New<v8::Object>();
        pointAnalysisV8Object->Set(Nan::New("normal").ToLocalChecked(), normalV8Object);
        pointAnalysisV8Object->Set(Nan::New("principal_curvatures").ToLocalChecked(), principalCurvaturesV8Object);
        pointAnalysisV8Object->Set(Nan::New("gaussian_curvature").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.gaussianCurvature));
        pointAnalysisV8Object->Set(Nan::New("shape_index").ToLocalChecked(), Nan::New<v8::Number>(pointAnalysis.shapeIndex));
        pointAnalysisV8Object->Set(Nan::New("geometric_features").ToLocalChecked(), gfV8Object);
        pointAnalysisV8Object->Set(Nan::New("neighborhood").ToLocalChecked(), parseCloudToV8Array(gfCloud));

        moduleResponse->Set(Nan::New("point_analysis").ToLocalChecked(), pointAnalysisV8Object);

        info.GetReturnValue().Set(moduleResponse);
    }
    catch (std::exception &e)
    {
        v8::Isolate *isolate = v8::Isolate::GetCurrent();
        isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, e.what())));
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
