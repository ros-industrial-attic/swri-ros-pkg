/*
* SupportClasses.cpp
*
*  Created on: Aug 3, 2012
*      Author: jnicho
*/
#include <vfh_recognition/SupportClasses.h>
#include <ros/ros.h>


void RosParametersList::loadParams(ros::NodeHandle &nh,bool useRelativeNamespace)
{
	using namespace RosParams;

	std::string paramScope;
	if(useRelativeNamespace)
	{
		paramScope = "";
	}
	else
	{
		paramScope = "/";
	}

	nh.param<std::string>(paramScope + Names::InputCloudTopicName,Vals.InputCloudTopicName,Defaults::InputCloudTopicName);
	nh.param<std::string>(paramScope + Names::InputDataDirectory,Vals.InputDataDirectory,Defaults::InputDataDirectory);
	nh.param<std::string>(paramScope + Names::InputDataExtension,Vals.InputDataExtension,Defaults::InputDataExtension);

	nh.param<std::string>(paramScope + Names::RecognitionServiceName,Vals.RecognitionServiceName,Defaults::RecognitionServiceName);
	nh.param<int>(paramScope + Names::RecognitionHistogramSize,Vals.RecognitionHistogramSize,Defaults::RecognitionHistogramSize);
	nh.param<int>(paramScope + Names::RecognitionNumNeighbors,Vals.RecognitionNumNeighbors,Defaults::RecognitionNumNeighbors);
	nh.param<double>(paramScope + Names::RecognitionSimilarityThreshold,Vals.RecognitionSimilarityThreshold,Defaults::RecognitionSimilarityThreshold);
	nh.param<double>(paramScope + Names::RecognitionNormalEstimationRadius,Vals.RecognitionNormalEstimationRadius,Defaults::RecognitionNormalEstimationRadius);

	nh.param<int>(paramScope + Names::SegmentationMaxIterations,Vals.SegmentationMaxIterations,Defaults::SegmentationMaxIterations);
	nh.param<double>(paramScope + Names::SegmentationDistanceThreshold,Vals.SegmentationDistanceThreshold,Defaults::SegmentationLeafSizeX);
	nh.param<double>(paramScope + Names::SegmentationLeafSizeX,Vals.SegmentationLeafSizeX,Defaults::SegmentationMaxIterations);
	nh.param<double>(paramScope + Names::SegmentationLeafSizeY,Vals.SegmentationLeafSizeY,Defaults::SegmentationLeafSizeY);
	nh.param<double>(paramScope + Names::SegmentationLeafSizeZ,Vals.SegmentationLeafSizeZ,Defaults::SegmentationLeafSizeZ);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMinX,Vals.SegmentationSpatialFilterMinX,Defaults::SegmentationSpatialFilterMinX);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMaxX,Vals.SegmentationSpatialFilterMaxX,Defaults::SegmentationSpatialFilterMaxX);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMinY,Vals.SegmentationSpatialFilterMinY,Defaults::SegmentationSpatialFilterMinY);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMaxY,Vals.SegmentationSpatialFilterMaxY,Defaults::SegmentationSpatialFilterMaxY);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMinZ,Vals.SegmentationSpatialFilterMinZ,Defaults::SegmentationSpatialFilterMinZ);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMaxZ,Vals.SegmentationSpatialFilterMinZ,Defaults::SegmentationSpatialFilterMinZ);
	nh.param<double>(paramScope + Names::SegmentationClusterConfigAcctPercnt,Vals.SegmentationClusterConfigAcctPercnt,Defaults::SegmentationClusterConfigAcctPercnt);
	nh.param<double>(paramScope + Names::SegmentationClusterConfigSpatialTolerance,Vals.SegmentationClusterConfigSpatialTolerance,
			Defaults::SegmentationClusterConfigSpatialTolerance);
	nh.param<int>(paramScope + Names::SegmentationClusterConfigMinSize,Vals.SegmentationClusterConfigMinSize,Defaults::SegmentationClusterConfigMinSize);
	nh.param<int>(paramScope + Names::SegmentationClusterConfigMaxSize,Vals.SegmentationClusterConfigMaxSize,Defaults::SegmentationClusterConfigMaxSize);
}

void RosParametersList::loadParams(bool useRelativeNamespace)
{
	using namespace RosParams;
	ros::NodeHandle nh;

	std::string paramScope;
	if(useRelativeNamespace)
	{
		paramScope = "";
	}
	else
	{
		paramScope = "/";
	}

	nh.param<std::string>(paramScope + Names::InputCloudTopicName,Vals.InputCloudTopicName,Defaults::InputCloudTopicName);
	nh.param<std::string>(paramScope + Names::InputDataDirectory,Vals.InputDataDirectory,Defaults::InputDataDirectory);
	nh.param<std::string>(paramScope + Names::InputDataExtension,Vals.InputDataExtension,Defaults::InputDataExtension);

	nh.param<std::string>(paramScope + Names::RecognitionServiceName,Vals.RecognitionServiceName,Defaults::RecognitionServiceName);
	nh.param<int>(paramScope + Names::RecognitionHistogramSize,Vals.RecognitionHistogramSize,Defaults::RecognitionHistogramSize);
	nh.param<int>(paramScope + Names::RecognitionNumNeighbors,Vals.RecognitionNumNeighbors,Defaults::RecognitionNumNeighbors);
	nh.param<double>(paramScope + Names::RecognitionSimilarityThreshold,Vals.RecognitionSimilarityThreshold,Defaults::RecognitionSimilarityThreshold);
	nh.param<double>(paramScope + Names::RecognitionNormalEstimationRadius,Vals.RecognitionNormalEstimationRadius,Defaults::RecognitionNormalEstimationRadius);

	nh.param<int>(paramScope + Names::SegmentationMaxIterations,Vals.SegmentationMaxIterations,Defaults::SegmentationMaxIterations);
	nh.param<double>(paramScope + Names::SegmentationDistanceThreshold,Vals.SegmentationDistanceThreshold,Defaults::SegmentationLeafSizeX);
	nh.param<double>(paramScope + Names::SegmentationLeafSizeX,Vals.SegmentationLeafSizeX,Defaults::SegmentationMaxIterations);
	nh.param<double>(paramScope + Names::SegmentationLeafSizeY,Vals.SegmentationLeafSizeY,Defaults::SegmentationLeafSizeY);
	nh.param<double>(paramScope + Names::SegmentationLeafSizeZ,Vals.SegmentationLeafSizeZ,Defaults::SegmentationLeafSizeZ);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMinX,Vals.SegmentationSpatialFilterMinX,Defaults::SegmentationSpatialFilterMinX);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMaxX,Vals.SegmentationSpatialFilterMaxX,Defaults::SegmentationSpatialFilterMaxX);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMinY,Vals.SegmentationSpatialFilterMinY,Defaults::SegmentationSpatialFilterMinY);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMaxY,Vals.SegmentationSpatialFilterMaxY,Defaults::SegmentationSpatialFilterMaxY);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMinZ,Vals.SegmentationSpatialFilterMinZ,Defaults::SegmentationSpatialFilterMinZ);
	nh.param<double>(paramScope + Names::SegmentationSpatialFilterMaxZ,Vals.SegmentationSpatialFilterMinZ,Defaults::SegmentationSpatialFilterMinZ);
	nh.param<double>(paramScope + Names::SegmentationClusterConfigAcctPercnt,Vals.SegmentationClusterConfigAcctPercnt,Defaults::SegmentationClusterConfigAcctPercnt);
	nh.param<double>(paramScope + Names::SegmentationClusterConfigSpatialTolerance,Vals.SegmentationClusterConfigSpatialTolerance,
			Defaults::SegmentationClusterConfigSpatialTolerance);
	nh.param<int>(paramScope + Names::SegmentationClusterConfigMinSize,Vals.SegmentationClusterConfigMinSize,Defaults::SegmentationClusterConfigMinSize);
	nh.param<int>(paramScope + Names::SegmentationClusterConfigMaxSize,Vals.SegmentationClusterConfigMaxSize,Defaults::SegmentationClusterConfigMaxSize);
}

RosParametersList::RosParametersList()
{
	loadParams();
}
