/*
* SupportClasses.cpp
*
*  Created on: Aug 3, 2012
*      Author: jnicho
*/
#include <vfh_recognition/SupportClasses.h>
#include <ros/ros.h>

RosParametersList::RosParametersList()
{
	//initialize();
	Vals = RosParams::Values();
	//loadParams();
}

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
		paramScope = ros::this_node::getName() + "/";
	}

	ROS_INFO("Loading Parameters from ros parameter server");

	nh.param<std::string>(paramScope + Names::InputCloudTopicName,Vals.InputCloudTopicName,Defaults::InputCloudTopicName);
	nh.param<std::string>(paramScope + Names::InputDataDirectory,Vals.InputDataDirectory,Defaults::InputDataDirectory);
	nh.param<std::string>(paramScope + Names::InputDataExtension,Vals.InputDataExtension,Defaults::InputDataExtension);

	nh.param<std::string>(paramScope + Names::RecognitionServiceName,Vals.RecognitionServiceName,Defaults::RecognitionServiceName);
	nh.param<std::string>(paramScope + Names::SegmentationServiceName,Vals.SegmentationServiceName,Defaults::SegmentationServiceName);
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

	ROS_INFO("Loaded Parameters");
}

void RosParametersList::loadParams(bool useRelativeNamespace)
{
	using namespace RosParams;

	std::string paramScope;
	if(useRelativeNamespace)
	{
		paramScope = "";
	}
	else
	{
		paramScope = ros::this_node::getName() + "/";
	}

	ROS_INFO("Loading Default Parameters");

	ros::param::param<std::string>(paramScope + Names::InputCloudTopicName,Vals.InputCloudTopicName,Defaults::InputCloudTopicName);
	ros::param::param<std::string>(paramScope + Names::InputDataDirectory,Vals.InputDataDirectory,Defaults::InputDataDirectory);
	ros::param::param<std::string>(paramScope + Names::InputDataExtension,Vals.InputDataExtension,Defaults::InputDataExtension);

	ros::param::param<std::string>(paramScope + Names::RecognitionServiceName,Vals.RecognitionServiceName,Defaults::RecognitionServiceName);
	ros::param::param<std::string>(paramScope + Names::SegmentationServiceName,Vals.SegmentationServiceName,Defaults::SegmentationServiceName);
	ros::param::param<int>(paramScope + Names::RecognitionHistogramSize,Vals.RecognitionHistogramSize,Defaults::RecognitionHistogramSize);
	ros::param::param<int>(paramScope + Names::RecognitionNumNeighbors,Vals.RecognitionNumNeighbors,Defaults::RecognitionNumNeighbors);
	ros::param::param<double>(paramScope + Names::RecognitionSimilarityThreshold,Vals.RecognitionSimilarityThreshold,Defaults::RecognitionSimilarityThreshold);
	ros::param::param<double>(paramScope + Names::RecognitionNormalEstimationRadius,Vals.RecognitionNormalEstimationRadius,Defaults::RecognitionNormalEstimationRadius);

	ros::param::param<int>(paramScope + Names::SegmentationMaxIterations,Vals.SegmentationMaxIterations,Defaults::SegmentationMaxIterations);
	ros::param::param<double>(paramScope + Names::SegmentationDistanceThreshold,Vals.SegmentationDistanceThreshold,Defaults::SegmentationLeafSizeX);
	ros::param::param<double>(paramScope + Names::SegmentationLeafSizeX,Vals.SegmentationLeafSizeX,Defaults::SegmentationMaxIterations);
	ros::param::param<double>(paramScope + Names::SegmentationLeafSizeY,Vals.SegmentationLeafSizeY,Defaults::SegmentationLeafSizeY);
	ros::param::param<double>(paramScope + Names::SegmentationLeafSizeZ,Vals.SegmentationLeafSizeZ,Defaults::SegmentationLeafSizeZ);
	ros::param::param<double>(paramScope + Names::SegmentationSpatialFilterMinX,Vals.SegmentationSpatialFilterMinX,Defaults::SegmentationSpatialFilterMinX);
	ros::param::param<double>(paramScope + Names::SegmentationSpatialFilterMaxX,Vals.SegmentationSpatialFilterMaxX,Defaults::SegmentationSpatialFilterMaxX);
	ros::param::param<double>(paramScope + Names::SegmentationSpatialFilterMinY,Vals.SegmentationSpatialFilterMinY,Defaults::SegmentationSpatialFilterMinY);
	ros::param::param<double>(paramScope + Names::SegmentationSpatialFilterMaxY,Vals.SegmentationSpatialFilterMaxY,Defaults::SegmentationSpatialFilterMaxY);
	ros::param::param<double>(paramScope + Names::SegmentationSpatialFilterMinZ,Vals.SegmentationSpatialFilterMinZ,Defaults::SegmentationSpatialFilterMinZ);
	ros::param::param<double>(paramScope + Names::SegmentationSpatialFilterMaxZ,Vals.SegmentationSpatialFilterMinZ,Defaults::SegmentationSpatialFilterMinZ);
	ros::param::param<double>(paramScope + Names::SegmentationClusterConfigAcctPercnt,Vals.SegmentationClusterConfigAcctPercnt,Defaults::SegmentationClusterConfigAcctPercnt);
	ros::param::param<double>(paramScope + Names::SegmentationClusterConfigSpatialTolerance,Vals.SegmentationClusterConfigSpatialTolerance,
			Defaults::SegmentationClusterConfigSpatialTolerance);
	ros::param::param<int>(paramScope + Names::SegmentationClusterConfigMinSize,Vals.SegmentationClusterConfigMinSize,Defaults::SegmentationClusterConfigMinSize);
	ros::param::param<int>(paramScope + Names::SegmentationClusterConfigMaxSize,Vals.SegmentationClusterConfigMaxSize,Defaults::SegmentationClusterConfigMaxSize);

	ROS_INFO("Loaded Default Parameters");
}
