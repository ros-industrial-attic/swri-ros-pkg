#ifndef SUPPORT_CLASSES_H
#define SUPPORT_CLASSES_H

#include <ros/package.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

static const std::string PACKAGE_NAME = "vfh_recognition";
static const std::string PACKAGE_PATH = ros::package::getPath(PACKAGE_NAME);
namespace RosParams
{
	namespace Names
	{
		static const std::string InputDataDirectory = "InputDataDirectory";
		static const std::string InputDataExtension = "InputDataExtension";
		static const std::string InputCloudTopicName = "InputCloudTopic";
		static const std::string RecognitionServiceName = "Recognition/ServiceName";
		static const std::string SegmentationServiceName = "Segmentation/ServiceName";
		static const std::string RecognitionHistogramSize = "Recognition/HistogramSize";
		static const std::string RecognitionSimilarityThreshold = "Recognition/SimilarityThreshold";
		static const std::string RecognitionNumNeighbors = "Recognition/NumberOfNeighbors";
		static const std::string RecognitionNormalEstimationRadius = "Recognition/NormalEstimationRadius";
		static const std::string SegmentationMaxIterations = "Segmentation/MaxIterations";
		static const std::string SegmentationDistanceThreshold = "Segmentation/DistanceThreshold";
		static const std::string SegmentationLeafSizeX = "Segmentation/LeafSize/X";
		static const std::string SegmentationLeafSizeY = "Segmentation/LeafSize/Y";
		static const std::string SegmentationLeafSizeZ = "Segmentation/LeafSize/Z";
		static const std::string SegmentationSpatialFilterMinX = "Segmentation/SpatialFilter/MinX";
		static const std::string SegmentationSpatialFilterMaxX = "Segmentation/SpatialFilter/MaxX";
		static const std::string SegmentationSpatialFilterMinY = "Segmentation/SpatialFilter/MinY";
		static const std::string SegmentationSpatialFilterMaxY = "Segmentation/SpatialFilter/MaxY";
		static const std::string SegmentationSpatialFilterMinZ = "Segmentation/SpatialFilter/MinZ";
		static const std::string SegmentationSpatialFilterMaxZ = "Segmentation/SpatialFilter/MaxZ";
		static const std::string SegmentationClusterConfigAcctPercnt = "Segmentation/ClusterConfiguration/AcceptablePercentage";
		static const std::string SegmentationClusterConfigSpatialTolerance = "Segmentation/ClusterConfiguration/Tolerance";
		static const std::string SegmentationClusterConfigMinSize = "Segmentation/ClusterConfiguration/MinSize";
		static const std::string SegmentationClusterConfigMaxSize = "Segmentation/ClusterConfiguration/MaxSize";
	};

	namespace Defaults
	{
		static const std::string InputDataDirectory = PACKAGE_PATH +  "/data";
		static const std::string InputDataExtension = ".pcd";
		static const std::string InputCloudTopicName = "/camera/depth_registered/points";
		static const std::string RecognitionServiceName = "/object_recognition";
		static const std::string SegmentationServiceName = "/tabletop_segmentation_serv";

		static const int RecognitionHistogramSize = 300;
		static const int RecognitionNumNeighbors = 1;
		static const double RecognitionSimilarityThreshold = 100.0f;
		static const double RecognitionNormalEstimationRadius = 0.03f;

		static const int SegmentationMaxIterations = 100;
		static const double SegmentationDistanceThreshold = 0.02f;
		static const double SegmentationLeafSizeX = 0.01;
		static const double SegmentationLeafSizeY = 0.01;
		static const double SegmentationLeafSizeZ = 0.01;
		static const double SegmentationSpatialFilterMinX = -0.7f;
		static const double SegmentationSpatialFilterMaxX = 0.7f;
		static const double SegmentationSpatialFilterMinY = 10.0f;
		static const double SegmentationSpatialFilterMaxY = -10;
		static const double SegmentationSpatialFilterMinZ = 0.0f;
		static const double SegmentationSpatialFilterMaxZ = 2.0f;
		static const double SegmentationClusterConfigAcctPercnt = 0.30f;
		static const double SegmentationClusterConfigSpatialTolerance = 0.02; // meters
		static const int SegmentationClusterConfigMinSize = 100;
		static const int SegmentationClusterConfigMaxSize = 25000;
	};

	struct Values
	{
		Values()
		{
				InputDataDirectory = "";
				InputDataExtension = "";
				InputCloudTopicName = "";

				RecognitionServiceName = "";
				SegmentationServiceName = "";
				RecognitionHistogramSize = 0;
				RecognitionNumNeighbors = 0;
				RecognitionSimilarityThreshold = 0;
				RecognitionNormalEstimationRadius = 0;

				SegmentationMaxIterations = 0;
				SegmentationDistanceThreshold = 0;
				SegmentationLeafSizeX = 0;
				SegmentationLeafSizeY = 0;
				SegmentationLeafSizeZ = 0;
				SegmentationSpatialFilterMinX = 0;
				SegmentationSpatialFilterMaxX = 0;
				SegmentationSpatialFilterMinY = 0;
				SegmentationSpatialFilterMaxY = 0;
				SegmentationSpatialFilterMinZ = 0;
				SegmentationSpatialFilterMaxZ = 0;
				SegmentationClusterConfigAcctPercnt = 0;
				SegmentationClusterConfigSpatialTolerance = 0; // meters
				SegmentationClusterConfigMinSize = 0;
				SegmentationClusterConfigMaxSize = 0;
		}

		std::string InputDataDirectory;
		std::string InputDataExtension;
		std::string InputCloudTopicName;

		std::string SegmentationServiceName;
		std::string RecognitionServiceName;
		int RecognitionHistogramSize;
		int RecognitionNumNeighbors;
		double RecognitionSimilarityThreshold;
		double RecognitionNormalEstimationRadius;

		int SegmentationMaxIterations;
		double SegmentationDistanceThreshold;
		double SegmentationLeafSizeX;
		double SegmentationLeafSizeY;
		double SegmentationLeafSizeZ;
		double SegmentationSpatialFilterMinX;
		double SegmentationSpatialFilterMaxX;
		double SegmentationSpatialFilterMinY;
		double SegmentationSpatialFilterMaxY;
		double SegmentationSpatialFilterMinZ;
		double SegmentationSpatialFilterMaxZ;
		double SegmentationClusterConfigAcctPercnt;
		double SegmentationClusterConfigSpatialTolerance; // meters
		int SegmentationClusterConfigMinSize;
		int SegmentationClusterConfigMaxSize;
	};

};

class RosParametersList
{
public:

	std::string NameSpace;
	RosParams::Values Vals;

public:

	RosParametersList();

	~RosParametersList(){}

	void loadParams(ros::NodeHandle &nh,bool useRelativeNamespace = true);

	void loadParams(bool useRelativeNamespace = false);

protected:

};

int alignTemplate (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string modelName,
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud, Eigen::Matrix4f &objectToView);

int SegmentCloud(sensor_msgs::PointCloud2 rawCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloudSegments);

int SegmentCloud(sensor_msgs::PointCloud2 rawCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloudSegments,RosParametersList &params);

int SegmentCloud(sensor_msgs::PointCloud2 rawCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloudSegments, pcl::PointCloud<pcl::PointXYZ>::Ptr table, RosParametersList &params);

#endif
