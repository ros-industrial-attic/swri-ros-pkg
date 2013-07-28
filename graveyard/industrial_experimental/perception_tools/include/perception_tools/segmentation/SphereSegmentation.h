/*
 * SphereSegmentation.h
 *
 *  Created on: Sep 19, 2012
 *      Author: jnicho
 */

#ifndef SPHERESEGMENTATION_H_
#define SPHERESEGMENTATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <tabletop_object_detector/TabletopSegmentation.h>

using namespace tabletop_object_detector;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud3D;

class SphereSegmentation
{
public:
	struct Parameters
	{
	public:
		Parameters()
		:WorldFrameId("base_link"),
		 KNearestNeighbors(50),
		 MaxIterations(100),
		 SacMethod(pcl::SAC_RANSAC), // pcl::SAC_RANSAC
		 NormalDistanceWeight(0.1f),
		 DistanceThreshold(0.05f),
		 MaxRadius(0.02),// 2cm
		 MinFitnessScore(0.0f),
		 AlignToTopCentroid(true),
		 Xmax(1.5),
		 Xmin(0.5),
		 Ymin(-1.2),
		 Ymax(1.2),
		 Zmin(-0.1),
		 Zmax(0.5)
		{

		}

		void fetchParameters(std::string nameSpace = "")
		{
			ros::param::get(nameSpace + "/world_frame_id",WorldFrameId);
			ros::param::get(nameSpace + "/k_nearest",KNearestNeighbors);
			ros::param::get(nameSpace + "/max_iterations",MaxIterations);
			ros::param::get(nameSpace + "/sac_method",SacMethod);
			ros::param::get(nameSpace + "/normal_distance_weight",NormalDistanceWeight);
			ros::param::get(nameSpace + "/distance_threshold",DistanceThreshold);
			ros::param::get(nameSpace + "/max_radius",MaxRadius);
			ros::param::get(nameSpace + "/min_fitness_score",MinFitnessScore);
			ros::param::get(nameSpace + "/x_max",Xmax);
			ros::param::get(nameSpace + "/x_min",Xmin);
			ros::param::get(nameSpace + "/y_max",Ymax);
			ros::param::get(nameSpace + "/y_min",Ymin);
			ros::param::get(nameSpace + "/z_max",Zmax);
			ros::param::get(nameSpace + "/z_min",Zmin);
			ros::param::get(nameSpace + "/align_to_top_centroid",AlignToTopCentroid);
		}

		// ros parameters
		std::string WorldFrameId;
		int KNearestNeighbors;
		int MaxIterations;
		int SacMethod; // pcl::SAC_RANSAC
		double NormalDistanceWeight;
		double DistanceThreshold;
		double MaxRadius;
		double MinFitnessScore; // ratio of inliers to total points
		bool AlignToTopCentroid;

		// bounds
		double Xmin;
		double Xmax;
		double Ymin;
		double Ymax;
		double Zmin;
		double Zmax;
	};

public:
	SphereSegmentation();
	virtual ~SphereSegmentation();

	void setParameters(const Parameters &parameters);
	Parameters getParameters();

	// gets parameters from ros
	void fetchParameters(std::string nameSpace = "");

	/*
	 * segments a sphere and produces collision object in world coordinates
	 */
	bool segment(const sensor_msgs::PointCloud &cloudMsg,arm_navigation_msgs::CollisionObject &obj);
	bool segment(const sensor_msgs::PointCloud2 &cloudMsg,arm_navigation_msgs::CollisionObject &obj);
	bool segment(const std::vector<sensor_msgs::PointCloud> &clusters,arm_navigation_msgs::CollisionObject &obj,int &bestClusterIndex);
	bool segment(const Cloud3D &cluster,arm_navigation_msgs::CollisionObject &obj);

	// returns the cluster corresponding to the inliers of the last sphere model found
	void getSphereCluster(sensor_msgs::PointCloud &cluster);

protected:

	/*
	 * this method uses the highest point in the cluster in order to infer the sphere's location
	 */
	bool findSphereUsingTopPoint(const Cloud3D &cloud,pcl::ModelCoefficients::Ptr coefficients,pcl::PointIndices::Ptr inliers);

	bool findTopCentroid(const Cloud3D &cloud,pcl::PointXYZ &topCentroid);
	/*
	 * uses ransac
	 */
	bool performSphereSegmentation(const Cloud3D &cloud,pcl::ModelCoefficients::Ptr coefficients,pcl::PointIndices::Ptr inliers);

	void filterBounds(Cloud3D &cloud);

	// creates polygon with "point" at its center and extract all points that fall within the bounds of the extruded body
	void filterWithPolygonalPrism(Cloud3D &cloud,pcl::PointXYZ &point,int nSides,double radius,double heightMax,double heightMin);

	void concatenateClouds(const std::vector<sensor_msgs::PointCloud> &clusters,Cloud3D &cluster);

	void createObject(const pcl::ModelCoefficients &coeffs,arm_navigation_msgs::CollisionObject &obj);

	// parameters
	Parameters _Parameters;

	// transform mapping
	tf::TransformListener _TfListener;

	// results from last segmentation
	double _LastSegmentationScore;
	pcl::PointIndices _LastIndices;
	pcl::ModelCoefficients _LastCoefficients;
	Cloud3D _LastSphereSegCluster;
	bool _LastSphereSegSuccess;

};

#endif /* SPHERESEGMENTATION_H_ */
