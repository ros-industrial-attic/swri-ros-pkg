/*
 * PickPlaceZoneSelector.h
 *
 *  Created on: Oct 22, 2012
 *      Author: jnicho
 */

#ifndef PICKPLACEZONESELECTOR_H_
#define PICKPLACEZONESELECTOR_H_

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/Shape.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <ros/ros.h>

// paramters used by the tabletop segmentation service
const std::string TABLETOP_SEGMT_DEFAULT_NAMESPACE = "tabletop_segmentation";
const std::string TABLETOP_SEGMT_XMIN_NAME = "x_filter_min";
const std::string TABLETOP_SEGMT_XMAX_NAME = "x_filter_max";
const std::string TABLETOP_SEGMT_YMIN_NAME = "y_filter_min";
const std::string TABLETOP_SEGMT_YMAX_NAME = "y_filter_max";

class PickPlaceZoneSelector
{
public:

	struct PlaceZone
	{
	public:
		PlaceZone()
		:FrameId("base_link"),
		 NumGoalCandidates(8),
		 Axis(0,0,1.0f),
		 ReleaseDistanceFromTable(0.02),// 2cm
		 MinObjectSpacing(0.05f),
		 MaxObjectSpacing(0.08f),
		 place_zone_center_(0.0f,0.0f,0.0f),
		 place_zone_radius_(0.20f),
		 bodies_in_zone_(),
		 grasped_object_size_(0.05,0.05f,0.05f)
		{
			srand(time(NULL));
		}
		virtual ~PlaceZone()
		{

		}

		void fetchParameters(std::string nameSpace)
		{
			// reference coordinate frame for zone
			ros::param::param(nameSpace + "/frame_id",FrameId,FrameId);

			// number of candidates
			ros::param::param(nameSpace + "/pose_candidates",NumGoalCandidates,NumGoalCandidates);
			if(NumGoalCandidates <= 0)
			{
				NumGoalCandidates = 1;
			}

			// axis for producing additional candidates
			double x = 0, y = 0,z = 0;
			ros::param::param(nameSpace + "/orientation_axis/x",x,Axis.x());
			ros::param::param(nameSpace + "/orientation_axis/y",y,Axis.y());
			ros::param::param(nameSpace + "/orientation_axis/z",z,Axis.z());
			Axis = tf::Vector3(x,y,z).normalized();

			// distance of bottom surface of object to resting surface
			ros::param::param(nameSpace + "/release_distance",ReleaseDistanceFromTable,ReleaseDistanceFromTable);

			// next goal location generation parameters
			ros::param::param(nameSpace + "/next_location/min_spacing",MinObjectSpacing,MinObjectSpacing);
			ros::param::param(nameSpace + "/next_location/max_spacing",MaxObjectSpacing,MaxObjectSpacing);
			//ros::param::param(nameSpace + "/next_location/place_region_radius",PlaceZoneRadius,PlaceZoneRadius);
		}

		void setGraspObjectSize(const tf::Vector3 &size);
		void setZoneRadius(double radius);
		bool generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses);
		void resetZone(const tf::Vector3 &zoneCenter); // clears occupied locations and sets new zone center

		geometry_msgs::Pose getZoneCenterPose();

	public:

		// ros parameters
		std::string FrameId;
		int NumGoalCandidates; // number of total goal transforms, each additional transform is produced by rotating
								// about an axis by a specified angle
		tf::Vector3 Axis; // used in producing additional goal candidates
		double ReleaseDistanceFromTable; // how far from the surface the lowest part of the object should be when released

		// Next goal position generation
		double MinObjectSpacing; // minimum distance between two objects inside goal region as measured from their local origins
		double MaxObjectSpacing; // maximum distance between two objects inside goal region as measured from their local origin

	protected:

		void createPlaceCandidatePosesByRotation(const tf::Transform &startTrans,int numCandidates,tf::Vector3 axis,
						std::vector<geometry_msgs::PoseStamped> &candidatePoses);

		bool generateNextPlacePosesInShuffleMode(std::vector<geometry_msgs::PoseStamped> &placePoses);

		tf::Vector3 place_zone_center_; // used as the center of place zone.  Only the x and y values are used
		double place_zone_radius_;// radius of circular region that contains all placed objects
		std::vector<tf::Transform> bodies_in_zone_;
		tf::Vector3 grasped_object_size_;

	};

	struct ZoneBounds
	{
	public:

		double XMin;
		double XMax;
		double YMin;
		double YMax;
		std::string ZoneName;
	};

public:
	PickPlaceZoneSelector();
	virtual ~PickPlaceZoneSelector();

	void fetchParameters(std::string nameSpace = "")
	{
		ros::NodeHandle nh;

		XmlRpc::XmlRpcValue list;
		if(ros::param::get(nameSpace + "/zone_bounds",list))
		{
			ROS_INFO_STREAM(ros::this_node::getName()<<": found zone_bounds struct array with "<<list.size()<<" elements");
			if(list.getType()==XmlRpc::XmlRpcValue::TypeArray && list.size() > 0 &&
					(list[0].getType() == XmlRpc::XmlRpcValue::TypeStruct))
			{
				ROS_INFO_STREAM(ros::this_node::getName()<<": zone_bounds struct met type conditions");
				Zones.clear();
				for(int i = 0; i < list.size(); i++)
				{
					ZoneBounds zone;
					std::string structMember;
					XmlRpc::XmlRpcValue &val = list[i];
					if(val.getType() == XmlRpc::XmlRpcValue::TypeStruct)
					{
						structMember = "x_min"; zone.XMin = static_cast<double>(val[structMember]);
						structMember = "x_max"; zone.XMax = static_cast<double>(val[structMember]);
						structMember = "y_min"; zone.YMin = static_cast<double>(val[structMember]);
						structMember = "y_max"; zone.YMax = static_cast<double>(val[structMember]);
						structMember = "zone_name"; zone.ZoneName = static_cast<std::string>(val[structMember]);
					}

					Zones.push_back(zone);
				}
			}
			else
			{
				ROS_WARN_STREAM(ros::this_node::getName()<<": zone_bounds struct did not meet type requirements");
			}
		}
		else
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": did not find zone_bounds struct array");
		}


		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/namespace",TabletopSegNamespace,TABLETOP_SEGMT_DEFAULT_NAMESPACE);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/xmin",TabletopSegXminName,TABLETOP_SEGMT_XMIN_NAME);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/xmax",TabletopSegXmaxName,TABLETOP_SEGMT_XMAX_NAME);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/ymin",TabletopSegYminName,TABLETOP_SEGMT_YMIN_NAME);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/ymax",TabletopSegYmaxName,TABLETOP_SEGMT_YMAX_NAME);

		// getting parameters for place zone
		place_zone_.fetchParameters(nameSpace + "/place_zone");

		// setting internal members
		place_zone_.resetZone(getPlaceZoneCenter());
		place_zone_.setZoneRadius(getPlaceZoneRadius());
	}

	void swapPickPlaceZones();
	bool isInPickZone(const std::vector<sensor_msgs::PointCloud> &clusters,std::vector<int> &inZone);
	bool isInPickZone(const sensor_msgs::PointCloud &cluster);

	void setGraspObjectSize(const tf::Vector3 &size)
	{
		place_zone_.setGraspObjectSize(size);
	}

	bool generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses)
	{
		return place_zone_.generateNextLocationCandidates(placePoses);
	}

	double getPlaceZoneRadius();
	tf::Vector3 getPlaceZoneCenter();
	tf::Vector3 getPlaceZoneSize();
	const PlaceZone& getPlaceZone()
	{
		return place_zone_;
	}

	void getPlaceZoneMarker(visualization_msgs::Marker &marker);
	void getPickZoneMarker(visualization_msgs::Marker &marker);

	static void zoneBoundsToMarker(const ZoneBounds &bounds,visualization_msgs::Marker &marker);

public:

	// ros parameters
	std::vector<ZoneBounds> Zones;
	std::string TabletopSegNamespace;
	std::string TabletopSegXminName;
	std::string TabletopSegXmaxName;
	std::string TabletopSegYminName;
	std::string TabletopSegYmaxName;

protected:

	void updateTabletopSegmentationBounds(); // updates tabletop segmentation parameters used by the segmentation service

	int pick_zone_index; // index to current pick zone in array "Zones"
	int place_zone_index;// index to current place zone in array "Zones"
	PlaceZone place_zone_;
};

#endif /* PICKPLACEZONESELECTOR_H_ */
