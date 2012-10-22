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
#include <ros/ros.h>

// paramters used by the tabletop segmentation service
const std::string TABLETOP_SEGMT_DEFAULT_NAMESPACE = "tabletop_segmentation";
const std::string TABLETOP_SEGMT_XMIN_NAME = "x_filter_min";
const std::string TABLETOP_SEGMT_XMAX_NAME = "x_filter_max";
const std::string TABLETOP_SEGMT_YMIN_NAME = "y_filter_min";
const std::string TABLETOP_SEGMT_YMAX_NAME = "y_filter_max";

class PickPlaceZoneSelector
{
	struct PlaceZone
	{
	public:
		PlaceZone();
		virtual ~PlaceZone(){};

		bool fetchParameters(std::string nameSpace)
		{
			// reference coordinate frame for zone
			ros::param::param(nameSpace + "/frame_id",FrameId,FrameId);

			// number of candidates
			ros::param::param(nameSpace + "/place_candidates",NumGoalCandidates,NumGoalCandidates);
			if(NumGoalCandidates <= 0)
			{
				NumGoalCandidates = 1;
			}

			// axis for producing additional candidates
			ros::param::param(nameSpace + "/place_orientation_axis/x",x,Axis.x());
			ros::param::param(nameSpace + "/place_orientation_axis/y",y,Axis.y());
			ros::param::param(nameSpace + "/place_orientation_axis/z",z,Axis.z());
			Axis = tf::Vector3(x,y,z).normalized();

			// distance of bottom surface of object to resting surface
			ros::param::param(nameSpace + "/release_distance",ReleaseDistanceFromTable,ReleaseDistanceFromTable);

			// next goal location generation parameters
			ros::param::param(nameSpace + "/next_location/min_spacing",MinObjectSpacing,MinObjectSpacing);
			ros::param::param(nameSpace + "/next_location/max_spacing",MaxObjectSpacing,MaxObjectSpacing);
			ros::param::param(nameSpace + "/next_location/place_region_radius",PlaceRegionRadius,PlaceRegionRadius);
		}

		void generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses);
		void resetZone(tf::Transform &zoneCenter); // clears occupied locations and sets new zone center

	public:

		// ros parameters
		std::string FrameId;
		int NumGoalCandidates; // number of total goal transforms, each additional transform is produced by rotating
								// about an axis by a specified angle
		tf::Vector3 Axis; // used in producing additional goal candidates
		double ReleaseDistanceFromTable; // how far from the surface the lowest part of the object should be when released

		// Next goal position generation
		double MinObjectSpacing; // minimum distance between two objects inside goal region as measured from their local origins
		double MaxObjectSpacing; // maximum distance between two objects inside goal region as measured from their local origins
		double PlaceRegionRadius; // radius of circular region that contains all placed objects

	protected:

		void generateNextLocationShuffleMode(std::vector<geometry_msgs::PoseStamped> &placePoses);

		tf::StampedTransform place_zone_transform_;
		std::vector<tf::Transform> occupied_locations_;

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

	bool fetchParameters(std::string nameSpace = "")
	{
		ros::NodeHandle nh;

		XmlRpc::XmlRpcValue list;
		if(ros::param::get(nameSpace + "/zone_bounds",list))
		{
			if(list.getType()==XmlRpc::XmlRpcValue::TypeStruct && list.size() > 0)
			{
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
		}


		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/namespace",TabletopSegNamespace,TABLETOP_SEGMT_DEFAULT_NAMESPACE);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/xmin",TabletopSegXminName,TABLETOP_SEGMT_XMIN_NAME);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/xmax",TabletopSegXmaxName,TABLETOP_SEGMT_XMAX_NAME);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/ymin",TabletopSegYminName,TABLETOP_SEGMT_YMIN_NAME);
		ros::param::param(nameSpace + "/tabletop_segmentation_names" + "/ymax",TabletopSegYmaxName,TABLETOP_SEGMT_YMAX_NAME);
	}

	void swapPickPlaceZones();
	bool isInPickZone(const std::vector<sensor_msgs::PointCloud> &clusters,std::vector<int> &inZone);
	bool isInPickZone(const sensor_msgs::PointCloud &cluster);


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
