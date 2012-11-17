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
#include <typeinfo>

// paramters used by the tabletop segmentation service
const std::string TABLETOP_SEGMT_DEFAULT_NAMESPACE = "tabletop_segmentation";
const std::string TABLETOP_SEGMT_XMIN_NAME = "x_filter_min";
const std::string TABLETOP_SEGMT_XMAX_NAME = "x_filter_max";
const std::string TABLETOP_SEGMT_YMIN_NAME = "y_filter_min";
const std::string TABLETOP_SEGMT_YMAX_NAME = "y_filter_max";

class PickPlaceZoneSelector
{
public:

	struct ZoneBounds
	{
	public:

		ZoneBounds(){}
		ZoneBounds(tf::Vector3 size, tf::Vector3 pos,std::string frameId = "base_link",std::string name = "")
		:FrameId(frameId),
		 ZoneName(name)
		{
			tf::Vector3 topLeft = tf::Transform(tf::Quaternion::getIdentity(),
					pos)*tf::Vector3(size.x()/2.0f,size.y()/2.0f,0.0f);
			XMax = topLeft.x();
			XMin = topLeft.x() - size.x();
			YMax = topLeft.y();
			YMin = topLeft.y() - size.y();

		}

		double XMin;
		double XMax;
		double YMin;
		double YMax;
		std::string ZoneName;
		std::string FrameId;

		tf::Vector3 getSize() const
		{
			return tf::Vector3(std::abs(XMax - XMin),std::abs(YMax - YMin),0.0f);
		}

		tf::Vector3 getCenter() const
		{
			tf::Vector3 size = getSize();
			return tf::Vector3(XMin + size.x()/2.0f,YMin + size.y()/2.0f,0.0f);
		}

		void getMarker(visualization_msgs::Marker &marker) const
		{
			tf::Vector3 zoneSize = getSize();

			//marker.header.frame_id = place_zone_.FrameId;
			//marker.pose = place_zone_.getZoneCenterPose();
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale.x = zoneSize.x();
			marker.scale.y = zoneSize.y();
			marker.scale.z = 1;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}

		double getBoundingRadius() const
		{
			tf::Vector3 size = getSize();
			return 	std::sqrt(std::pow(size.x()/2.0f,2.0f) + std::pow(size.y()/2.0f,2.0f));
		}

		static bool intersect(const ZoneBounds& zone1, const ZoneBounds& zone2)
		{
			// sizes
			tf::Vector3 z1Size = zone1.getSize();
			tf::Vector3 z2Size = zone2.getSize();

			// centers
			tf::Vector3 z1Center = zone1.getCenter();
			tf::Vector3 z2Center = zone2.getCenter();

			double xIntersectDist = std::abs(z1Size.x() + z2Size.x())/2.0f;
			double yIntersectDist = std::abs(z1Size.y() + z2Size.y())/2.0f;

			tf::Vector3 dist= z1Center - z2Center;
			return (std::abs(dist.x()) < xIntersectDist) && (std::abs(dist.y()) < yIntersectDist);
		}

		static bool boundingCirclesIntersect(const ZoneBounds& z1, const ZoneBounds& z2)
		{
			double sumRadius = z1.getBoundingRadius() + z2.getBoundingRadius();
			double distance = (z1.getCenter() - z2.getCenter()).length();

			return distance <= sumRadius;
		}

		static bool contains(const ZoneBounds& outer, const ZoneBounds& inner)
		{
			// checking sizes first
			tf::Vector3 bigBoxSize = outer.getSize();
			tf::Vector3 smallBoxSize = inner.getSize();

			if( (bigBoxSize.x() < smallBoxSize.x()) || ( bigBoxSize.y() < smallBoxSize.y()) )
			{
				return false;
			}

			// centers
			tf::Vector3 bigBoxCenter = outer.getCenter();
			tf::Vector3 smallBoxCenter = inner.getCenter();

			tf::Vector3 containDist = (bigBoxSize - smallBoxSize)/tfScalar(2.0f);
			tf::Vector3 dist = bigBoxCenter - smallBoxCenter;

			return (std::abs(dist.x()) < containDist.x()) && ( std::abs(dist.y()) < containDist.y() );
		}

		bool parseParam(XmlRpc::XmlRpcValue& val)
		{
			std::string structMember;
			if(val.getType() == XmlRpc::XmlRpcValue::TypeStruct)
			{
				structMember = "x_min"; XMin = static_cast<double>(val[structMember]);
				structMember = "x_max"; XMax = static_cast<double>(val[structMember]);
				structMember = "y_min"; YMin = static_cast<double>(val[structMember]);
				structMember = "y_max"; YMax = static_cast<double>(val[structMember]);
				structMember = "zone_name"; ZoneName = static_cast<std::string>(val[structMember]);
				structMember = "frame_id"; FrameId = static_cast<std::string>(val[structMember]);
				//structMember = "next_location_gen_mode"; zone.NextLocationGenMode = static_cast<int>(val[structMember]);
			}
			else
			{
				return false;
			}

			return true;
		}

	};

	struct ObjectDetails
	{
		ObjectDetails(){}
		ObjectDetails(const tf::Transform &trans, const tf::Vector3 &size, int id = 0,std::string tag = "")
		:Trans(trans),
		 Size(size),
		 Id(id),
		 Tag(tag)
		{

		}


		tf::Transform Trans;
		tf::Vector3 Size;
		int Id;
		std::string Tag;
	};

	struct PlaceZone : public ZoneBounds
	{
	public:
		enum NextPoseGenerationMode
		{
			RANDOM = 0,
			GRID_ALONG_X = 1,
			GRID_ALONG_Y = 2,
			CIRCLE = 4,
			ZIGZAG_ALONG_X = 5,
			ZIGZAG_ALONG_Y = 6
		};

	public:
		PlaceZone()
		:NumGoalCandidates(8),
		 Axis(0,0,1.0f),
		 ReleaseDistanceFromTable(0.02),// 2cm
		 MinObjectSpacing(0.05f),
		 MaxObjectSpacing(0.08f),
		 objects_in_zone_()
		{
			srand(time(NULL));
		}

		PlaceZone(tf::Vector3 size, tf::Vector3 pos)
		:ZoneBounds(size,pos),
		 NumGoalCandidates(8),
		 Axis(0,0,1.0f),
		 ReleaseDistanceFromTable(0.02),// 2cm
		 MinObjectSpacing(0.05f),
		 MaxObjectSpacing(0.08f),
		 objects_in_zone_()
		{

		}

		virtual ~PlaceZone()
		{

		}

		bool parseParam(XmlRpc::XmlRpcValue& val)
		{
			if(!ZoneBounds::parseParam(val))
			{
				return false;
			}

			std::string structMember;
			double x = 0.0f,y = 0.0f,z = 0.0f;
			if(val.getType() == XmlRpc::XmlRpcValue::TypeStruct)
			{
				//structMember = "frame_id"; FrameId = static_cast<std::string>(val[structMember]);
				structMember = "pose_candidates"; NumGoalCandidates = static_cast<int>(val[structMember]);
				structMember = "release_distance"; ReleaseDistanceFromTable = static_cast<double>(val[structMember]);

				XmlRpc::XmlRpcValue nextParam;
				structMember = "next_location"; nextParam = val[structMember];
				structMember = "min_spacing"; MinObjectSpacing = static_cast<double>(nextParam[structMember]);
				structMember = "max_spacing"; MinObjectSpacing = static_cast<double>(nextParam[structMember]);
				structMember = "generation_mode"; NextLocationGenMode = static_cast<int>(nextParam[structMember]);

				XmlRpc::XmlRpcValue axisParam;
				structMember = "orientation_axis"; axisParam = val[structMember];
				structMember = "x"; x = static_cast<double>(axisParam[structMember]);
				structMember = "y"; y = static_cast<double>(axisParam[structMember]);
				structMember = "z"; z = static_cast<double>(axisParam[structMember]);
				Axis = tf::Vector3(x,y,z).normalized();

				ROS_INFO_STREAM(ros::this_node::getName()<<": Axis: "<<Axis.x()<<", "<<Axis.y()<<", "<<Axis.z());

				// parsing allowed id's array
				structMember = "ids_allowed";
				XmlRpc::XmlRpcValue idArray = val[structMember];
				if((idArray.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
						(idArray[0].getType() == XmlRpc::XmlRpcValue::TypeInt))
				{
					for(unsigned int i = 0; i < idArray.size(); i++)
					{
						int id = static_cast<int>(idArray[i]);
						Ids.push_back(id);
					}

				}
				else
				{
					return false;
				}
			}

			resetZone();
			return true;
		}

		void setNextObjectDetails(const ObjectDetails &objectDetails);

		bool isIdInZone(int i);

		bool generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);

		void resetZone(); // clear object list

		geometry_msgs::Pose getZoneCenterPose();

		void getMarker(visualization_msgs::Marker &marker) const
		{
			ZoneBounds::getMarker(marker);
			marker.scale.z = 1.6;
		}

	public:

		// ros parameters
		//std::string FrameId;
		int NumGoalCandidates; // number of total goal transforms, each additional transform is produced by rotating
								// about an axis by a specified angle
		tf::Vector3 Axis; // used in producing additional goal candidates
		double ReleaseDistanceFromTable; // how far from the surface the lowest part of the object should be when released

		// Next goal position generation
		double MinObjectSpacing; // minimum distance between two objects inside goal region as measured from their local origins
		double MaxObjectSpacing; // maximum distance between two objects inside goal region as measured from their local origin
		int NextLocationGenMode; // one of the supported enumeration values that determines how to generate the next location
		std::vector<int> Ids; // Array of object id's allowed in this zone

	protected:

		void createPlaceCandidatePosesByRotation(const tf::Transform &startTrans,int numCandidates,tf::Vector3 axis,
						std::vector<geometry_msgs::PoseStamped> &candidatePoses);

		bool generateNextPlacePoseInRandomizedMode(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);
		bool generateNextPlacePoseInZigZagXMode(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);
		bool generateNextPlacePoseInZigZagYMode(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);
		bool generateNextPlacePoseInGridXWise(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);
		bool generateNextPlacePoseInGridYWise(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);
		bool generateNextPlacePoseInCircle(std::vector<geometry_msgs::PoseStamped> &placePoses,std::vector<PlaceZone* > &otherZones);

		// check overlaps with object in theses zones.
		bool checkOverlaps(ZoneBounds &nextObjBounds,std::vector<PlaceZone* > zones);

		// general zone member
		std::vector<ObjectDetails> objects_in_zone_; // array of transforms and size for each object in zone
		ObjectDetails next_object_details_;

		// grid mode members
		int grid_x_size_;
		int grid_y_size_;

	};

public:
	PickPlaceZoneSelector();
	virtual ~PickPlaceZoneSelector();

	void fetchParameters(std::string nameSpace = "")
	{
		ros::NodeHandle nh;
		std::string nodeName = ros::this_node::getName();

		// getting place zones
		XmlRpc::XmlRpcValue placeZonesParam;
		place_zones_.clear();
		if(ros::param::get(nameSpace + "/place_zones",placeZonesParam))
		{
			ROS_INFO_STREAM(nodeName<<": found place_zones struct array with "<<placeZonesParam.size()<<" elements");

			// checking if array contains structure elements
			if(placeZonesParam.getType()==XmlRpc::XmlRpcValue::TypeArray && placeZonesParam.size() > 0 &&
					(placeZonesParam[0].getType() == XmlRpc::XmlRpcValue::TypeStruct))
			{
				for(int i = 0; i < placeZonesParam.size(); i++)
				{
					PlaceZone zone;
					if(!zone.parseParam(placeZonesParam[i]))
					{
						ROS_ERROR_STREAM(nodeName<<": Error parsing place zone");
						break;
					}
					place_zones_.push_back(zone);
				}
			}

		}
		else
		{
			ROS_ERROR_STREAM(ros::this_node::getName()<<": did not find place_zones structure array under "<<nameSpace + "/place_zones");
		}

		// getting pick zones
		XmlRpc::XmlRpcValue pickZoneParam;
		pick_zones_.clear();
		if(ros::param::get(nameSpace + "/pick_zones",pickZoneParam) && (pickZoneParam.getType() == XmlRpc::XmlRpcValue::TypeArray))
		{
			ROS_INFO_STREAM(nodeName<<": found pick_zones struct array with "<<pickZoneParam.size()<<" elements");

			for(int  i = 0; i < pickZoneParam.size(); i++)
			{
				ZoneBounds pickZone;
				if(!pickZone.parseParam(pickZoneParam[i]))
				{
					ROS_ERROR_STREAM(nodeName<<": Error parsing pick zone");
					break;
				}
				pick_zones_.push_back(pickZone);
			}
		}
		else
		{
			ROS_ERROR_STREAM(ros::this_node::getName()<<": did not find pick_zones structure array under "<<nameSpace + "/pick_zones");
		}

		ROS_INFO_STREAM(nodeName<<": Completed parsing zone arrays, setting up zone selector");

		goToNextPickZone();

		ROS_INFO_STREAM(nodeName<<": Zone Selector is ready");
	}

	void goToNextPickZone(); //
	bool isInPickZone(const std::vector<sensor_msgs::PointCloud> &clusters,std::vector<int> &inZone);
	bool isInPickZone(const sensor_msgs::PointCloud &cluster);

	bool generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses);


	void setNextObjectDetails(const ObjectDetails &obj)
	{
		next_obj_details_ = obj;
	}

	void getActivePlaceZonesMarkers(visualization_msgs::MarkerArray &markers);
	void getPickZoneMarker(visualization_msgs::Marker &marker);
	void getAllActiveZonesMarkers(visualization_msgs::MarkerArray &markers); // includes pick and active place zones

	const std::vector<ZoneBounds>& getPickZones()
	{
		return pick_zones_;
	}

	const std::vector<PlaceZone>& getAllPlaceZones()
	{
		return place_zones_;
	}

protected:

	void initializeColorArray();


protected:

	// ros parameters
	std::vector<ZoneBounds> pick_zones_;
	std::vector<PlaceZone> place_zones_;

	// internal
	int pick_zone_index_; // index to current pick zone in array "Zones"
	std::vector<PlaceZone* > active_place_zones_;  // reference array to place zones that do not overlap with current pick zone
	std::vector<PlaceZone* > inactive_place_zones_;
	ObjectDetails next_obj_details_;

	// markers
	std::vector<std_msgs::ColorRGBA> marker_colors_;
};

#endif /* PICKPLACEZONESELECTOR_H_ */
