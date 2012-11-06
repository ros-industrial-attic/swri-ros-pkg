/*
 * PickPlaceZoneSelector.cpp
 *
 *  Created on: Oct 22, 2012
 *      Author: jnicho
 */

#include <mantis_object_manipulation/zone_selection/PickPlaceZoneSelector.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/common/transforms.h> // allows conversions between ros msgs and pcl types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PclCloud;

PickPlaceZoneSelector::PickPlaceZoneSelector()
:pick_zone_index(0),
 place_zone_index(1),
 TabletopSegNamespace(TABLETOP_SEGMT_DEFAULT_NAMESPACE),
 TabletopSegXmaxName(TABLETOP_SEGMT_XMAX_NAME),
 TabletopSegXminName(TABLETOP_SEGMT_XMIN_NAME),
 TabletopSegYminName(TABLETOP_SEGMT_YMIN_NAME),
 TabletopSegYmaxName(TABLETOP_SEGMT_YMAX_NAME),
 place_zone_()
{
	// TODO Auto-generated constructor stub

}

PickPlaceZoneSelector::~PickPlaceZoneSelector() {
	// TODO Auto-generated destructor stub
}

void PickPlaceZoneSelector::swapPickPlaceZones()
{
	// swapping zones
	int current_pick_zone_index = pick_zone_index;
	pick_zone_index = place_zone_index;
	place_zone_index = current_pick_zone_index;

	// resetting place location
	place_zone_.resetZone(getPlaceZoneCenter());

	// updating place zone radius
	place_zone_.setZoneRadius(getPlaceZoneRadius());

	// updating tabletop segmentation bounds
	//updateTabletopSegmentationBounds();
}

tf::Vector3 PickPlaceZoneSelector::getPlaceZoneCenter()
{
	ZoneBounds &placeZone = Zones[place_zone_index];
	tf::Vector3 zoneCenter;
	zoneCenter.setX(0.5f*(placeZone.XMax + placeZone.XMin));
	zoneCenter.setY(0.5f*(placeZone.YMax + placeZone.YMin));
	zoneCenter.setZ(0.0f);

	return zoneCenter;
}

double PickPlaceZoneSelector::getPlaceZoneRadius()
{
	ZoneBounds &placeZone = Zones[place_zone_index];
	double xSpan = std::abs(placeZone.XMax - placeZone.XMin);
	double ySpan = std::abs(placeZone.YMax - placeZone.YMin);

	return (xSpan > ySpan) ? (ySpan/2) : (xSpan/2);
}

tf::Vector3 PickPlaceZoneSelector::getPlaceZoneSize()
{
	ZoneBounds &placeZone = Zones[place_zone_index];
	tf::Vector3 zoneSize;
	zoneSize.setX(std::abs(placeZone.XMax - placeZone.XMin));
	zoneSize.setY(std::abs(placeZone.YMax - placeZone.YMin));
	zoneSize.setZ(0.0f);

	return zoneSize;
}

bool PickPlaceZoneSelector::isInPickZone(const std::vector<sensor_msgs::PointCloud> &clusters,std::vector<int> &inZone)
{
	inZone.clear();

	for(unsigned int i = 0; i < clusters.size(); i++)
	{
		if(isInPickZone(clusters[i]))
		{
			inZone.push_back(i);
		}
	}

	return !inZone.empty();
}

bool PickPlaceZoneSelector::isInPickZone(const sensor_msgs::PointCloud &cluster)
{

	// converting clouds
	PclCloud cloud;
	sensor_msgs::PointCloud2 clusterMsg;
	sensor_msgs::convertPointCloudToPointCloud2(cluster,clusterMsg);
	pcl::fromROSMsg(clusterMsg,cloud);

	// finding centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud,centroid);

	// checking if centroid of cloud is in bounds of pick zone
	pcl::PointXYZ clusterCentroid;
	clusterCentroid.x = centroid[0];
	clusterCentroid.y = centroid[1];

	ZoneBounds &pickZone = Zones[pick_zone_index];

	if(((pickZone.XMin > clusterCentroid.x) || (pickZone.XMax < clusterCentroid.x)) ||
			((pickZone.YMin > clusterCentroid.y) || (pickZone.YMax < clusterCentroid.y)))
	{
		return false;
	}

	return true;
}

//void PickPlaceZoneSelector::updateTabletopSegmentationBounds()
//{
//	ros::NodeHandle nh;
//
//	ZoneBounds &pickZoneBounds = Zones[pick_zone_index];
//	ros::param::set(TabletopSegNamespace + "/" + TabletopSegXminName,pickZoneBounds.XMin);
//	ros::param::set(TabletopSegNamespace + "/" + TabletopSegXmaxName,pickZoneBounds.XMax);
//	ros::param::set(TabletopSegNamespace + "/" + TabletopSegYminName,pickZoneBounds.YMin);
//	ros::param::set(TabletopSegNamespace + "/" + TabletopSegYmaxName,pickZoneBounds.YMax);
//}

void PickPlaceZoneSelector::zoneBoundsToMarker(const PickPlaceZoneSelector::ZoneBounds &bounds,visualization_msgs::Marker &marker)
{
	tf::Vector3 zoneSize;
	zoneSize.setX(std::abs(bounds.XMax - bounds.XMin));
	zoneSize.setY(std::abs(bounds.YMax - bounds.YMin));
	zoneSize.setZ(0.0f);


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

void PickPlaceZoneSelector::getPickZoneMarker(visualization_msgs::Marker &marker)
{
	ZoneBounds &zone = Zones[pick_zone_index];
	zoneBoundsToMarker(zone,marker);

	std_msgs::ColorRGBA color;
	color.r = 1.0f;
	color.g = 1.0f;
	color.b = 0.0f;
	color.a = 0.4f;

	// computing transform
	tf::Vector3 center;
	center.setX(std::abs(0.5f*(zone.XMax + zone.XMin)));
	center.setY(std::abs(0.5f*(zone.YMax + zone.YMin)));
	center.setZ(0.0f);
	tf::Quaternion q = tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),0.0f);
	tf::Transform zoneTf = tf::Transform(q,center);
	tf::poseTFToMsg(zoneTf,marker.pose);

	// filling additional fields
	marker.color = color;
	marker.header.frame_id = place_zone_.FrameId;
}

void PickPlaceZoneSelector::getPlaceZoneMarker(visualization_msgs::Marker &marker)
{
	ZoneBounds &zone = Zones[place_zone_index];
	zoneBoundsToMarker(zone,marker);

	std_msgs::ColorRGBA color;
	color.r = 72.0f/255.0f;
	color.g = 209.0f/255.0f;
	color.b = 204.0f/255.0f;
	color.a = 0.4f;

	marker.color = color;
	marker.header.frame_id = place_zone_.FrameId;
	marker.pose = place_zone_.getZoneCenterPose();
}

//void PickPlaceZoneSelector::PlaceZone::resetZone(const tf::Vector3 &zoneCenter)
//{
//	place_zone_center_ = zoneCenter;
//	objects_in_zone_.clear();
//}

void PickPlaceZoneSelector::PlaceZone::resetZone(const PickPlaceZoneSelector::ZoneBounds &bounds)
{
	place_zone_bounds_ = bounds;
}

void PickPlaceZoneSelector::PlaceZone::setGraspObjectSize(const tf::Vector3 &size)
{
	grasped_object_size_ = size;
}

//void PickPlaceZoneSelector::PlaceZone::setZoneRadius(double radius)
//{
//	place_zone_radius_ = radius;
//}

bool PickPlaceZoneSelector::PlaceZone::generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	bool success = false;

	switch(GenerationMode)
	{
	case PickPlaceZoneSelector::PlaceZone::NextPoseGenerationMode::RANDOM:

		success = generateNextPlacePoseInRandomizedMode(placePoses);
		break;

	case PickPlaceZoneSelector::PlaceZone::NextPoseGenerationMode::DESIGNATED_ZIGZAG:

		success = generateNextPlacePoseInDesignatedEvenOddMode(placePoses);
		break

	default:

		success = generateNextPlacePoseInDesignatedEvenOddMode(placePoses);
		break;
	}

	return success;
}

void PickPlaceZoneSelector::PlaceZone::createPlaceCandidatePosesByRotation(const tf::Transform &startTrans,
		int numCandidates,tf::Vector3 axis, std::vector<geometry_msgs::PoseStamped> &candidatePoses)
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = FrameId;

	// rotate about z axis and apply to original goal pose in order to create candidates;
	for(int i = 0; i < numCandidates; i++)
	{
		double ratio = ((double)i)/((double)numCandidates);
		double angle = 2*M_PI*ratio;
		tf::Quaternion q = tf::Quaternion(axis,angle);
		tf::Vector3 p = tf::Vector3(0,0,0);
		tf::Transform candidateTransform = startTrans*tf::Transform(q,p);
		tf::poseTFToMsg(candidateTransform,pose.pose);
		candidatePoses.push_back(pose);
	}
}

geometry_msgs::Pose PickPlaceZoneSelector::PlaceZone::getZoneCenterPose()
{
	geometry_msgs::Pose pose;
	tf::Quaternion rot = tf::Quaternion(Axis,0.0f);
	tf::Vector3 pos = tf::Vector3(place_zone_center_.x(),place_zone_center_.y(),0.0f);
	tf::poseTFToMsg(tf::Transform(rot,pos),pose);
	return pose;
}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInRandomizedMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	// previos pose
	tf::Transform &lastTf = objects_in_zone_.back().Trans;

	// returned bool
	bool foundNewPlaceLocation = false;

	// place zone details
	tf::Vector3 placeZoneCenter = place_zone_bounds_.getCenter();

	// next object details
	ZoneBounds nextObjectBounds = ZoneBounds(next_object_details_.Size,next_object_details_.Trans.getOrigin());

	// next pose
	tf::Transform nextTf;
	if(objects_in_zone_.size() == 0)
	{
		tf::Quaternion rot = tf::Quaternion(Axis,0.0f);
		// the z component equals the height of the object plus the requested release distance

		tf::Vector3 pos = tf::Vector3(placeZoneCenter.x(),
				placeZoneCenter.y(),next_object_details_.Size.z() + ReleaseDistanceFromTable);
		nextTf = tf::Transform(rot,pos);
		foundNewPlaceLocation = true;
	}
	else
	{
		nextTf = tf::Transform(objects_in_zone_.back().Trans);

		// last object details
		ZoneBounds lastObjectBounds(objects_in_zone_.back().Size,objects_in_zone_.back().Trans.getOrigin());
		// sum of the bounding radius of the next and last objects
		double radialDistance = nextObjectBounds.getBoundingRadius() + lastObjectBounds.getBoundingRadius();

		// new location variables
		int distanceSegments = 20; // number of possible values between min and max object spacing
		int angleSegments = 8; // number of possible values between 0 and 2pi
		int randVal;
		double distance; // number between Max and Min Object spacing parameters.  Should be larger than radial distance (meters)
		double angle,angleMin = 0,angleMax = 2*M_PI; // radians
		double ratio;

		// creating new location relative to the last object
		int maxIterations = 200;
		int iter = 0;
		while(iter < maxIterations)
		{
			iter++;

			// computing distance
			randVal = rand()%distanceSegments + 1;
			ratio = (double)randVal/(double)distanceSegments;
			distance = MinObjectSpacing + ratio*(MaxObjectSpacing - MinObjectSpacing);
			distance = (distance > radialDistance ? distance : radialDistance);// choosing largest between distance and radial distance
			tf::Vector3 trans = tf::Vector3(distance,0.0f,0.0f);

			// computing angle
			randVal = rand()%angleSegments + 1;
			ratio = (double)randVal/(double)angleSegments;
			angle = angleMin + ratio*(angleMax - angleMin);
			tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),angle);

			// computing next pose by rotating and translating from last pose
			nextTf = lastTf * tf::Transform(quat,tf::Vector3(0.0f,0.0f,0.0f))*tf::Transform(tf::Quaternion::getIdentity(),trans);

			// updating next object bounds to new candidate location
			nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

			// checking if located inside place region
			if(!ZoneBounds::contains(place_zone_bounds_,nextObjectBounds))
			{
				continue;
			}

			// checking for overlaps against objects already in place region
			bool overlapFound = false;

			typedef  const std::vector<ObjectDetails>::iterator ConstIter;
			for(ConstIter iter = objects_in_zone_.begin(); iter != objects_in_zone_.end(); iter++)
			{
				ZoneBounds objInZoneBounds(iter->Size,iter->Trans.getOrigin());
				if(ZoneBounds::intersect(nextObjectBounds,objInZoneBounds))
				{
					overlapFound = true;
					break;
				}
			}

			if(overlapFound)
			{
				continue; // try again
			}

			double distFromCenter = (place_zone_bounds_.getCenter() - nextTf.getOrigin()).distance();
			ROS_INFO_STREAM(ros::this_node::getName()<<": Found available position at a distance of "<< distFromCenter
					<<" from the center after "<<iter<<" iterations");

			// adjusting place point to object height
			nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);
			foundNewPlaceLocation = true;
			break;
		}

		if(!foundNewPlaceLocation)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": Could not find available position after "<<iter<<" iterations");
			return false;
		}
	}

	// generating candidate poses from next location found
	createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

	// storing object
	next_object_details_.Trans = nextTf;
	objects_in_zone_.push_back(next_object_details_);

	return foundNewPlaceLocation;
}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInDesignatedEvenOddMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	// next object details
	ZoneBounds nextObjectBounds;;
	double nextIndex = (double)next_object_details_.Id;

	// will use next object id (even or odd) to determine its location
	tf::Transform nextTf = tf::Transform::getIdentity();

	/* will use evenness of next object index to compute a new location relative to the top left corner of the place zone.
	 * Odds go to the top and evens at the bottom (top view of table)
	 */
	double xCoor = (place_zone_bounds_.XMin + MinObjectSpacing/2.0f) + ((int)std::ceil(nextIndex/2.0f) - 1)*MinObjectSpacing;
	double yCoor = (((int)nextIndex%2) == 0) ? (place_zone_bounds_.YMin + MinObjectSpacing/2.0f) : (place_zone_bounds_.YMax - MinObjectSpacing/2.0f);

	// computing next candidate transform
	nextTf.setOrigin(tf::Vector3(xCoor,yCoor,0.0f));

	// updating next object bounds
	nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

	// checking if it is within place zone
	if(!ZoneBounds::contains(place_zone_bounds_,nextObjectBounds))
	{
		return false;
	}

	// checking if overlaps with objects in place zone
	bool overlapFound = false;
	typedef  const std::vector<ObjectDetails>::iterator ConstIter;
	for(ConstIter iter = objects_in_zone_.begin(); iter != objects_in_zone_.end(); iter++)
	{
		ZoneBounds objInZoneBounds(iter->Size,iter->Trans.getOrigin());
		if(ZoneBounds::intersect(nextObjectBounds,objInZoneBounds))
		{
			overlapFound = true;
			break;
		}
	}

	if(overlapFound)
	{
		return false;
	}

	// passed all intersection test
	double distFromCenter = (place_zone_bounds_.getCenter() - nextTf.getOrigin()).distance();
	ROS_INFO_STREAM(ros::this_node::getName()<<": Found available position for Id: "<<next_object_details_.Id);

	// adjusting place point to object height
	nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);

	// generating candidate poses from next location found
	createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

	// storing object
	next_object_details_.Trans = nextTf;
	objects_in_zone_.push_back(next_object_details_);

	return true;
}

