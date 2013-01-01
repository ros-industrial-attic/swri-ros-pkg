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
#include <algorithm>
#include <boost/ptr_container/ptr_vector.hpp>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ> PclCloud;

PickPlaceZoneSelector::PickPlaceZoneSelector()
:pick_zone_index_(0),
 pick_zones_(),
 place_zones_(),
 active_place_zones_(),
 inactive_place_zones_()
{
	// TODO Auto-generated constructor stub
	initializeColorArray();

}

PickPlaceZoneSelector::~PickPlaceZoneSelector() {
	// TODO Auto-generated destructor stub
}

void PickPlaceZoneSelector::initializeColorArray()
{
	const int numColors = 40;
	const double ratio = 255.0f/((double)numColors);
	std_msgs::ColorRGBA color;

	// manually enter colors

	color.r = 0.0f;color.g = 1.0f;color.b = 1.0f; color.a = 0.4f; marker_colors_.push_back(color); // aqua
	color.r = 1.0f; color.g = 1.0f; color.b = 1.0f; color.a = 0.4f; marker_colors_.push_back(color); // white

	// random colors
	for(int i = 0; i < numColors ; i++)
	{
		color.r = (255.0f -  0.25f*((double)i)*ratio)/255.0f; // 255 to ~180
		color.g = (0.75f*255.0f + (i % 2 == 0 ? (1) : (-1))*0.25f*((double)i)*ratio)/255.0f; // ~180 to 255
		color.b = (0.80f*255.0f + (i % 2 == 0 ? (-1) : (1))*0.2f*(((double)i)*ratio))/255.0f; //[60%,100%]*255
		color.a = 0.4f;

		marker_colors_.push_back(color);

	}

	// yellow
	color.r = 1.0f; color.g = 1.0f; color.b = 0.0f; color.a = 0.2f;
	marker_colors_.push_back(color);

	//red
	//color.r = 1.0f; color.g = 0.0f; color.b = 0.0f; color.a = 0.2f;
	//marker_colors_.push_back(color);

}

void PickPlaceZoneSelector::goToPickZone(int index)
{
	if(index < (int)pick_zones_.size() && index >= 0)
	{
		while(index != pick_zone_index_)
		{
			goToNextPickZone();
		}
	}
}

bool PickPlaceZoneSelector::isPickZoneActive(int index)
{
	return index == pick_zone_index_;
}

void PickPlaceZoneSelector::goToNextPickZone()
{
	// incrementing pick zone counter
	pick_zone_index_++;
	if(pick_zone_index_  == (int)pick_zones_.size())
	{
		pick_zone_index_ = 0;
	}

	// resetting list of available place zones
	active_place_zones_.clear();
	inactive_place_zones_.clear();
	ZoneBounds &pickZone = pick_zones_[pick_zone_index_];
	for(unsigned int i = 0; i < place_zones_.size(); i++)
	{
		PlaceZone &placeZone = place_zones_[i];
		if(ZoneBounds::intersect(placeZone,pickZone) || ZoneBounds::contains(pickZone,placeZone))
		{
			// intersection or containment found, unavailable place zone
			inactive_place_zones_.push_back(&place_zones_[i]);
		}
		else
		{
			placeZone.resetZone();
			active_place_zones_.push_back(&place_zones_[i]);
		}
	}
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

	// reference to active pick zone
	ZoneBounds &pickZone = pick_zones_[pick_zone_index_];

	// checking frame ids and transforming cloud if they are different
	tf::TransformListener tfListener;
	tf::StampedTransform clusterTransform;// = tf::StampedTransform( tf::Transform::getIdentity());
	Eigen::Affine3d tfEigen;
	if(pickZone.FrameId.compare(cluster.header.frame_id) != 0)
	{
		try
		{
			tfListener.lookupTransform(pickZone.FrameId,cluster.header.frame_id,ros::Time::now(),clusterTransform);
			tf::TransformTFToEigen(clusterTransform,tfEigen);
			pcl::transformPointCloud(cloud,cloud,Eigen::Affine3f(tfEigen));
			ROS_INFO_STREAM(ros::this_node::getName()<<"/ZoneSelection"<<": transformed cluster to frame: "<<pickZone.FrameId);
		}
		catch(tf::LookupException &e)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<"/ZoneSelection"<<": lookup exception thrown, not transforming to frame");
		}
	}

	// finding centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud,centroid);

	// checking if centroid of cloud is in bounds of pick zone
	pcl::PointXYZ clusterCentroid;
	clusterCentroid.x = centroid[0];
	clusterCentroid.y = centroid[1];


	if(((pickZone.XMin > clusterCentroid.x) || (pickZone.XMax < clusterCentroid.x)) ||
			((pickZone.YMin > clusterCentroid.y) || (pickZone.YMax < clusterCentroid.y)))
	{
		return false;
	}

	return true;
}

void PickPlaceZoneSelector::addObstacleClusters(std::vector<sensor_msgs::PointCloud> &clusters)
{
	for(std::vector<sensor_msgs::PointCloud>::iterator i = clusters.begin(); i != clusters.end(); i++)
	{
		addObstacleCluster(*i);
	}
}

void PickPlaceZoneSelector::addObstacleCluster(sensor_msgs::PointCloud &cluster)
{
	// converting cloud
	PclCloud cloud;
	sensor_msgs::PointCloud2 clusterMsg;
	sensor_msgs::convertPointCloudToPointCloud2(cluster,clusterMsg);
	pcl::fromROSMsg(clusterMsg,cloud);

	// reference to active pick zone
	ZoneBounds &pickZone = pick_zones_[pick_zone_index_];

	// checking frame ids and transforming cloud if they are different
	tf::TransformListener tfListener;
	tf::StampedTransform clusterTransform;// = tf::StampedTransform( tf::Transform::getIdentity());
	Eigen::Affine3d tfEigen;
	if(pickZone.FrameId.compare(cluster.header.frame_id) != 0)
	{
		try
		{
			tfListener.lookupTransform(pickZone.FrameId,cluster.header.frame_id,ros::Time::now(),clusterTransform);
			tf::TransformTFToEigen(clusterTransform,tfEigen);
			pcl::transformPointCloud(cloud,cloud,Eigen::Affine3f(tfEigen));
		}
		catch(tf::LookupException &e)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<"/ZoneSelection"<<": lookup exception thrown, not transforming cluster from '"<<
					cluster.header.frame_id<<"' to frame '"<< pickZone.FrameId<<"'");
		}
	}

	// finding centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud,centroid);

	// checking if centroid of cloud is in bounds of pick zone
	pcl::PointXYZ clusterCentroid;
	clusterCentroid.x = centroid[0];
	clusterCentroid.y = centroid[1];

	// finding size (overestimating by using largest size)
	pcl::PointXYZ min, max, size;
	pcl::getMinMax3D(cloud,min,max);
	size.x = std::abs(max.x - min.x);
	size.y = std::abs(max.y - min.y);
	size.z = std::abs(max.z - min.z);

	double maxSide = (size.x > size.y)? size.x : size.y;

	std::stringstream ss; ss<< obstacle_objects_.size();
	PlaceZone obstacleZone = PlaceZone(tf::Vector3(maxSide,maxSide,size.z),tf::Vector3(clusterCentroid.x,clusterCentroid.y,0.0f));
	obstacleZone.FrameId = pickZone.FrameId;
	obstacleZone.ZoneName = "obstacle" + ss.str();
	obstacle_objects_.push_back(obstacleZone);

}

bool  PickPlaceZoneSelector::generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	std::vector<PlaceZone* > nearbyZones;

	// searches in all available place zones for a location
	bool locationFound = false;
	for(std::size_t i = 0; i < active_place_zones_.size(); i++)
	{
		PlaceZone& placeZone = *active_place_zones_[i];
		if(placeZone.isIdInZone(next_obj_details_.Id))
		{
			ROS_WARN_STREAM("ZoneSelector: located id "<<next_obj_details_.Id <<" in zone "<<placeZone.ZoneName);

			// creating array with available place zones not including the current one
			nearbyZones.assign(active_place_zones_.begin(),active_place_zones_.end());
			nearbyZones.erase(nearbyZones.begin() + i);

			// inserting inactive place zone
			nearbyZones.insert(nearbyZones.end(),inactive_place_zones_.begin(),inactive_place_zones_.end());

			// inserting detected environment obstacles
			for(std::vector<PlaceZone>::iterator iter = obstacle_objects_.begin(); iter != obstacle_objects_.end(); iter++)
			{
				//PlaceZone &obstacleZone = *iter;
				nearbyZones.push_back(&(*iter));
			}

			placeZone.setNextObjectDetails(next_obj_details_);
			if(placeZone.generateNextLocationCandidates(placePoses,nearbyZones))
			{
				// location found, exit search
				locationFound = true;

				// passed all intersection test
				geometry_msgs::Point p = placePoses[0].pose.position;
				ROS_INFO_STREAM("ZoneSelector : Found available location at [ "<<p.x<<", "<<p.y<<", "<<p.z<<"]");

				break;
			}
		}

		ROS_WARN_STREAM("ZoneSelector: Did not located id in zone "<<placeZone.ZoneName);
	}

	// storing last added object
	if(locationFound)
	{

		last_object_added_ = next_obj_details_;
	}
	else
	{
		last_object_added_.Id = -1;
	}

	return locationFound;
}

void PickPlaceZoneSelector::removeLastObjectAdded()
{
	for(std::size_t i = 0; i < active_place_zones_.size(); i++)
	{
		PlaceZone& placeZone = *active_place_zones_[i];
		if(placeZone.isIdInZone(last_object_added_.Id))
		{
			ZoneBounds objectBounds = ZoneBounds(last_object_added_.Size,last_object_added_.Trans.getOrigin());
			if(ZoneBounds::contains(placeZone,objectBounds))
			{
				placeZone.removeLastObjectAdded();
				ROS_INFO_STREAM("Zone Selector: successfully removed last object added");
				return;
			}
		}
	}
}

void PickPlaceZoneSelector::getPickZoneMarker(visualization_msgs::Marker &marker)
{
	ZoneBounds &zone = pick_zones_[pick_zone_index_];
	zone.getMarker(marker);

	std_msgs::ColorRGBA color = marker_colors_.back();

	// computing transform
	tf::Vector3 center = zone.getCenter();
	tf::Quaternion q = tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),0.0f);
	tf::Transform zoneTf = tf::Transform(q,center);
	tf::poseTFToMsg(zoneTf,marker.pose);

	// filling additional fields
	marker.color = color;
	marker.ns = "pick_zone";
	marker.header.stamp = ros::Time(0);
	marker.header.frame_id = zone.FrameId;
}

void PickPlaceZoneSelector::getPickZoneTextMarker(visualization_msgs::Marker &marker)
{
	ZoneBounds &zone = pick_zones_[pick_zone_index_];
	zone.getTextMarker(marker);

	// computing transform
	tf::Vector3 center = zone.getCenter();
	tf::Quaternion q = tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),0.0f);
	tf::Transform zoneTf = tf::Transform(q,center);
	tf::poseTFToMsg(zoneTf,marker.pose);
	marker.pose.position.z = PICK_ZONE_HEIGHT*0.5f;

	// filling additional fields
	marker.ns = "pick_zone_name";
	marker.header.stamp = ros::Time(0);
	marker.header.frame_id = zone.FrameId;
}

void PickPlaceZoneSelector::getActivePlaceZonesMarkers(visualization_msgs::MarkerArray &markers)
{

	for(unsigned int i = 0; i < active_place_zones_.size(); i++)
	{
		PlaceZone &placeZone = *active_place_zones_[i];
		visualization_msgs::Marker marker;
		placeZone.getMarker(marker);
		marker.color = marker_colors_[((i > (marker_colors_.size()-1))? (i % marker_colors_.size()):(i))];
		marker.header.stamp = ros::Time(0);
		marker.header.frame_id = placeZone.FrameId;
		marker.pose = placeZone.getZoneCenterPose();
		marker.id= i;
		marker.ns = "place_zones";

		markers.markers.push_back(marker);
	}
}

void PickPlaceZoneSelector::getAllActiveZonesMarkers(visualization_msgs::MarkerArray &markers)
{
	std::string markerNs = "active_zones";
	for(unsigned int i = 0; i < active_place_zones_.size(); i++)
	{
		PlaceZone &placeZone = *active_place_zones_[i];
		visualization_msgs::Marker marker;
		placeZone.getMarker(marker);
		marker.color = marker_colors_[((i%2 == 0) ? (i%marker_colors_.size()): marker_colors_.size() - (i%marker_colors_.size()) )];
		//marker.color = marker_colors_[i];
		marker.header.frame_id = placeZone.FrameId;
		marker.header.stamp = ros::Time(0);
		marker.pose = placeZone.getZoneCenterPose();
		marker.id= i;
		marker.ns = markerNs;

		markers.markers.push_back(marker);
	}

	visualization_msgs::Marker pickZoneMarker;
	getPickZoneMarker(pickZoneMarker);
	//pickZoneMarker.ns = markerNs;
	pickZoneMarker.id = active_place_zones_.size();
	markers.markers.push_back(pickZoneMarker);
}

void PickPlaceZoneSelector::getAllActiveZonesTextMarkers(visualization_msgs::MarkerArray &markers)
{
	std::string markerNs = "zone_names";
	for(unsigned int i = 0; i < active_place_zones_.size(); i++)
	{
		PlaceZone &placeZone = *active_place_zones_[i];
		visualization_msgs::Marker marker;
		placeZone.getTextMarker(marker);
		marker.header.frame_id = placeZone.FrameId;
		marker.header.stamp = ros::Time(0);
		marker.pose = placeZone.getZoneCenterPose();
		marker.pose.position.z = PLACE_ZONE_HEIGHT*0.5f;
		marker.id= i;
		marker.ns = markerNs;

		markers.markers.push_back(marker);
	}

	visualization_msgs::Marker pickZoneMarker;
	getPickZoneTextMarker(pickZoneMarker);
	//pickZoneMarker.ns = markerNs;
	pickZoneMarker.id = active_place_zones_.size();
	markers.markers.push_back(pickZoneMarker);
}

void PickPlaceZoneSelector::getAllActiveZonesCombinedMarkers(visualization_msgs::MarkerArray &markers)
{
	getAllActiveZonesMarkers(markers);
	getAllActiveZonesTextMarkers(markers);
}

void PickPlaceZoneSelector::getAllObjectsMarkers(visualization_msgs::MarkerArray &markers)
{
	for(std::size_t i = 0 ; i < active_place_zones_.size(); i++)
	{
		PlaceZone &zone = *active_place_zones_[i];
		zone.getObjectsInZoneMarkers(markers,"place_zones");
	}

	visualization_msgs::Marker obstacleMarker;
	obstacleMarker.ns = "obstacles";
	obstacleMarker.header.frame_id = getPickZones()[0].FrameId;
	for(std::size_t i = 0; i < obstacle_objects_.size(); i++)
	{
		PlaceZone &obstacle = obstacle_objects_[i];
		obstacleMarker.id = i;
		obstacle.getMarker(obstacleMarker);
		obstacleMarker.scale.z = obstacle.getSize().z();
		obstacleMarker.pose = obstacle.getZoneCenterPose();
		obstacleMarker.color = marker_colors_[marker_colors_.size() - 2];
		obstacleMarker.color.a = 0.8f;
		obstacleMarker.pose.position.z = obstacleMarker.pose.position.z + obstacleMarker.scale.z;
		markers.markers.push_back(obstacleMarker);
	}
}

void PickPlaceZoneSelector::PlaceZone::resetZone()
{
	objects_in_zone_.clear();

	// computing grid mode
	tf::Vector3 zoneSize = getSize();
	grid_x_size_ = std::floor(zoneSize.x()/MinObjectSpacing);
	grid_y_size_ = std::floor(zoneSize.y()/MinObjectSpacing);

	ROS_INFO_STREAM("ZoneSelector: Grid size set to: X "<<grid_x_size_<<" x Y "<<grid_y_size_);
	grid_x_size_ = (grid_x_size_ > 0)?grid_x_size_:1;
	grid_y_size_ = (grid_y_size_ > 0)?grid_y_size_:1;
}

void PickPlaceZoneSelector::PlaceZone::setNextObjectDetails(const PickPlaceZoneSelector::ObjectDetails &objDetails)
{
	next_object_details_ = objDetails;
}

bool PickPlaceZoneSelector::PlaceZone::isIdInZone(int i)
{
	return std::find(Ids.begin(),Ids.end(),i) != Ids.end();
}

bool PickPlaceZoneSelector::PlaceZone::generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{
	bool success = false;

	switch(NextLocationGenMode)
	{
	case PickPlaceZoneSelector::PlaceZone::RANDOM:

		success = generateNextPlacePoseInRandomizedMode(placePoses,otherZones);
		break;

	case PickPlaceZoneSelector::PlaceZone::ZIGZAG_ALONG_X:

		success = generateNextPlacePoseInZigZagXMode(placePoses,otherZones);
		break;

	case PickPlaceZoneSelector::PlaceZone::ZIGZAG_ALONG_Y:
		success = generateNextPlacePoseInZigZagYMode(placePoses,otherZones);
		break;

	case PickPlaceZoneSelector::PlaceZone::CIRCLE:
		success = generateNextPlacePoseInCircle(placePoses,otherZones);
		break;

	case PickPlaceZoneSelector::PlaceZone::GRID_ALONG_X:
		success = generateNextPlacePoseInGridXWise(placePoses,otherZones);
		break;

	case PickPlaceZoneSelector::PlaceZone::GRID_ALONG_Y:
		success = generateNextPlacePoseInGridYWise(placePoses,otherZones);
		break;

	default:
		success = generateNextPlacePoseInGridXWise(placePoses,otherZones);
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
	tf::Vector3 pos = getCenter();
	tf::poseTFToMsg(tf::Transform(rot,pos),pose);
	return pose;
}

bool PickPlaceZoneSelector::PlaceZone::checkOverlaps(ZoneBounds &nextObjBounds,std::vector<PlaceZone* > zones)
{
	typedef std::vector<PlaceZone* >::iterator Iter;
	for(Iter i = zones.begin();i != zones.end(); i++)
	{
		PlaceZone zone = **i;
//		if(!ZoneBounds::intersect(zone,nextObjBounds) && !ZoneBounds::contains(zone,nextObjBounds))
//		{
//			continue;
//		}

		if(ZoneBounds::intersect(zone,nextObjBounds) || ZoneBounds::contains(zone,nextObjBounds))
		{
			return true;
		}

		std::vector<ObjectDetails> &objInZone = (*i)->objects_in_zone_;
		for(std::size_t j = 0; j < objInZone.size(); j++)
		{
			ZoneBounds objBounds = ZoneBounds(objInZone[j].Size,objInZone[j].Trans.getOrigin());
			if(ZoneBounds::intersect(nextObjBounds,objBounds))
			{
				return true;
			}
		}
	}

	return false;
}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInRandomizedMode(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{
	std::vector<ObjectDetails> object_array(objects_in_zone_);
	ObjectDetails seed_search_object_ = ObjectDetails();
	tf::Transform lastTf;
	tf::Transform nextTf;
	ZoneBounds nextObjectBounds;
	tf::Vector3 placeZoneCenter;
	bool foundNewPlaceLocation = false;

	// place zone details
	placeZoneCenter = getCenter();

	// next object details
	nextObjectBounds = ZoneBounds(next_object_details_.Size,next_object_details_.Trans.getOrigin());

	// initializing candidate location
	if(object_array.size() == 0)
	{
		tf::Quaternion rot = tf::Quaternion(Axis,0.0f);
		// the z component equals the height of the object plus the requested release distance

		tf::Vector3 pos = tf::Vector3(placeZoneCenter.x(),
				placeZoneCenter.y(),this->next_object_details_.Size.z() + ReleaseDistanceFromTable);
		nextTf = tf::Transform(rot,pos);

		// updating next object bounds to new candidate location
		nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

		// checking if located inside place region
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			ROS_INFO_STREAM("ZoneSelector/"<< ZoneName <<": First object with id: "<<
					this->next_object_details_.Id<<" too large for zone, exiting");
			return false;
		}

		// checking for intersections with obstacles
		ROS_INFO_STREAM("ZoneSelector/"<< ZoneName <<": Checking collision with other zones and obstacles");
		if( checkOverlaps(nextObjectBounds,otherZones))
		{
			// in collision with at least one obstacle, using last obstacle as seed
			ROS_WARN_STREAM("ZoneSelector/"<< ZoneName <<": Collision found, using last object as seed");

			seed_search_object_ = ObjectDetails(tf::Transform(tf::Quaternion::getIdentity()
			,otherZones.back()->getCenter()),otherZones.back()->getSize());
			lastTf = nextTf;

			ROS_WARN_STREAM("ZoneSelector/"<< ZoneName <<": New seed object set");
		}
		else
		{
			foundNewPlaceLocation = true;
			ROS_INFO_STREAM("ZoneSelector/"<< ZoneName <<": First object with id: "<< this->next_object_details_.Id
					<<", using center of place zone at x: "<<pos.x()<<", y: "<<pos.y()<<", z: "<<pos.z());
		}
	}
	else
	{
		seed_search_object_ = object_array.back();
		lastTf = seed_search_object_.Trans;

	}

	// initializing new  location search variables
	ZoneBounds lastObjectBounds = ZoneBounds(seed_search_object_.Size,seed_search_object_.Trans.getOrigin());
	double radialDistance = nextObjectBounds.getBoundingRadius() + lastObjectBounds.getBoundingRadius();
	int distanceSegments = 20; // number of possible values between min and max object spacing
	int angleSegments = 8; // number of possible values between 0 and 2pi
	int randVal;
	double distance; // number between Max and Min Object spacing parameters.  Should be larger than radial distance (meters)
	double angle,angleMin = 0,angleMax = 2*M_PI; // radians
	double ratio;

	// creating new location relative to the last object
	int maxIterations = 200;
	int iter = 0;

	ROS_WARN_STREAM("ZoneSelector/"<< ZoneName <<": Starting search");
	while(!foundNewPlaceLocation && (iter++ < maxIterations))
	{

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
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			if(object_array.size()>1)
			{
				// updating variables if using another object as seed
				ROS_WARN_STREAM("ZoneSelector/"<< ZoneName <<": Updating seed object");
				object_array.pop_back();
				seed_search_object_ = object_array.back();
				lastTf = seed_search_object_.Trans;
				lastObjectBounds = ZoneBounds(seed_search_object_.Size,seed_search_object_.Trans.getOrigin());
				radialDistance = nextObjectBounds.getBoundingRadius() + lastObjectBounds.getBoundingRadius();
				ROS_WARN_STREAM("ZoneSelector/"<< ZoneName <<": Seed object updated");
			}
			continue;
		}

		// checking for overlaps against objects already in place region
		bool overlapFound = false;
		typedef std::vector<ObjectDetails>::iterator IterType;
		for(IterType it = objects_in_zone_.begin();it != objects_in_zone_.end(); it++)
		{
			ZoneBounds objInZoneBounds(it->Size,it->Trans.getOrigin());
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

		// checking for overlaps against object in other zones
		overlapFound = checkOverlaps(nextObjectBounds,otherZones);

		if(overlapFound)
		{
			continue; // try again
		}

		double distFromCenter = (getCenter() - nextTf.getOrigin()).length();
		ROS_INFO_STREAM(ros::this_node::getName()<<"/ZoneSelection"<<": Found available position at a distance of "<< distFromCenter
				<<" from the center after "<<iter<<" iterations");

		// adjusting place point to object height
		nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);
		foundNewPlaceLocation = true;
		break;
	}

	if(!foundNewPlaceLocation)
	{
		ROS_ERROR_STREAM("ZoneSelector/"<<ZoneName<<": Did not find location after "
				<<maxIterations<<" iterations, exiting");
		return false;
	}

	// generating candidate poses from next location found
	createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

	// storing object
	next_object_details_.Trans = nextTf;
	objects_in_zone_.push_back(next_object_details_);

	return foundNewPlaceLocation;
}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInZigZagXMode(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{
	// next object details
	ZoneBounds nextObjectBounds;;
	int nextIndex = (int)objects_in_zone_.size();

	// will use next object id (even or odd) to determine its location
	tf::Transform nextTf = tf::Transform::getIdentity();

	// search parameters
	const int maxIterations = 200;
	int counter = 0;
	double xCoor;
	double yCoor;
	bool overlapFound = false;
	while(counter < maxIterations)
	{
		/* will use evenness of next object index to compute a new location relative to the top left corner of the place zone.
		 * Odds go to the top and evens at the bottom (top view of table)
		 */
		xCoor = (this->XMin + MinObjectSpacing/2.0f) + ((int)std::ceil((double)nextIndex/2.0f) - 1)*MinObjectSpacing;
		yCoor = ((nextIndex%2) == 0) ? (this->YMin + MinObjectSpacing/2.0f) : (this->YMax - MinObjectSpacing/2.0f);

		// incrementing counter
		counter++;

		// computing next candidate transform
		nextTf.setOrigin(tf::Vector3(xCoor,yCoor,0.0f));

		// updating next object bounds
		nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

		// checking if it is within place zone
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			overlapFound = true;
			break;// exit no more space
		}

		// checking if overlaps with objects in place zone
		typedef std::vector<ObjectDetails>::iterator ConstIter;
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
			nextIndex++;
			continue;
		}

		// checking for overlaps against object in other zones
		overlapFound = checkOverlaps(nextObjectBounds,otherZones);
		if(overlapFound)
		{
			nextIndex++;
			continue;
		}
		else
		{
			break;
		}

	}
	if(overlapFound)
	{
		ROS_ERROR_STREAM("ZoneSelector/"<<ZoneName<<": Did not find location after "
				<<maxIterations<<" iterations, exiting");
	}
	else
	{
		// adjusting place point to object height
		nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);

		// generating candidate poses from next location found
		createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

		// storing object
		next_object_details_.Trans = nextTf;
		objects_in_zone_.push_back(next_object_details_);
	}

	return !overlapFound;

}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInZigZagYMode(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{

	// next object details
	ZoneBounds nextObjectBounds;;
	int nextIndex = (int)objects_in_zone_.size();

	// will use next object id (even or odd) to determine its location
	tf::Transform nextTf = tf::Transform::getIdentity();

	// search parameters
	const int maxIterations = 200;
	int counter = 0;
	double xCoor;
	double yCoor;
	bool overlapFound = false;
	while(counter < maxIterations)
	{
		/* will use evenness of next object index to compute a new location relative to the top left corner of the place zone.
		 * Odds go to the top and evens at the bottom (top view of table)
		 */
		yCoor = (this->YMax - MinObjectSpacing/2.0f) - ((int)std::ceil((double)nextIndex/2.0f) - 1)*MinObjectSpacing;
		xCoor = ((nextIndex%2) == 0) ? (this->XMin + MinObjectSpacing/2.0f) : (this->XMax - MinObjectSpacing/2.0f);

		// incrementing counter
		counter++;

		// computing next candidate transform
		nextTf.setOrigin(tf::Vector3(xCoor,yCoor,0.0f));

		// updating next object bounds
		nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

		// checking if it is within place zone
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			overlapFound = true;
			break;// exit no more space
		}

		// checking if overlaps with objects in place zone
		typedef std::vector<ObjectDetails>::iterator ConstIter;
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
			nextIndex++;
			continue;
		}

		// checking for overlaps against object in other zones
		overlapFound = checkOverlaps(nextObjectBounds,otherZones);
		if(overlapFound)
		{
			nextIndex++;
			continue;
		}
		else
		{
			break;
		}

	}
	if(overlapFound)
	{
		ROS_ERROR_STREAM("ZoneSelector/"<<ZoneName<<": Did not find location after "
				<<maxIterations<<" iterations, exiting");
	}
	else
	{
		// adjusting place point to object height
		nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);

		// generating candidate poses from next location found
		createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

		// storing object
		next_object_details_.Trans = nextTf;
		objects_in_zone_.push_back(next_object_details_);
	}

	return !overlapFound;

}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInGridXWise(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{
	// next object details
	ZoneBounds nextObjectBounds;;
	int nextIndex = (int)objects_in_zone_.size();

	// will use next object id (even or odd) to determine its location
	tf::Transform nextTf = tf::Transform::getIdentity();

	// search parameters
	const int maxIterations = 50;
	int counter = 0;
	double xCoor;
	double yCoor;
	bool overlapFound = false;
	while(counter < maxIterations)
	{
		overlapFound = false;
		/* will use evenness of next object index to compute a new location relative to the top left corner of the place zone.
		 * Odds go to the top and evens at the bottom (top view of table)
		 */
//		xCoor = (this->XMin + MinObjectSpacing/2.0f) + ((nextIndex - 1)%grid_x_size_)*MinObjectSpacing;
//		yCoor = (this->YMax - MinObjectSpacing/2.0f) - ((int)std::ceil((double)nextIndex/((double)grid_x_size_)) - 1)*MinObjectSpacing;
		xCoor = (this->XMin + MinObjectSpacing/2.0f) + ((nextIndex)%grid_x_size_)*MinObjectSpacing;
		yCoor = (this->YMax - MinObjectSpacing/2.0f) - ((int)std::floor((double)nextIndex/((double)grid_x_size_)))*MinObjectSpacing;
		// incrementing counter
		counter++;

		// computing next candidate transform
		nextTf.setOrigin(tf::Vector3(xCoor,yCoor,0.0f));

		// updating next object bounds
		nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

		// checking if it is within place zone
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			overlapFound = true;
			ROS_ERROR_STREAM(ZoneName<<": Object falls outside place zone, exiting");
			return false;// exit no more space
		}

		// checking if overlaps with objects in place zone
		typedef std::vector<ObjectDetails>::iterator ConstIter;
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
			nextIndex++;
			continue;
		}

		// checking for overlaps against object in other zones
		overlapFound = checkOverlaps(nextObjectBounds,otherZones);
		if(overlapFound)
		{
			nextIndex++;
			continue;
		}
		else
		{
			break;
		}

	}
	if(overlapFound)
	{
		ROS_ERROR_STREAM("ZoneSelector/"<<ZoneName<<": Did not find location after "
				<<maxIterations<<" iterations, exiting");
	}
	else
	{
		// adjusting place point to object height
		nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);

		// generating candidate poses from next location found
		createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

		// storing object
		next_object_details_.Trans = nextTf;
		objects_in_zone_.push_back(next_object_details_);
	}

	return !overlapFound;
}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInGridYWise(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{
	// next object details
	ZoneBounds nextObjectBounds;;
	int nextIndex = (int)objects_in_zone_.size();

	// will use next object id (even or odd) to determine its location
	tf::Transform nextTf = tf::Transform::getIdentity();

	// search parameters
	const int maxIterations = 50;
	int counter = 0;
	double xCoor;
	double yCoor;
	bool overlapFound = false;
	while(counter < maxIterations)
	{
		overlapFound = false;
		/* will use evenness of next object index to compute a new location relative to the top left corner of the place zone.
		 * Odds go to the top and evens at the bottom (top view of table)
		 */
//		yCoor = (this->YMax - MinObjectSpacing/2.0f) - ((nextIndex - 1)%grid_y_size_)*MinObjectSpacing;
//		xCoor = (this->XMin + MinObjectSpacing/2.0f) + ((int)std::ceil((double)nextIndex/((double)grid_y_size_)) - 1)*MinObjectSpacing;
		yCoor = (this->YMax - MinObjectSpacing/2.0f) - ((nextIndex)%grid_y_size_)*MinObjectSpacing;
		xCoor = (this->XMin + MinObjectSpacing/2.0f) + ((int)std::floor((double)nextIndex/((double)grid_y_size_)))*MinObjectSpacing;

		// incrementing counter
		counter++;

		// computing next candidate transform
		nextTf.setOrigin(tf::Vector3(xCoor,yCoor,0.0f));

		// updating next object bounds
		nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

		// checking if it is within place zone
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			overlapFound = true;
			ROS_ERROR_STREAM(ZoneName<<": Object falls outside place zone, exiting");
			return false;// exit no more space
		}

		// checking if overlaps with objects in place zone
		typedef std::vector<ObjectDetails>::iterator ConstIter;
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
			nextIndex++;
			continue;
		}

		// checking for overlaps against object in other zones
		overlapFound = checkOverlaps(nextObjectBounds,otherZones);
		if(overlapFound)
		{
			nextIndex++;
			continue;
		}
		else
		{
			break;
		}

	}
	if(overlapFound)
	{
		ROS_ERROR_STREAM("ZoneSelector/"<<ZoneName<<": Did not find location after "
				<<maxIterations<<" iterations, exiting");
	}
	else
	{
		// adjusting place point to object height
		nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);

		// generating candidate poses from next location found
		createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

		// storing object
		next_object_details_.Trans = nextTf;
		objects_in_zone_.push_back(next_object_details_);
	}

	return !overlapFound;
}

bool PickPlaceZoneSelector::PlaceZone::generateNextPlacePoseInCircle(std::vector<geometry_msgs::PoseStamped> &placePoses,
		std::vector<PlaceZone* > &otherZones)
{
	// persistent variables
	static std::size_t layerCount = 0;
	static std::size_t nextLayerCount = 0;
	const static int elementsInFirstLayer = 6;
	const static int maxIterations = 50;
	const static double deltaTheta = 2.0f * M_PI/(double)elementsInFirstLayer;

	// initializing persistent variables
	if(objects_in_zone_.size() == 0)
	{
		layerCount = 0;
		nextLayerCount = 1;
	}
	else
	{
		if(nextLayerCount == objects_in_zone_.size())
		{
			layerCount++;
			nextLayerCount = nextLayerCount + elementsInFirstLayer * std::pow(2.0f,layerCount - 1);
		}
	}

	// next object details
	ZoneBounds nextObjectBounds;;
	int nextIndex = (int)objects_in_zone_.size();

	// will use next object id (even or odd) to determine its location
	tf::Transform nextTf = tf::Transform::getIdentity();

	// current search variables
	int counter = 0;
	double xCoor;
	double yCoor;
	double radius;
	double theta;
	bool overlapFound = false;
	tf::Vector3 center = getCenter();
	while(counter < maxIterations)
	{
		overlapFound = false;

		radius = layerCount * MinObjectSpacing;
		theta = (layerCount == 0)? 0 : (nextLayerCount - nextIndex) * (deltaTheta/(double)std::pow(2.0f,layerCount - 1));
		xCoor = center.x() + radius * std::cos(theta);
		yCoor = center.y() + radius * std::sin(theta);
		//ROS_WARN_STREAM("Zone Selection: Circle mode factor: "<<factor<<", angle: "<<theta);


		// incrementing counter
		counter++;

		// computing next candidate transform
		nextTf.setOrigin(tf::Vector3(xCoor,yCoor,0.0f));

		// updating next object bounds
		nextObjectBounds = ZoneBounds(next_object_details_.Size,nextTf.getOrigin());

		// checking if it is within place zone
		if(!ZoneBounds::contains(*this,nextObjectBounds))
		{
			overlapFound = true;
			nextIndex++;
			continue;
			ROS_ERROR_STREAM("Zone Selection:  Object falls outside zone "<<ZoneName);
		}

		// checking if overlaps with objects in place zone
		typedef std::vector<ObjectDetails>::iterator ConstIter;
		for(ConstIter iter = objects_in_zone_.begin(); iter != objects_in_zone_.end(); iter++)
		{
			ZoneBounds objInZoneBounds(iter->Size,iter->Trans.getOrigin());
			if(ZoneBounds::boundingCirclesIntersect(nextObjectBounds,objInZoneBounds))
			{
				ROS_ERROR_STREAM("Zone Selection: bounding circles of object in zone "<<ZoneName<<" intersect\n\t"
						<<"New object radius: "<<nextObjectBounds.getBoundingRadius()<<"\n\t"
						<<"Object in zone radius: "<<objInZoneBounds.getBoundingRadius()<<"\n\t"
						<<"Layer Count: "<<layerCount<<"\n\t"
						<<"Object Count: "<<nextIndex);
				overlapFound = true;
				break;
			}
		}
		if(overlapFound)
		{
			nextIndex++;
			continue;
		}

		// checking for overlaps against object in other zones
		overlapFound = checkOverlaps(nextObjectBounds,otherZones);
		if(overlapFound)
		{
			nextIndex++;
			continue;
		}
		else
		{
			break;
		}

	}
	if(overlapFound)
	{
		ROS_ERROR_STREAM("ZoneSelector/"<<ZoneName<<": Did not find location after "
				<<maxIterations<<" iterations, exiting");
	}
	else
	{
		// adjusting place point to object height
		nextTf.getOrigin().setZ(next_object_details_.Size.z() + ReleaseDistanceFromTable);

		// generating candidate poses from next location found
		createPlaceCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

		// storing object
		next_object_details_.Trans = nextTf;
		objects_in_zone_.push_back(next_object_details_);
	}

	return !overlapFound;
}
