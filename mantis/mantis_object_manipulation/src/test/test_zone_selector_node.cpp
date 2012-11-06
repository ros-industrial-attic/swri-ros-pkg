/*
 * test_zone_selector_node.cpp
 *
 *  Created on: Oct 23, 2012
 */

#include <mantis_object_manipulation/zone_selection/PickPlaceZoneSelector.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

double DURATION_VALUE = 0.5f; // seconds
visualization_msgs::MarkerArray OBJECT_MARKER_ARRAY;
visualization_msgs::MarkerArray ZONE_MARKER_ARRAY;
visualization_msgs::Marker OBJECT_MARKER_MSG;
visualization_msgs::Marker ZONE_MARKER_MSG;
ros::Publisher OBJECT_MARKER_PUBLISHER;
ros::Publisher ZONE_MARKER_PUBLISHER;
boost::shared_ptr<tf::TransformBroadcaster> TF_BROADCASTER_PTR;
tf::StampedTransform WORLD_TRANSFORM;
int NUM_OBJECTS_IN_ZONE = 10;

void timerCallback(const ros::TimerEvent &evnt)
{
	if(!OBJECT_MARKER_ARRAY.markers.empty())
	{
		OBJECT_MARKER_PUBLISHER.publish(OBJECT_MARKER_ARRAY);
	}

	ZONE_MARKER_PUBLISHER.publish(ZONE_MARKER_ARRAY);

	WORLD_TRANSFORM.stamp_ = ros::Time::now();
	TF_BROADCASTER_PTR->sendTransform(WORLD_TRANSFORM);
}

void generatePosesInRandomMode(PickPlaceZoneSelector::PlaceZone& placeZone,const PickPlaceZoneSelector::ObjectDetails &obj,int numLocations)
{
	std::vector<geometry_msgs::PoseStamped> poses;

	for(int i = 0; i < numLocations && ros::ok(); i++)
	{
		placeZone.setNextObjectDetails(obj);

		if(!placeZone.generateNextLocationCandidates(poses))
		{
			std::cout<<"Next location could not be found\n";
			OBJECT_MARKER_ARRAY.markers.clear();
			break;
		}
		else
		{
			geometry_msgs::Point &location = poses[0].pose.position;
			std::cout<<"\tnext position found at "<<i<<": "<<location.x<<", "<<location.y<<", "<<location.z<<"\n";

			// adding to marker array
			OBJECT_MARKER_MSG.id = i;
			OBJECT_MARKER_MSG.pose = poses[0].pose;
			OBJECT_MARKER_ARRAY.markers.push_back(OBJECT_MARKER_MSG);

			poses.clear();
		}

		//ros::spin();
		ros::Duration(DURATION_VALUE).sleep();
	}

}

void generatePosesInZigZagMode(PickPlaceZoneSelector::PlaceZone& placeZone,PickPlaceZoneSelector::ObjectDetails &obj,int numLocations)
{
	std::vector<geometry_msgs::PoseStamped> poses;

	for(int i = 0; i < numLocations && ros::ok(); i++)
	{
		obj.Id = i + 1;
		placeZone.setNextObjectDetails(obj);

		if(!placeZone.generateNextLocationCandidates(poses))
		{
			std::cout<<"Next location could not be found\n";
			OBJECT_MARKER_ARRAY.markers.clear();
			break;
		}
		else
		{
			geometry_msgs::Point &location = poses[0].pose.position;
			std::cout<<"\tposition for object id "<<obj.Id<<" found at: "<<location.x<<", "<<location.y<<", "<<location.z<<"\n";

			// adding to marker array
			OBJECT_MARKER_MSG.id = i;
			OBJECT_MARKER_MSG.pose = poses[0].pose;
			OBJECT_MARKER_ARRAY.markers.push_back(OBJECT_MARKER_MSG);

			poses.clear();
		}

		//ros::spin();
		ros::Duration(DURATION_VALUE).sleep();
	}
}

void fetchParameters(std::string nameSpace)
{
	ros::param::param(nameSpace + "/num_objects_in_zone",NUM_OBJECTS_IN_ZONE,NUM_OBJECTS_IN_ZONE);
	ros::param::param(nameSpace + "/update_rate",DURATION_VALUE,DURATION_VALUE);
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_zone_selector_node");
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();
	ros::AsyncSpinner spinner(4);

	// initializing zone selector instance
	PickPlaceZoneSelector zoneSelector;
	zoneSelector.fetchParameters(nodeName);

	// setting up ros objects
	TF_BROADCASTER_PTR = boost::make_shared<tf::TransformBroadcaster>();
	OBJECT_MARKER_PUBLISHER = nh.advertise<visualization_msgs::MarkerArray>("placed_objects",0);
	ZONE_MARKER_PUBLISHER = nh.advertise<visualization_msgs::MarkerArray>("work_zones",0);
	ros::Timer timer = nh.createTimer(ros::Duration(DURATION_VALUE),&timerCallback,false);

	// setting up transform message
	WORLD_TRANSFORM = tf::StampedTransform(tf::Transform::getIdentity(),ros::Time::now(),"NO_PARENT","base_link");
	WORLD_TRANSFORM.child_frame_id_ = zoneSelector.getPlaceZone().FrameId;

	// setting up marker msg
	double sphereRadius = zoneSelector.getPlaceZone().MinObjectSpacing/2.0f;
	OBJECT_MARKER_MSG.header.frame_id = zoneSelector.getPlaceZone().FrameId;
	OBJECT_MARKER_MSG.header.stamp = ros::Time();
	OBJECT_MARKER_MSG.ns = nodeName;
	OBJECT_MARKER_MSG.type = visualization_msgs::Marker::SPHERE;
	OBJECT_MARKER_MSG.action = visualization_msgs::Marker::ADD;
	OBJECT_MARKER_MSG.scale.x = sphereRadius;
	OBJECT_MARKER_MSG.scale.y = sphereRadius;
	OBJECT_MARKER_MSG.scale.z = sphereRadius;
	OBJECT_MARKER_MSG.color.a = 1.0;
	OBJECT_MARKER_MSG.color.r = 0.0;
	OBJECT_MARKER_MSG.color.g = 0.4f;
	OBJECT_MARKER_MSG.color.b = 0.8f;

	// setting up zone markers
	ZONE_MARKER_MSG.header.stamp = ros::Time();
	ZONE_MARKER_MSG.ns = "zones";
	ZONE_MARKER_MSG.action = visualization_msgs::Marker::ADD;
	ZONE_MARKER_MSG.id = 0;
	zoneSelector.getPickZoneMarker(ZONE_MARKER_MSG);
	ZONE_MARKER_ARRAY.markers.push_back(ZONE_MARKER_MSG);

	ZONE_MARKER_MSG.id = 1;
	zoneSelector.getPlaceZoneMarker(ZONE_MARKER_MSG);
	ZONE_MARKER_ARRAY.markers.push_back(ZONE_MARKER_MSG);

	// printing zones
	std::cout<<"Zone details has "<<zoneSelector.Zones.size()<<" elements\n";
	int counter = 1;
	BOOST_FOREACH(PickPlaceZoneSelector::ZoneBounds zone,zoneSelector.Zones)
	{
		std::cout<<"\tZone "<<counter<<"\n";
		std::cout<<"\t\tName: "<<zone.ZoneName<<"\n";
		std::cout<<"\t\tXmin: "<<zone.XMin<<"\n";
		std::cout<<"\t\tXmax: "<<zone.XMax<<"\n";
		std::cout<<"\t\tYmin: "<<zone.YMin<<"\n";
		std::cout<<"\t\tXmin: "<<zone.YMax<<"\n";
		counter++;
	}

	PickPlaceZoneSelector::PlaceZone &placeZone = zoneSelector.getPlaceZone();
	std::cout<<"\tAxis: "<<placeZone.Axis.x()<<", "<<placeZone.Axis.y()<<", "<<placeZone.Axis.z()<<"\n";
	std::cout<<"\tMinObjectSpacing: "<<placeZone.MinObjectSpacing<<"\n";
	std::cout<<"\tMaxObjectSpacing: "<<placeZone.MaxObjectSpacing<<"\n";
	std::cout<<"\tplace zone radius: "<<placeZone.getZoneBounds().getBoundingRadius()<<"\n";

	std::cout<<"\nTesting place generation"<<"\n";

	// object details
	tf::Vector3 objectSize = tf::Vector3(0.04f,0.04f,0.04f);
	tf::Transform objectTf = tf::Transform::getIdentity();
	int id = 0;
	std::string tag = "test_object";
	PickPlaceZoneSelector::ObjectDetails objDetails(objectTf,objectSize,id,tag);
	//int numLocations = 6; // locations to search in current place zone before switching

	spinner.start();
	while(ros::ok())
	{
		// getting parameters
		fetchParameters(nodeName);

		std::cout<<"\nGenerating Poses in Random Mode at place location with center:\n";
		tf::Vector3 placeCenter = zoneSelector.getPlaceZone().getZoneBounds().getCenter();
		std::cout<<"\tx: "<< placeCenter.x()<<",y: "<< placeCenter.y()<<",z: "<<placeCenter.z()<<"\n";
		placeZone.GenerationMode = PickPlaceZoneSelector::PlaceZone::RANDOM;
		generatePosesInRandomMode(placeZone,objDetails,NUM_OBJECTS_IN_ZONE);

		// swapping zones
		zoneSelector.swapPickPlaceZones();
		std::cout<<"\n\nSwapt pick and place zones"<<"\n";

		// getting parameters
		fetchParameters(nodeName);

		// generating zigzag arrangement
		std::cout<<"\nGenerating Poses in Zigzag Mode at place location with center\n";
		placeCenter = zoneSelector.getPlaceZone().getZoneBounds().getCenter();
		std::cout<<"\tx: "<< placeCenter.x()<<",y: "<< placeCenter.y()<<",z: "<<placeCenter.z()<<"\n";
		placeZone.GenerationMode = PickPlaceZoneSelector::PlaceZone::DESIGNATED_ZIGZAG;
		generatePosesInZigZagMode(placeZone,objDetails,NUM_OBJECTS_IN_ZONE);

		// swapping zones
		zoneSelector.swapPickPlaceZones();
		std::cout<<"\n\nSwapt pick and place zones"<<"\n";

	}

	return 0;
}


