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
#include <boost/thread/mutex.hpp>
#include <iostream>

typedef std::vector<PickPlaceZoneSelector::PlaceZone> PlaceZoneArray;
typedef std::vector<PickPlaceZoneSelector::ZoneBounds> PickZoneArray;
typedef std::vector<PickPlaceZoneSelector::PlaceZone>::const_iterator PlaceZoneIter;
typedef std::vector<PickPlaceZoneSelector::ZoneBounds>::const_iterator PickZoneIter;


visualization_msgs::MarkerArray OBJECT_MARKER_ARRAY;
visualization_msgs::MarkerArray ZONE_MARKER_ARRAY;
visualization_msgs::Marker OBJECT_MARKER_MSG;
visualization_msgs::Marker ZONE_MARKER_MSG;
ros::Publisher OBJECT_MARKER_PUBLISHER;
ros::Publisher ZONE_MARKER_PUBLISHER;
boost::shared_ptr<tf::TransformBroadcaster> TF_BROADCASTER_PTR;
tf::StampedTransform WORLD_TRANSFORM;

// ros parameters
double DURATION_VALUE = 0.5f; // seconds
int NUM_OBJECTS_IN_ZONE = 10;
double TEST_OBJECT_RADIUS = 0.02f;
std::vector<int> IDS_USED = std::vector<int>();

// multithreading
boost::mutex MUTEX_OBJ;

void timerCallback(const ros::TimerEvent &evnt)
{
	{
		boost::mutex::scoped_lock lock(MUTEX_OBJ);
		if(!OBJECT_MARKER_ARRAY.markers.empty())
		{
			OBJECT_MARKER_PUBLISHER.publish(OBJECT_MARKER_ARRAY);
		}
	}

	{
		boost::mutex::scoped_lock lock(MUTEX_OBJ);
		if(!ZONE_MARKER_ARRAY.markers.empty())
		{
			ZONE_MARKER_PUBLISHER.publish(ZONE_MARKER_ARRAY);
		}
	}

	WORLD_TRANSFORM.stamp_ = ros::Time::now();
	TF_BROADCASTER_PTR->sendTransform(WORLD_TRANSFORM);
}

void removeZoneMarkers()
{
	boost::mutex::scoped_lock lock(MUTEX_OBJ);
	{
		for(unsigned int i = 0; i < ZONE_MARKER_ARRAY.markers.size(); i++)
		{
			visualization_msgs::Marker &m =  ZONE_MARKER_ARRAY.markers[i];
			m.action = visualization_msgs::Marker::DELETE;
		}
		ZONE_MARKER_PUBLISHER.publish(ZONE_MARKER_ARRAY);
		ZONE_MARKER_ARRAY.markers.clear();
	}
}

void clearObjectMarkers()
{
	boost::mutex::scoped_lock lock(MUTEX_OBJ);
	OBJECT_MARKER_ARRAY.markers.clear();
}

void addObjectMarker(const visualization_msgs::Marker objMarker)
{
	boost::mutex::scoped_lock lock(MUTEX_OBJ);
	OBJECT_MARKER_ARRAY.markers.push_back(objMarker);
}

void generatePosesInRequestedMode(PickPlaceZoneSelector &zoneSelector,PickPlaceZoneSelector::ObjectDetails &obj,int numLocations)
{
	std::vector<geometry_msgs::PoseStamped> poses;

	const int numIds = IDS_USED.size();
	for(int i = 0; i < numLocations && ros::ok(); i++)
	{

		obj.Id = IDS_USED[rand()%numIds];
		zoneSelector.setNextObjectDetails(obj);

		if(!zoneSelector.generateNextLocationCandidates(poses))
		{
			std::cout<<"Next location could not be found\n";
			clearObjectMarkers();
			break;
		}
		else
		{
			geometry_msgs::Point &location = poses[0].pose.position;
			std::cout<<"position for the "<<i<< "th object id "<<obj.Id<<" found at: "<<location.x<<", "<<location.y<<", "<<location.z<<"\n";

			// adding to marker array
			OBJECT_MARKER_MSG.id = i;
			OBJECT_MARKER_MSG.pose = poses[0].pose;
			//OBJECT_MARKER_ARRAY.markers.push_back(OBJECT_MARKER_MSG);
			addObjectMarker(OBJECT_MARKER_MSG);

		}
		poses.clear();

		//ros::spin();
		ros::Duration(DURATION_VALUE).sleep();
	}

	clearObjectMarkers();
}

void fetchParameters(std::string nameSpace)
{
	ros::param::param(nameSpace + "/num_objects_in_zone",NUM_OBJECTS_IN_ZONE,NUM_OBJECTS_IN_ZONE);
	ros::param::param(nameSpace + "/update_rate",DURATION_VALUE,DURATION_VALUE);
	ros::param::param(nameSpace + "/test_object_radius",TEST_OBJECT_RADIUS,TEST_OBJECT_RADIUS);

	if(!IDS_USED.empty())
	{
		return;
	}

	XmlRpc::XmlRpcValue array;
	ros::param::get(nameSpace + "/ids_used",array);
	if(array.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		for(int i = 0; i < array.size(); i++)
		{
			int id = static_cast<int>(array[i]);
			IDS_USED.push_back(id);
		}
		ROS_INFO_STREAM(ros::this_node::getName()<<": found "<<IDS_USED.size()<<" ids in array");
	}
	else
	{
		ROS_ERROR_STREAM(ros::this_node::getName()<<": ids array not found in parameters, exiting");
		ros::shutdown();
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_zone_selector_node");
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();
	ros::AsyncSpinner spinner(4);

	// initializing random generator seed
	srand(time(0));

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
	WORLD_TRANSFORM.child_frame_id_ = zoneSelector.getPickZones()[0].FrameId;

	// setting up marker msg
	OBJECT_MARKER_MSG.header.frame_id = zoneSelector.getPickZones()[0].FrameId;
	OBJECT_MARKER_MSG.header.stamp = ros::Time();
	OBJECT_MARKER_MSG.ns = nodeName;
	OBJECT_MARKER_MSG.type = visualization_msgs::Marker::SPHERE;
	OBJECT_MARKER_MSG.action = visualization_msgs::Marker::ADD;
	OBJECT_MARKER_MSG.scale.x = TEST_OBJECT_RADIUS;
	OBJECT_MARKER_MSG.scale.y = TEST_OBJECT_RADIUS;
	OBJECT_MARKER_MSG.scale.z = TEST_OBJECT_RADIUS;
	OBJECT_MARKER_MSG.color.a = 1.0;
	OBJECT_MARKER_MSG.color.r = 0.0;
	OBJECT_MARKER_MSG.color.g = 0.4f;
	OBJECT_MARKER_MSG.color.b = 0.8f;

	// setting up zone markers
	zoneSelector.getAllActiveZonesMarkers(ZONE_MARKER_ARRAY);

	// printing found place zones
	std::cout<<"There are "<<zoneSelector.getAllPlaceZones().size()<<" place zones\n";
	int counter = 1;
	//BOOST_FOREACH(const PickPlaceZoneSelector::PlaceZone zone,zoneSelector.getAllPlaceZones())
	const PlaceZoneArray &zones = zoneSelector.getAllPlaceZones();
	for(PlaceZoneIter i = zones.begin(); i != zones.end(); i++)
	{
		const PickPlaceZoneSelector::PlaceZone &zone = *i;
		std::cout<<"\tZone "<<counter<<"\n";
		std::cout<<"\t\tName: "<<zone.ZoneName<<"\n";
		std::cout<<"\t\tXmin: "<<zone.XMin<<"\n";
		std::cout<<"\t\tXmax: "<<zone.XMax<<"\n";
		std::cout<<"\t\tYmin: "<<zone.YMin<<"\n";
		std::cout<<"\t\tYMax: "<<zone.YMax<<"\n";
		counter++;
	}

	// printing found pick zones
	std::cout<<"There are "<< zoneSelector.getPickZones().size() <<" pick zones\n";
	counter = 1;
	//BOOST_FOREACH(const PickPlaceZoneSelector::ZoneBounds zone,zoneSelector.getPickZones())
	const PickZoneArray &pickZones = zoneSelector.getPickZones();
	for(PickZoneIter i = pickZones.begin(); i != pickZones.end(); i++)
	{
		const PickPlaceZoneSelector::ZoneBounds &zone = *i;
		std::cout<<"\tZone "<<counter<<"\n";
		std::cout<<"\t\tName: "<<zone.ZoneName<<"\n";
		std::cout<<"\t\tXmin: "<<zone.XMin<<"\n";
		std::cout<<"\t\tXmax: "<<zone.XMax<<"\n";
		std::cout<<"\t\tYmin: "<<zone.YMin<<"\n";
		std::cout<<"\t\tXmin: "<<zone.YMax<<"\n";
		counter++;
	}

	std::cout<<"\nTesting place generation"<<"\n";

	// object details
	tf::Vector3 objectSize = tf::Vector3(2.0f*TEST_OBJECT_RADIUS,2.0f*TEST_OBJECT_RADIUS,2.0f*TEST_OBJECT_RADIUS);
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

		// updating object
		objDetails.Size = tf::Vector3(2.0f*TEST_OBJECT_RADIUS,2.0f*TEST_OBJECT_RADIUS,2.0f*TEST_OBJECT_RADIUS);

		// updateing marker size
		OBJECT_MARKER_MSG.scale.x = TEST_OBJECT_RADIUS;
		OBJECT_MARKER_MSG.scale.y = TEST_OBJECT_RADIUS;
		OBJECT_MARKER_MSG.scale.z = TEST_OBJECT_RADIUS;

		std::cout<<"\nGenerating Poses :\n";
		generatePosesInRequestedMode(zoneSelector,objDetails,NUM_OBJECTS_IN_ZONE);

		// swapping zones
		ROS_INFO_STREAM(nodeName<<": Going to next zone");
		zoneSelector.goToNextPickZone();

		ROS_INFO_STREAM(nodeName<<": removing all zone markers");
		removeZoneMarkers();

		ROS_INFO_STREAM(nodeName<<": getting new zone markers");
		zoneSelector.getAllActiveZonesMarkers(ZONE_MARKER_ARRAY);

		ROS_INFO_STREAM(nodeName<<": End of cycle, going to next zone");
	}

	return 0;
}


