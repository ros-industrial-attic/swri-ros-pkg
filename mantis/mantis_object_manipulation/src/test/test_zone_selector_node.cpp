/*
 * test_zone_selector_node.cpp
 *
 *  Created on: Oct 23, 2012
 */

#include <mantis_object_manipulation/zone_selection/PickPlaceZoneSelector.h>
#include <boost/foreach.hpp>
#include <iostream>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_zone_selector_node");
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();

	PickPlaceZoneSelector zoneSelector;
	zoneSelector.fetchParameters(nodeName);

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

	const PickPlaceZoneSelector::PlaceZone &placeZone = zoneSelector.getPlaceZone();
	std::cout<<"\tAxis: "<<placeZone.Axis.x()<<", "<<placeZone.Axis.y()<<", "<<placeZone.Axis.z()<<"\n";
	std::cout<<"\tMinObjectSpacing: "<<placeZone.MinObjectSpacing<<"\n";
	std::cout<<"\tMaxObjectSpacing: "<<placeZone.MaxObjectSpacing<<"\n";
	std::cout<<"\tplace zone radius: "<<zoneSelector.getPlaceZoneRadius()<<"\n";

	std::cout<<"\nTesting place generation"<<"\n";

	// setting object size
	tf::Vector3 objectSize = tf::Vector3(0.04f,0.04f,0.04f);
	int numLocations = 10;

	zoneSelector.setGraspObjectSize(objectSize);
	tf::Vector3 placeCenter = zoneSelector.getPlaceZoneCenter();
	std::vector<geometry_msgs::PoseStamped> poses;
	counter = 1;
	std::cout<<"\nPlace Locations for zone with center: "<< placeCenter.x()<<", "<< placeCenter.y()<<", "<<placeCenter.z()<<"\n";
	std::cout<<"\nPlace Locations radius : "<< zoneSelector.getPlaceZoneRadius()<<"\n";
	while(zoneSelector.generateNextLocationCandidates(poses) && ros::ok())
	{
		geometry_msgs::Point &location = poses[0].pose.position;
		std::cout<<"\tposition "<<counter<<": "<<location.x<<", "<<location.y<<", "<<location.z<<"\n";

		poses.clear();
		counter++;
	}

	// swapping zones
	zoneSelector.swapPickPlaceZones();
	placeCenter = zoneSelector.getPlaceZoneCenter();
	counter = 1;
	std::cout<<"\n\nSwapt pick and place zones"<<"\n";
	std::cout<<"\nPlace Locations for zone with center: "<< placeCenter.x()<<", "<< placeCenter.y()<<", "<<placeCenter.z()<<"\n";
	std::cout<<"\nPlace Locations radius : "<< zoneSelector.getPlaceZoneRadius()<<"\n";
	while(zoneSelector.generateNextLocationCandidates(poses) && ros::ok())
	{
		geometry_msgs::Point &location = poses[0].pose.position;
		std::cout<<"\tposition "<<counter<<": "<<location.x<<", "<<location.y<<", "<<location.z<<"\n";

		poses.clear();
		counter++;
	}

}
