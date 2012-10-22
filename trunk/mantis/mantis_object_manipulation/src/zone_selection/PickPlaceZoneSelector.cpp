/*
 * PickPlaceZoneSelector.cpp
 *
 *  Created on: Oct 22, 2012
 *      Author: jnicho
 */

#include <mantis_object_manipulation/zone_selection/PickPlaceZoneSelector.h>

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
	int current_pick_zone_index = pick_zone_index;
	pick_zone_index = place_zone_index;
	place_zone_index = current_pick_zone_index;
	updateTabletopSegmentationBounds();
}

bool PickPlaceZoneSelector::isInPickZone(const std::vector<sensor_msgs::PointCloud> &clusters,std::vector<int> &inZone)
{
	return true;
}

bool PickPlaceZoneSelector::isInPickZone(const sensor_msgs::PointCloud &cluster)
{
	return true;
}

void PickPlaceZoneSelector::updateTabletopSegmentationBounds()
{
	ros::NodeHandle nh;

	ZoneBounds &pickZoneBounds = Zones[pick_zone_index];
	ros::param::set(TabletopSegNamespace + "/" + TabletopSegXminName,pickZoneBounds.XMin);
	ros::param::set(TabletopSegNamespace + "/" + TabletopSegXmaxName,pickZoneBounds.XMax);
	ros::param::set(TabletopSegNamespace + "/" + TabletopSegYminName,pickZoneBounds.YMin);
	ros::param::set(TabletopSegNamespace + "/" + TabletopSegYmaxName,pickZoneBounds.YMax);
}
