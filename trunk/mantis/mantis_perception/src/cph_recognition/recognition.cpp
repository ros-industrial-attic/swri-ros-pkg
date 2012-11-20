//mantis_object_recognition_node
//based on work done at UT Austin by Brian O'Neil
#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

#include <visualization_msgs/Marker.h>

#include <boost/filesystem.hpp>

#include "cph.h"
#include "mantis_perception/mantis_recognition.h"
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"

ros::ServiceClient cph_client;
ros::Publisher rec_pub;
ros::Publisher vis_pub;
ros::Publisher noise_pub;

bool rec_cb(mantis_perception::mantis_recognition::Request &main_request,
            mantis_perception::mantis_recognition::Response &main_response)
{  

  float pvct_1_x_offset = 0.004324;
  float pvct_1_y_offset = -0.008176;
  float pvct_1_z_offset = 0.01698;
  float plug_1_x_offset = 0.00988;
  float plug_1_y_offset = -0.010399;
  float plug_1_z_offset = 0.03052;
  float enc_1_x_offset = 0.0085645;
  float enc_1_y_offset = -0.0099451;
  float enc_1_z_offset = 0.03923;
  float pvc_elbow_1_x_offset = 0.004949;
  float pvc_elbow_1_y_offset = -0.006247;
  float pvc_elbow_1_z_offset = 0.0182536;
  float plug_pick_point_z = 0.048;
  float enc_pick_point_x = 0.027;
  float enc_pick_point_z = 0.051;
  float pvct_pick_point_z = 0.055;
  float pvc_elbow_pick_point_z = 0.030;

  ROS_INFO("Starting mantis recognition");
  ROS_INFO("Number of clusters received in request = %d", (int)main_request.clusters.size());

  //convert segmentation results from array of PointCloud to single PointCloud2
  sensor_msgs::PointCloud received_cluster;
  received_cluster=main_request.clusters.at(0);
  sensor_msgs::PointCloud2 cluster;
  sensor_msgs::convertPointCloudToPointCloud2(received_cluster, cluster);
  cluster.header.frame_id=main_request.table.pose.header.frame_id;
  cluster.header.stamp=main_request.table.pose.header.stamp;

  //Brian's recognition service
  nrg_object_recognition::recognition rec_srv;

  rec_srv.request.cluster = cluster;
  rec_srv.request.threshold = 1000;
      
  //Call recognition service
  cph_client.call(rec_srv);
  if (!cph_client.call(rec_srv))
  {
    ROS_ERROR("Call to cph recognition service failed");
  }
  float theta = rec_srv.response.pose.rotation*3.14159/180;//radians
  
  static tf::TransformBroadcaster obj_broadcaster;



////////////////////Assign response values/////////////////////////
  main_response.label = rec_srv.response.label;
  main_response.pose = rec_srv.response.pose;

  ROS_INFO_STREAM("Recgonized pose: \n x: " << rec_srv.response.pose.x << "\n y: "<<rec_srv.response.pose.y <<
		  "\n z: "<<rec_srv.response.pose.z << "\n theta: "<<rec_srv.response.pose.rotation);

  tf::Quaternion part_orientation;
  part_orientation.setValue(0, 0, sin(theta/2), cos(theta/2));
  tf::Vector3 part_origin;
  part_origin.setValue(rec_srv.response.pose.x, rec_srv.response.pose.y, rec_srv.response.pose.z);
  tf::Transform part_frame;
  part_frame.setOrigin(part_origin);
  part_frame.setRotation(part_orientation);
  ROS_INFO_STREAM("Pick point frame origin: X: "<<part_origin.x()<< " Y: "<<part_origin.y()<< " Z: "<<part_origin.z());
  tf::Point enc_pick_point;
  enc_pick_point.setValue(enc_pick_point_x, enc_pick_point_x, enc_pick_point_z);
  ROS_INFO_STREAM("Pick point for enclosure: X: "<<enc_pick_point.x()<< " Y: "<<enc_pick_point.y()<< " Z: "<<enc_pick_point.z());
  tf::Point enc_pick = part_frame * enc_pick_point;
  ROS_INFO_STREAM("Pick point in part frame: X: "<<enc_pick.x()<< " Y: "<<enc_pick.y()<< " Z: "<<enc_pick.z());


  tf::Quaternion rot_part;
  rot_part.setValue(sin(3.14159/2), 0, 0, cos(3.14159/2));

  tf::Quaternion rotated_part = part_orientation * rot_part;
  //tf::Transform rotated_frame =rot_part * part_frame ;

  geometry_msgs::Quaternion pick_frame;
  tf::quaternionTFToMsg(rotated_part, pick_frame);

  geometry_msgs::PoseStamped pick_pose;

  pick_pose.header.stamp=main_request.table.pose.header.stamp;
  pick_pose.header.frame_id=main_request.table.pose.header.frame_id;
  pick_pose.pose.orientation.x = pick_frame.x;
  pick_pose.pose.orientation.y = pick_frame.y;
  pick_pose.pose.orientation.z = pick_frame.z;
  pick_pose.pose.orientation.w = pick_frame.w;

  //Assign id and marker based on label
  visualization_msgs::Marker mesh_marker;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  //mesh_marker.type = visualization_msgs::Marker::SPHERE;
  mesh_marker.action = visualization_msgs::Marker::ADD;
  mesh_marker.lifetime = ros::Duration();
  mesh_marker.header.frame_id = pick_pose.header.frame_id;
  mesh_marker.header.stamp= pick_pose.header.stamp;//ros::Time();
  mesh_marker.scale.x = 1;
  mesh_marker.scale.y = 1;
  mesh_marker.scale.z = 1;
  mesh_marker.pose.orientation.x = part_orientation.x();
  mesh_marker.pose.orientation.y = part_orientation.y();
  mesh_marker.pose.orientation.z = part_orientation.z();
  mesh_marker.pose.orientation.w = part_orientation.w();
  mesh_marker.color.a = 1.0;
  mesh_marker.color.r = 0.0;
  mesh_marker.color.g = 1.0;
  mesh_marker.color.b = 0.0;
  std::size_t found;
  std::string label = main_response.label;
  found=label.find_last_of("/");

  //Depending on label and angle, assign marker mesh, model_id, position for mesh and for part frame
  ROS_WARN_STREAM("Object labeled as "<< label.substr(found+1));
  if(label.substr(found+1)=="enclosure_1" || label.substr(found+1)=="enclosuref")
  {
    main_response.model_id=1;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/elec_enclosure.STL";
    pick_pose.pose.position.x = enc_pick.x();// - (enc_1_x_offset);//+enc_pick_point_x;
    pick_pose.pose.position.y = enc_pick.y();//rec_srv.response.pose.y - (enc_1_y_offset);
    pick_pose.pose.position.z = rec_srv.response.pose.z - (enc_1_z_offset)+enc_pick_point_z;
    mesh_marker.pose.position.x=rec_srv.response.pose.x - (enc_1_x_offset);
    mesh_marker.pose.position.y=rec_srv.response.pose.y - (enc_1_y_offset);
    mesh_marker.pose.position.z=rec_srv.response.pose.z - (enc_1_z_offset);
    /*obj_broadcaster.sendTransform(tf::StampedTransform(
  	        tf::Transform(tf::Quaternion(pick_pose.pose.orientation.x, pick_pose.pose.orientation.y, pick_pose.pose.orientation.z, pick_pose.pose.orientation.w),
  	        tf::Vector3(pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z)),
  	      main_request.table.pose.header.stamp,"/base_link", "/object_training_frame"));*/
  }
  else if (label.substr(found+1)=="coupling" || label.substr(found+1)=="couplingf")
  {
    main_response.model_id=2;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/rubber_coupler_clamps.STL";
  }
  else if (label.substr(found+1)=="pvc_t_1" || label.substr(found+1)=="pvct")
  {
    main_response.model_id=3;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/pvc_t.STL";
    pick_pose.pose.position.x = rec_srv.response.pose.x - (pvct_1_x_offset);
    pick_pose.pose.position.y = rec_srv.response.pose.y - (pvct_1_y_offset);
    pick_pose.pose.position.z = rec_srv.response.pose.z - (pvct_1_z_offset)+pvct_pick_point_z/2;
    mesh_marker.pose.position.x=rec_srv.response.pose.x - (pvct_1_x_offset);
    mesh_marker.pose.position.y=rec_srv.response.pose.y - (pvct_1_y_offset);
    mesh_marker.pose.position.z=rec_srv.response.pose.z - (pvct_1_z_offset);
    /*obj_broadcaster.sendTransform(tf::StampedTransform(
  	        tf::Transform(tf::Quaternion(pick_pose.pose.orientation.x, pick_pose.pose.orientation.y, pick_pose.pose.orientation.z, pick_pose.pose.orientation.w),
  	        tf::Vector3(pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z)),
  	      main_request.table.pose.header.stamp,"/base_link", "/object_training_frame"));*/
  }
  else if (label.substr(found+1)=="plug_1" || label.substr(found+1)=="plugf")
  {
    main_response.model_id=4;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/white_plug.STL";
    pick_pose.pose.position.x = rec_srv.response.pose.x - (plug_1_x_offset);
    pick_pose.pose.position.y = rec_srv.response.pose.y - (plug_1_y_offset);
    pick_pose.pose.position.z = rec_srv.response.pose.z - (plug_1_z_offset)+plug_pick_point_z;
    mesh_marker.pose.position.x=rec_srv.response.pose.x - (plug_1_x_offset);
    mesh_marker.pose.position.y=rec_srv.response.pose.y - (plug_1_y_offset);
    mesh_marker.pose.position.z=rec_srv.response.pose.z - (plug_1_z_offset);
    /*obj_broadcaster.sendTransform(tf::StampedTransform(
  	        tf::Transform(tf::Quaternion(pick_pose.pose.orientation.x, pick_pose.pose.orientation.y, pick_pose.pose.orientation.z, pick_pose.pose.orientation.w),
  	        tf::Vector3(pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z)),
  	      main_request.table.pose.header.stamp,"/base_link", "/object_training_frame"));*/
  }
  else if (label.substr(found+1)=="pvc_elbow_1" || label.substr(found+1)=="pvcelbow")
  {
    main_response.model_id=5;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/pvc_elbow.STL";
    pick_pose.pose.position.x = rec_srv.response.pose.x - (pvc_elbow_1_x_offset);
    pick_pose.pose.position.y = rec_srv.response.pose.y - (pvc_elbow_1_y_offset);
    pick_pose.pose.position.z = rec_srv.response.pose.z - (pvc_elbow_1_z_offset)+pvc_elbow_pick_point_z;
    mesh_marker.pose.position.x=rec_srv.response.pose.x - (pvc_elbow_1_x_offset);
    mesh_marker.pose.position.y=rec_srv.response.pose.y - (pvc_elbow_1_y_offset);
    mesh_marker.pose.position.z=rec_srv.response.pose.z - (pvc_elbow_1_z_offset);
    /*obj_broadcaster.sendTransform(tf::StampedTransform(
  	        tf::Transform(tf::Quaternion(pick_pose.pose.orientation.x, pick_pose.pose.orientation.y, pick_pose.pose.orientation.z, pick_pose.pose.orientation.w),
  	        tf::Vector3(pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z)),
  	      main_request.table.pose.header.stamp,"/base_link", "/object_training_frame"));*/
  }
  else
  {
	main_response.model_id=1;
	ROS_ERROR_STREAM("Object not properly identified");
  }
  ROS_WARN_STREAM("Object labeled as "<< label.substr(found+1)<<" with model id "<<main_response.model_id);
  //broadcast transform (mostly for visualization in rviz, as this information is passed back in response.pick_poses)
  obj_broadcaster.sendTransform(tf::StampedTransform(
	        tf::Transform(tf::Quaternion(pick_pose.pose.orientation.x, pick_pose.pose.orientation.y, pick_pose.pose.orientation.z, pick_pose.pose.orientation.w),
	        tf::Vector3(pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z)),
	      main_request.table.pose.header.stamp,"/base_link", "/object_training_frame"));



  //Push current pick pose into array of pick poses as part of recognition service
  main_response.pick_poses.push_back(pick_pose);

  //finish inputing marker properties and publish
    ROS_INFO_STREAM("Marker pose: \n x: " << mesh_marker.pose.position.x << "\n y: "<<mesh_marker.pose.position.y <<
		  "\n z: "<<mesh_marker.pose.position.z << "\n theta: "<<rec_srv.response.pose.rotation);
  main_response.mesh_marker = mesh_marker;
  vis_pub.publish( mesh_marker );

  ROS_INFO("CPH recognition complete");

//////Visualization: Matching PointCloud//////////////////////////////////////////////////////
      //Import pcd file which matches recognition response
      std::stringstream fileName;
      fileName << "/home"<<rec_srv.response.label << "_" << rec_srv.response.pose.rotation << ".pcd";
      //Load and convert file.
      pcl::PointCloud<pcl::PointXYZ>::Ptr trainingMatch (new pcl::PointCloud<pcl::PointXYZ>);

      pcl::io::loadPCDFile(fileName.str(), *trainingMatch);
      //Translate to location:
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*trainingMatch, centroid);
      pcl::demeanPointCloud<pcl::PointXYZ> (*trainingMatch, centroid, *trainingMatch);

      Eigen::Vector3f translate;
      Eigen::Quaternionf rotate;
      translate(0) = rec_srv.response.pose.x;
      translate(1) = rec_srv.response.pose.y;
      translate(2) = rec_srv.response.pose.z;
      rotate.setIdentity();

      pcl::transformPointCloud(*trainingMatch, *trainingMatch, translate, rotate);
      //make into ros message for publlishing
      sensor_msgs::PointCloud2 recognized_cloud;
      pcl::toROSMsg(*trainingMatch, recognized_cloud);
      //Add transform to header
      recognized_cloud.header.frame_id = main_request.table.pose.header.frame_id;
      recognized_cloud.header.stamp=main_request.table.pose.header.stamp;
      //Publish to topic /recognition_result.
      rec_pub.publish(recognized_cloud);
/////////end visualization////////////////////////////////////////////////////

  return true;
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_object_recognition");
  ros::NodeHandle n;  
  
  ros::ServiceServer rec_serv = n.advertiseService("/mantis_object_recognition", rec_cb);
  cph_client = n.serviceClient<nrg_object_recognition::recognition>("/cph_recognition");
  rec_pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_result",1);
  vis_pub = n.advertise<visualization_msgs::Marker>( "matching_mesh_marker", 10 );
  noise_pub=n.advertise<sensor_msgs::PointCloud2>("/noisy_points",1);
  
  ROS_INFO("mantis object detection/recognition node ready!");
  
  ros::spin(); 
}




