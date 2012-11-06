//main_dataset_node
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include <flann/flann.h>

#include <boost/filesystem.hpp>

#include "cph.h"
#include "mantis_perception/mantis_recognition.h"
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"

//float subtract_angle(float angle_1, float angle_2);

ros::ServiceClient cph_client;//, seg_client;



//ros::Publisher pan_pub;
ros::Publisher rec_pub;
//sensor_msgs::PointCloud2 cloud_to_process;


/*float subtract_angle(float angle_1, float angle_2){
 if(angle_1-angle_2 > -180 && angle_1-angle_2 < 180)
   return(angle_1-angle_2);
 else if(angle_1-angle_2 < -180)
   return(angle_1-angle_2+360);
 else
   return(angle_1-angle_2-360);
}

void kinect_cb(sensor_msgs::PointCloud2 fromKinect)
{
cloud_to_process = fromKinect;
}
*/
bool rec_cb(mantis_perception::mantis_recognition::Request &main_request,
            mantis_perception::mantis_recognition::Response &main_response)
{  


  nrg_object_recognition::recognition rec_srv;


  sensor_msgs::PointCloud2 cluster;
  sensor_msgs::PointCloud received_cluster;
  received_cluster=main_request.clusters.at(0);
  sensor_msgs::convertPointCloudToPointCloud2(received_cluster, cluster);

  rec_srv.request.cluster = cluster;
  rec_srv.request.threshold = 10000;

  main_response.label = rec_srv.response.label;
  main_response.pose = rec_srv.response.pose;
  main_response.model_id=0;


  ROS_INFO("Model label: %s", main_response.label.c_str());
  ROS_INFO("Model label: %d", main_response.model_id);

  /*std_msgs::UInt16 command;
  command.data = 0;
  ros::Rate loop_rate(.1);
  
  pan_pub.publish(command);
  loop_rate.sleep();
  

  std::string objectName = main_request.object_name;
  std::vector<float> pcc_row(k,0); //will hold class conditional probabilities. P(c|testObject)
  std::vector<float> filtered_result(k,0); //holds filtered result.
  
  nrg_object_recognition::recognition rec_srv;
  //nrg_object_recognition::segmentation seg_srv;
  int num_objects = 0;
  std::string angleStr;
  float angle=0, pose_err = 0, cum_err=0, filtered_pose_err=0, cum_filt_dev=0;
  
  std::cout << "test running...\n"; 
  //For each view - begin image iterator
  for(unsigned int i=0; i<main_request.num_images; i++)
    {
    //Set up probabilistic filters:
    std::vector<float> classProb;
    Eigen::Matrix4f K, Sigma, Q, I;
    Eigen::Vector4f pose;
    Q(0,0) = .005f; Q(1,1) = .005f; Q(2,2) = .005f; Q(3,3) = 0.1;
    I.setIdentity();
      
    //start with uniform prior
    classProb.clear();
    classProb.resize(k,(float)1/7.0f);
  
    //Start with high covariance:
    Sigma(0,0) = 1000; Sigma(1,1) = 1000; Sigma(2,2) = 1000; Sigma(3,3) = 1000;
  
    //Hold results:
    std::string label, angleStr;
    int angle, z;
    Eigen::Vector4f translation;
    
    //Take several images (we were setting num_samples to 1)
    for(unsigned int j=0; j<main_request.num_samples; j++)
    {
      
      //Call segmentation service...
      std::cout << "time stamp of cloud being processed: " << cloud_to_process.header.stamp << std::endl;
      seg_srv.request.scene = cloud_to_process;
      seg_srv.request.min_x = -.75, seg_srv.request.max_x = .5;
      seg_srv.request.min_y = -.10, seg_srv.request.max_y = 1.0;
      seg_srv.request.min_z = 0.0, seg_srv.request.max_z = 1.3;
      seg_client.call(seg_srv);
      rec_srv.request.cluster = seg_srv.response.clusters.at(0);
	
      //could iterate through all clusters here in future app.
      rec_srv.request.cluster =main_request.clusters[0];
      rec_srv.request.threshold = 10000;
	
      //Call recognition service
      cph_client.call(rec_srv);

      num_objects++;
	
      //Increment appropriate row/object
      z = classMap[rec_srv.response.label];
      pcc_row.at(z)++;
	
      //compute error in pose estimate, and accumulate squared error.
      angle = 0; //angle is ground truth angle

      translation(0) = rec_srv.response.pose.x;
      translation(1) = rec_srv.response.pose.y;
      translation(2) = rec_srv.response.pose.z;
      pose_err = subtract_angle(angle, pose(3));
      //std::cout << pose_err << std::endl;
      cum_err += pow(pose_err, 2);
	
      //Visualization://////////////////////////////////////////////////////
      //build filename.
      std::stringstream fileName;
      fileName << "data/" << rec_srv.response.label << "_" << rec_srv.response.pose.rotation << ".pcd";
      //Load and convert file.
      pcl::PointCloud<pcl::PointXYZ>::Ptr trainingMatch (new pcl::PointCloud<pcl::PointXYZ>);
      sensor_msgs::PointCloud2 rosMsg;
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

      pcl::toROSMsg(*trainingMatch, rosMsg);
      //Add transform to header
      rosMsg.header.frame_id = "/camera_depth_optical_frame";
      //Publish to topic /recognition_result.
      rec_pub.publish(rosMsg);
      /////////end visualization////////////////////////////////////////////////////

      //Filter results:
      //Bayes filter for class probability:
      //Compute denominator:
      float bayesDen = 0;
      for(unsigned int ck=0; ck<7; ck++)
        {
          bayesDen += probTable.at(ck).at(z)*classProb.at(ck);
	  //std::cout << "Bayes Denominator:" << bayesDen << std::endl;
        }
      //Update P(C|z)
      //std::cout << "probability: [";
      for(unsigned int cj=0; cj<k; cj++)
      {
        classProb.at(cj) = probTable.at(cj).at(z)*classProb.at(cj)/bayesDen;
        //std::cout << classProb.at(cj) << " ";
      }
      //std::cout << "]\n";

      //Filter the pose estimate info in:
      if(j==0)
      {
        pose(3) = rec_srv.response.pose.rotation;
      }

      else
      {
        Q(3,3) = pose_dev.at(z);
        //std::cout << "Q: " << Q << std::endl;
        K = Sigma*(Sigma + Q).inverse();
        //std::cout << "K: " << K << std::endl;
        float newAngle = pose(3) + K(3,3)*subtract_angle(rec_srv.response.pose.rotation, pose(3));
        //std::cout << "newAngle: " << newAngle << std::endl;
        if(newAngle < 0)
          newAngle += 360;
          pose = pose + K*(translation-pose);
        if(newAngle > 360)
          newAngle -= 360;
          pose(3) = newAngle;
	  Sigma = (I-K)*Sigma;
	  //std::cout << "Sigma: " << Sigma << std::endl;
      }
      //std::cout << "pose: " << pose(3) << std::endl;
      ros::spinOnce();
    }
    
    //increment bin of label
    for(unsigned int class_it = 0; class_it < k; class_it++)
    {
      if(classProb.at(class_it) > classProb.at(z))
        z = class_it;
    }
    //std::cout << "current label estimate: " << z << std::endl;
    filtered_result.at(z)++;
    cum_filt_dev += pow(subtract_angle(pose(3),angle),2);//Sigma(3,3);
    // std::cout << "tested " << num_objects << " cases...\n";
      
      
    command.data += 360/main_request.num_images;
    pan_pub.publish(command);
    loop_rate.sleep();
  }//end image iterator

  std::cout << "done.\n";
  main_response.sigma_pose = pow(cum_err/num_objects, .5);
  //std::cout << "Raw recognition distribution:\n";
  for(unsigned int i=0; i<pcc_row.size(); i++)
  {
    pcc_row.at(i) = pcc_row.at(i)/num_objects;
    //std::cout << pcc_row.at(i) << " ";
  }
  //std::cout << std::endl;
  
   //std::cout << "Filtered recognition distribution:\n";
  for(unsigned int i=0; i<pcc_row.size(); i++)
  {
    filtered_result.at(i) = main_request.num_samples*filtered_result.at(i)/num_objects;
    //std::cout << filtered_result.at(i) << " ";
  }
  
  main_response.rec_rate = pcc_row.at(classMap[objectName]);
  main_response.prob_dist = pcc_row;
  main_response.sigma_filtered = pow(main_request.num_samples*cum_filt_dev/num_objects,.5);
  main_response.filt_dist = filtered_result;

 */
  return(1);
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_object_recognition");
  ros::NodeHandle n;  
  
  ros::ServiceServer rec_serv = n.advertiseService("/recognition_service", rec_cb);
  cph_client = n.serviceClient<nrg_object_recognition::recognition>("cph_recognition");
  //seg_client = n.serviceClient<nrg_object_recognition::segmentation>("segmentation");
  //ros::Subscriber kin_sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  //pan_pub = n.advertise<std_msgs::UInt16>("/pan_command",1);
  rec_pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_result",1);
  
  std::vector<std::vector<float> > probTable;
  std::map<std::string, int> classMap;
  std::vector<float> pose_dev;
  std::ifstream objectListFile;
  objectListFile.open("classes.list", std::ios::in);
  std::string tempName, objectLine;
  int k = 0;
  std::vector<float> probRow;
  
  
  while(std::getline(objectListFile, objectLine)){
    std::istringstream objectSS(objectLine);
    //Record object name and add to map.
    objectSS >> tempName;
    classMap.insert(std::pair<std::string, int>(tempName, k));
    //Get pose sigma 
    objectSS >> tempName;
    pose_dev.push_back(atof(tempName.c_str()));
    
    //populate probability table (line k;)
    while(objectSS >> tempName){
      probRow.push_back(atof(tempName.c_str()));
    }
    probTable.push_back(probRow);
    probRow.clear();
    k++;  
  }
  if(k != probTable.at(0).size())
    std::cout << "Error: Probability table is not square!" << std::endl;
  
  for(unsigned int i=0; i<k; i++){
   for(unsigned int j=0; j<k; j++){
     std::cout << probTable.at(i).at(j) << " ";
   }
   std::cout << std::endl;
  }
  std::cout << probTable.size() << " objects in set.\n";
  
  ROS_INFO("mantis object detection/recognition node ready!");
  
  ros::spin(); 
}




