//main_dataset_node
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include <flann/flann.h>

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include "cph.h"
#include "nrg_object_recognition/run_data.h"
#include "nrg_object_recognition/recognition.h"
#include "nrg_object_recognition/segmentation.h"

float subtract_angle(float angle_1, float angle_2);

ros::ServiceClient cph_client, vfh_client, seg_client;
std::vector<std::vector<float> > probTable;
std::map<std::string, int> classMap;
std::vector<float> pose_dev;
int k;

ros::Publisher pan_pub;
ros::Publisher rec_pub;
sensor_msgs::PointCloud2 cloud_to_process;

void kinect_cb(sensor_msgs::PointCloud2 fromKinect)
{
cloud_to_process = fromKinect;
}


bool test_cb(nrg_object_recognition::run_data::Request &main_request,
	    nrg_object_recognition::run_data::Response &main_response)
{  
  std::string objectName = main_request.object_name;
  std::stringstream fileName;
  std::vector<float> pcc_row(k,0); //will hold class conditional probabilities. P(c|testObject)
  std::vector<float> filtered_result(k,0); //holds filtered result.
  
  nrg_object_recognition::recognition rec_srv;
  int num_objects = 0;
  std::string angleStr;
  float angle=0, pose_err = 0, cum_err=0, filtered_pose_err=0, cum_filt_dev=0;
  
   //Noise generator:
  boost::mt19937 mers;
  mers.seed(static_cast<unsigned int>(std::time(0)));
  boost::normal_distribution<float> dist(0, main_request.noise_level);
  boost::variate_generator<boost::mt19937 , boost::normal_distribution<float> > noise(mers,dist);
  
  std::cout << "test running...\n"; 
  int avgSize = 0;
  int size_idx = 0;
  //For each test file:
  for(boost::filesystem::directory_iterator it ("test"); it != boost::filesystem::directory_iterator (); ++it){
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == ".pcd")
    {
      //Set up probabilistic filters:
      std::vector<float> classProb;
      Eigen::Matrix4f K, Sigma, Q, I;
      Eigen::Vector4f pose;
      Q(0,0) = .005f; Q(1,1) = .005f; Q(2,2) = .005f; Q(3,3) = 0.1;
      I.setIdentity();
      
      //start with uniform prior
      classProb.clear();
      classProb.resize(7,(float)1/7.0f);
    
      //Start with high covariance:
      Sigma(0,0) = 1000; Sigma(1,1) = 1000; Sigma(2,2) = 1000; Sigma(3,3) = 1000;
    
      //Hold results:
      std::string label, angleStr;
      int angle, z;
      Eigen::Vector4f translation;
      
      int cloudSize=0;
      
      //Iterate through noisy samples
      for(unsigned int j=0; j<main_request.num_samples; j++){
      
	//Read .pcd file.
	fileName.str("");
	fileName << "test/" << it->path().filename().c_str();
	//std::cout << "reading file: " << fileName.str() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile (fileName.str(), *cluster);
	
// 	cloudSize = cluster->points.size();
// 	avgSize = (avgSize*size_idx+cloudSize)/(size_idx+1);
// 	std::cout << "j: " << size_idx << std::endl;
// 	std::cout << "cloudSize: " << cloudSize << std::endl;
// 	std::cout << "avg Size: " << avgSize << std::endl;
// 	size_idx++;
	
	//Add noise to the z channel.
	for(size_t idx = 0; idx < cluster->points.size(); idx++){
	  cluster->points[idx].z += noise();
	}
	
	pcl::toROSMsg(*cluster, rec_srv.request.cluster);
	rec_srv.request.threshold = 10000; //(nearest neighbor no matter how far.)
	
	if(main_request.method == 0){
	 cph_client.call(rec_srv); 
	}
	else if(main_request.method == 1){
	 vfh_client.call(rec_srv); 
	}
	num_objects++;
	
	//std::cout << "label: " << rec_srv.response.label << std::endl;
	//std::cout << "pose: " << rec_srv.response.pose.rotation << std::endl;
	
	//Increment appropriate row.
	z = classMap[rec_srv.response.label];
	pcc_row.at(z)++;
	
	//compute error in pose estimate, and accumulate squared error.
	std::string fileNameString = fileName.str();
	angleStr.assign(fileNameString.begin()+fileNameString.rfind("_")+1, fileNameString.end()-3);
	angle = atof(angleStr.c_str()); //angle is ground truth angle
	translation(0) = rec_srv.response.pose.x; translation(1) = rec_srv.response.pose.y; translation(2) = rec_srv.response.pose.z;
	pose_err = subtract_angle(angle, rec_srv.response.pose.rotation);
	//std::cout << "pose error: " << pose_err << std::endl;
	cum_err += pow(pose_err, 2);
	
	//Filter results:
	//Bayes filter for class probability:
        //Compute denominator:
        float bayesDen = 0;
	for(unsigned int ck=0; ck<7; ck++){
	  bayesDen += probTable.at(ck).at(z)*classProb.at(ck);
	  //std::cout << "Bayes Denominator:" << bayesDen << std::endl;
	}
	//Update P(C|z)
	//std::cout << "probability: [";
	for(unsigned int cj=0; cj<7; cj++){
	 classProb.at(cj) = probTable.at(cj).at(z)*classProb.at(cj)/bayesDen;
	 //std::cout << classProb.at(cj) << " ";
	}
	//std::cout << "]\n";
	
	//Filter the pose estimate info in:
	if(j==0){
	  pose(3) = rec_srv.response.pose.rotation;
	}
	
	else{
	  Q(3,3) = pose_dev.at(z);
	  std::cout << "Q: " << Q << std::endl;
	  K = Sigma*(Sigma + Q).inverse();
	  std::cout << "K: " << K << std::endl;
	  float newAngle = pose(3) + K(3,3)*subtract_angle(rec_srv.response.pose.rotation, pose(3));
	  std::cout << "newAngle: " << newAngle << std::endl;
	  if(newAngle < 0)
	    newAngle += 360;
	  pose = pose + K*(translation-pose);
	  if(newAngle > 360)
	    newAngle -= 360;
	  pose(3) = newAngle;
	  Sigma = (I-K)*Sigma; 
	  std::cout << "Sigma: " << Sigma << std::endl;
	}
	std::cout << "pose: " << pose(3) << std::endl;
      }
      //increment bin of label
      for(unsigned int class_it = 0; class_it < 7; class_it++){
	if(classProb.at(class_it) > classProb.at(z))
	  z = class_it;	
      }
      filtered_result.at(z)++;
      cum_filt_dev += pow(subtract_angle(pose(3),angle),2);//Sigma(3,3);
      //std::cout << "tested " << num_objects << " cases...\n"; 
    }
  }
  std::cout << "done.\n";
  main_response.sigma_pose = pow(cum_err/num_objects, .5);
  //std::cout << "Raw recognition distribution:\n";
  for(unsigned int i=0; i<pcc_row.size(); i++){
    pcc_row.at(i) = pcc_row.at(i)/num_objects;
    //std::cout << pcc_row.at(i) << " ";
  }
  //std::cout << std::endl;
  
  //std::cout << "Filtered recognition distribution:\n";
  for(unsigned int i=0; i<pcc_row.size(); i++){
    filtered_result.at(i) = main_request.num_samples*filtered_result.at(i)/num_objects;
    //std::cout << filtered_result.at(i) << " ";
  }
  
  main_response.rec_rate = pcc_row.at(classMap[objectName]);
  main_response.prob_dist = pcc_row;
  main_response.sigma_filtered = pow(main_request.num_samples*cum_filt_dev/num_objects,.5);
  main_response.filt_dist = filtered_result;
  
  return(1);
}

bool live_cb(nrg_object_recognition::run_data::Request &main_request,
	    nrg_object_recognition::run_data::Response &main_response)
{  
  std_msgs::UInt16 command; 
  command.data = 0;
  ros::Rate loop_rate(.1);
  
   pan_pub.publish(command);
   loop_rate.sleep();
  
  std::string objectName = main_request.object_name;
  std::vector<float> pcc_row(k,0); //will hold class conditional probabilities. P(c|testObject)
  std::vector<float> filtered_result(k,0); //holds filtered result.
  
  nrg_object_recognition::recognition rec_srv;
  nrg_object_recognition::segmentation seg_srv;
  int num_objects = 0;
  std::string angleStr;
  float angle=0, pose_err = 0, cum_err=0, filtered_pose_err=0, cum_filt_dev=0;
  
  std::cout << "test running...\n"; 
  //For each view
  for(unsigned int i=0; i<main_request.num_images; i++){
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
      
      //Take several images.
      for(unsigned int j=0; j<main_request.num_samples; j++){
      
	//Call segmentation service...
	std::cout << "time stamp of cloud being processed: " << cloud_to_process.header.stamp << std::endl;
	seg_srv.request.scene = cloud_to_process;
	seg_srv.request.min_x = -.75, seg_srv.request.max_x = .4;
	seg_srv.request.min_y = -5, seg_srv.request.max_y = .5;
	seg_srv.request.min_z = 0.0, seg_srv.request.max_z = 1.15;
	seg_client.call(seg_srv);
	
	//could iterate through all clusters here in future app.
	
	rec_srv.request.cluster = seg_srv.response.clusters.at(0);
	rec_srv.request.threshold = 10000;
	
	//Call recognition service...
	if(main_request.method == 0){
	 cph_client.call(rec_srv); 
	}
	else if(main_request.method == 1){
	 vfh_client.call(rec_srv); 
	}
	num_objects++;
	
	
	
	//Increment appropriate row.
	z = classMap[rec_srv.response.label];
	pcc_row.at(z)++;
	
	//compute error in pose estimate, and accumulate squared error.
	angle = command.data; //angle is ground truth angle
	
	translation(0) = rec_srv.response.pose.x; translation(1) = rec_srv.response.pose.y; translation(2) = rec_srv.response.pose.z;
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
	for(unsigned int ck=0; ck<7; ck++){
	  bayesDen += probTable.at(ck).at(z)*classProb.at(ck);
	  //std::cout << "Bayes Denominator:" << bayesDen << std::endl;
	}
	//Update P(C|z)
	//std::cout << "probability: [";
	for(unsigned int cj=0; cj<k; cj++){
	 classProb.at(cj) = probTable.at(cj).at(z)*classProb.at(cj)/bayesDen;
	 //std::cout << classProb.at(cj) << " ";
	}
	//std::cout << "]\n";
	
	//Filter the pose estimate info in:
	if(j==0){
	  pose(3) = rec_srv.response.pose.rotation;
	}
	
	else{
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
      for(unsigned int class_it = 0; class_it < k; class_it++){
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
  for(unsigned int i=0; i<pcc_row.size(); i++){
    pcc_row.at(i) = pcc_row.at(i)/num_objects;
    //std::cout << pcc_row.at(i) << " ";
  }
  //std::cout << std::endl;
  
  //std::cout << "Filtered recognition distribution:\n";
  for(unsigned int i=0; i<pcc_row.size(); i++){
    filtered_result.at(i) = main_request.num_samples*filtered_result.at(i)/num_objects;
    //std::cout << filtered_result.at(i) << " ";
  }
  
  main_response.rec_rate = pcc_row.at(classMap[objectName]);
  main_response.prob_dist = pcc_row;
  main_response.sigma_filtered = pow(main_request.num_samples*cum_filt_dev/num_objects,.5);
  main_response.filt_dist = filtered_result;
  
  return(1);
}

bool roc_cb(nrg_object_recognition::run_data::Request &main_request,
	    nrg_object_recognition::run_data::Response &main_response)
{  
  std::string objectName = main_request.object_name;
  std::stringstream fileName;
  std::vector<float> pcc_row(k,0); //will hold class conditional probabilities. P(c|testObject)
  std::vector<float> filtered_result(k,0); //holds filtered result.
  
  nrg_object_recognition::recognition rec_srv;
  int num_false=0, total_true=0, total_false=0;
  std::ofstream roc_file;
  roc_file.open("ROC_data.csv");
  
   //Noise generator:
  boost::mt19937 mers;
  mers.seed(static_cast<unsigned int>(std::time(0)));
  boost::normal_distribution<float> dist(0, main_request.noise_level);
  boost::variate_generator<boost::mt19937 , boost::normal_distribution<float> > noise(mers,dist);
  
  std::cout << "generating ROC data...\n"; 
  
  //For several thresholds:
  for(unsigned int thresh = 0; thresh < 5000; thresh += 50){
    num_false=0; total_false=0; total_true=0;
    //For each test file:
    for(boost::filesystem::directory_iterator it ("test"); it != boost::filesystem::directory_iterator (); ++it){
      if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == ".pcd")
      {    
	//Hold results:
	std::string label;
	int z;
	
	//Read .pcd file.
	fileName.str("");
	fileName << "test/" << it->path().filename().c_str();
	//std::cout << "reading file: " << fileName.str() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile (fileName.str(), *cluster);
	
	//check to see if it is a true instance or a false instance:
	std::string fileNameString = fileName.str();
	label.assign(fileNameString.begin(), fileNameString.begin()+fileNameString.rfind("_"));
	std::cout << label << std::endl;
	if(label == objectName)
	  total_true++;
	else
	  total_false++;
	
	
	//Add noise to the z channel.
	for(size_t idx = 0; idx < cluster->points.size(); idx++){
	  cluster->points[idx].z += noise();
	}
	
	pcl::toROSMsg(*cluster, rec_srv.request.cluster);
	rec_srv.request.threshold = thresh; //()
	
	if(main_request.method == 0){
	  cph_client.call(rec_srv); 
	}
	else if(main_request.method == 1){
	  vfh_client.call(rec_srv); 
	}
	
	//Increment appropriate row.
	z = classMap[rec_srv.response.label];
	pcc_row.at(z)++;
	
      }
    }
    for (unsigned int i = 0; i< pcc_row.size(); i++){
     if(i != classMap[objectName])
       num_false += pcc_row.at(i);
    }
    
    float FPR = float(num_false)/float(total_false);
    float TPR = float(pcc_row.at(classMap[objectName]))/float(total_true);
    roc_file << thresh << FPR << TPR << std::endl;
  }
  roc_file.close();
  std::cout << "done.\n";  
  return(1);
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "main_dataset_node");
  ros::NodeHandle n;  
  
  ros::ServiceServer offline_serv = n.advertiseService("/run_test", test_cb);
  ros::ServiceServer live_serv = n.advertiseService("/live_test", live_cb);
  cph_client = n.serviceClient<nrg_object_recognition::recognition>("cph_recognition");
  vfh_client = n.serviceClient<nrg_object_recognition::recognition>("vfh_recognition");
  seg_client = n.serviceClient<nrg_object_recognition::segmentation>("segmentation");
  ros::Subscriber kin_sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  pan_pub = n.advertise<std_msgs::UInt16>("/pan_command",1);
  rec_pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_result",1);
  
  
  std::ifstream objectListFile;
  objectListFile.open("classes.list", std::ios::in);
  std::string tempName, objectLine;
  k = 0;
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
  
  ROS_INFO("main_dataset_node ready!");
  
  ros::spin(); 
}

float subtract_angle(float angle_1, float angle_2){
 if(angle_1-angle_2 > -180 && angle_1-angle_2 < 180)
   return(angle_1-angle_2);
 else if(angle_1-angle_2 < -180)
   return(angle_1-angle_2+360);
 else
   return(angle_1-angle_2-360);
}

