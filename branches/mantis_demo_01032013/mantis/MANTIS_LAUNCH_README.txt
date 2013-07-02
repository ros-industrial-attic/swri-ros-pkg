------------------------------------------------------------------------------------------
Dual Robot Demo ( mantis/automate):

If you are launching from an ssh client you will need to configure the diplay
for the remote system by 'export DISPLAY=:0' in whatever ssh terminal launches
RViz

 - Simulation Launch (in this order, separate terminals):

	roslaunch mantis_config mantis_dual_arm_setup.launch
	roslaunch mantis_config mantis_dual_arm_visualization.launch
	roslaunch mantis_object_manipulation start_dual_arm_sorting.launch
	roslaunch mantis_object_manipulation launch_concurrent_arm_move_supervisor.launch


 - Actual Launch  (in this order, separate terminals):

	roslaunch mantis_io mantis_io.launch
	roslaunch mantis_config mantis_dual_arm_setup.launch sim_only:=false
	roslaunch mantis_config mantis_dual_arm_visualization.launch
	roslaunch mantis_object_manipulation start_dual_arm_sorting.launch
	roslaunch mantis_object_manipulation launch_concurrent_arm_move_supervisor.launch
	telnet 192.168.32.3
	    -user: Net_Maint_Mng
	    -pass: 99999999

------------------------------------------------------------------------------------------
UR5 sorting demo
- Real Robot (run each shell command in separate terminals and in this specific order):

	- roslaunch mantis_io mantis_io.launch
	- roslaunch mantis_config mantis_ur5_sort_clutter_setup.launch sim_only:=false sensor_usb_manual_setup:=true
	- roslaunch mantis_config mantis_ur5_visualization.launch
	- roslaunch mantis_object_manipulation start_ur5_automated_sorting.launch

- Simulation mode (run each shell command in separate terminals and in this specific order):

	- roslaunch mantis_config mantis_ur5_sort_clutter_setup.launch sensor_usb_manual_setup:=true
	- roslaunch mantis_config mantis_ur5_visualization.launch
	- roslaunch mantis_object_manipulation start_ur5_automated_sorting.launch

------------------------------------------------------------------------------------------
UR5 recognition and pose estimation demo 
- Extra Hardware Requirements:	
	3-Finger gripper
	Prosilica camera (Pose correction)

- Real Robot (run each shell command in separate terminals and in this specific order):

	- roslaunch mantis_config mantis_ur5_recognition_pick_place_setup.launch sim_only:=false
	- roslaunch mantis_object_manipulation start_ur5_recognition_pick_place.launch

- Simulation Mode (run each shell command in separate terminals and in this specific order):

	- roslaunch mantis_config mantis_ur5_recognition_pick_place_setup.launch 
	- roslaunch mantis_object_manipulation start_ur5_recognition_pick_place.launch
