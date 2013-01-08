Directions for launching the mantis/automate demo:

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
