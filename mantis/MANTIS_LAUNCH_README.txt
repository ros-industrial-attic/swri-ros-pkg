Directions for launching the mantis/automate demo:

Simulation Launch (in this order, seperate terminals):

If you are launching from an ssh client you will need to configure the diplay
for the remote system by 'export DISPLAY=:0' in whatever ssh terminal launches
RViz

roslaunch mantis_config mantis_dual_arm_setup.launch
roslaunch mantis_config mantis_dual_arm_visualization.launch
roslaunch mantis_object_manipulation start_dual_arm_sorting.launch
roslaunch mantis_object_manipulation launch_concurrent_arm_move_supervisor.launch

Actual Launch  (in this order, seperate terminals):

Same as simulation, except:
roslaunch mantis_config mantis_dual_arm_setup.launch sim_only:=false
