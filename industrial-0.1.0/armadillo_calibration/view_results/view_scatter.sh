#! /bin/bash
rosrun calibration_estimation error_visualization.py /tmp/armadillo_calibration/cal_measurements.bag /tmp/armadillo_calibration/ `rospack find armadillo_calibration`/view_results/scatter_config.yaml
