/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, TU Delft Robotics Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the TU Delft Robotics Institute nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: G.A. vd. Hoorn - TU Delft Robotics Institute
 */

#include <fanuc_common/fanuc_utils.h>

#include <industrial_robot_client/robot_state_interface.h>
#include <industrial_utils/param_utils.h>


using industrial_robot_client::robot_state_interface::RobotStateInterface;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_utils::param::getJointNames;


class Fanuc_JointRelayHandler : public JointRelayHandler
{
	int J23_factor_;


public:
	Fanuc_JointRelayHandler() : JointRelayHandler(), J23_factor_(0)
	{
		if (ros::param::has("J23_factor"))
		{
			ros::param::get("J23_factor", this->J23_factor_);
		}
		else
		{
			// TODO: abort on missing parameter
			ROS_ERROR("Joint 2-3 linkage factor parameter not supplied.");
		}
	}


	virtual ~Fanuc_JointRelayHandler() {}


	bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
	{
		// compensate for J2-J3 coupling in Fanuc manipulator, if required
		fanuc::utils::linkage_transform(pos_in, pos_out, J23_factor_);
		return true;
	}
};


int main(int argc, char** argv)
{
	// initialize node
	ros::init(argc, argv, "state_interface");

	// launch the default Robot State Interface connection/handlers
	RobotStateInterface rsi;
	rsi.init();

	// replace the generic JointRelayHandler with our Fanuc specific one as
	// we need to correct the reported joint angles
	Fanuc_JointRelayHandler jointHandler;
	std::vector<std::string> joint_names;
	getJointNames("controller_joint_names", joint_names);
	jointHandler.init(rsi.get_connection(), joint_names);
	rsi.add_handler(&jointHandler);

	// run the node
	rsi.run();

	return 0;
}
