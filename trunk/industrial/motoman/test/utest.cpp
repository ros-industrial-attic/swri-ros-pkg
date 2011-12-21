/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils.h"
#include "definitions.h"

#include <vector>
#include <gtest/gtest.h>

using namespace motoman::utils;

TEST(Utils, hasSuffix)
{
  std::string test = "prefix_root_suffix";
  std::string suffix = "suffix";
  std::string root = "root";
  std::string prefix = "prefix";
  std::string empty = "";
  EXPECT_TRUE(hasSuffix(test, test));
  EXPECT_TRUE(hasSuffix(test, suffix));
  EXPECT_FALSE(hasSuffix(test, root));
  EXPECT_FALSE(hasSuffix(test, prefix));
  EXPECT_TRUE(hasSuffix(test, empty));

}

TEST(Utils, checkJointNames)
{
  std::vector<std::string> names;
  EXPECT_TRUE(checkJointNames(names));
  names.push_back("joint_s");
  EXPECT_TRUE(checkJointNames(names));
  names.push_back("prefix_joint_l");
  EXPECT_TRUE(checkJointNames(names));
  names.push_back("bad_joint");
  EXPECT_FALSE(checkJointNames(names));
  names.clear();
  names.resize(motoman::parameters::Parameters::JOINT_SUFFIXES_SIZE + 1 );
  EXPECT_FALSE(checkJointNames(names));

  names.clear();
  for (int i = 0; i < motoman::parameters::Parameters::JOINT_SUFFIXES_SIZE; i++)
  {
    names.push_back(motoman::parameters::Parameters::JOINT_SUFFIXES[i]);
  }
  EXPECT_TRUE(checkJointNames(names));

}

TEST(Utils, toMotomanVelocity)
{
  std::vector<double> vel;
  std::vector<double> lim;

  vel.push_back(1.0);
  lim.push_back(2.0);

  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 1.0/2.0);
  EXPECT_FLOAT_EQ(toMotomanVelocity(vel, lim), 1.0);

  vel.push_back(2.0);
  lim.push_back(3.0);

  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 2.0/3.0);

  vel.push_back(3.0);
  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 0.0);

  lim.push_back(4.0);
  lim.push_back(5.0);
  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 0.0);

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

