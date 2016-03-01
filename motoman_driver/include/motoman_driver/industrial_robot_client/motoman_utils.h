/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_MOTOMAN_UTILS_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_MOTOMAN_UTILS_H

#include <sstream>
#include <iostream>
#include <string>
#include <map>

#include "motoman_driver/industrial_robot_client/robot_group.h"

namespace industrial_robot_client
{
namespace motoman_utils
{

bool getJointGroups(std::string topic_param, std::map<int, RobotGroup> robot_groups);

/**
  @brief Common error message displayed when node fails to find 'topic_list' parameter
  */

const std::string TOPIC_LIST_ERROR_MSG(
    "Failed to find 'topic_list' parameter (new in Indigo)\r\n"
    "  \tNew parameter documentaion can be found here:\r\n"
    "  \thttp://wiki.ros.org/motoman_driver/Tutorials/Creating%20a%20Dual-Arm%20System\r\n"
    "If still using the Hydro server version on the controller, then set the 'version0' parameter to FALSE\r\n"
    "  \tThe driver will assume a single arm with joint names in order of the URDF (base to tip) OR\r\n"
    "  \tit will read the joint order from the 'controller_joint_names' parameter described here:\r\n"
    "  \t\thttp://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot#Update_Configuration_Files"
    );
}//motoman_utils
}//industrial_robot_client

#endif /* MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_MOTOMAN_UTILS_H */
