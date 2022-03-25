/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 * Author: Shaun Edwards
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Southwest Research Institute, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#include "motoman_driver/industrial_robot_client/motoman_utils.h"
#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>

namespace industrial_robot_client
{
namespace motoman_utils
{

bool getJointGroups(const std::string topic_param, std::map<int, RobotGroup> & robot_groups)
{
  if (ros::param::has(topic_param))
  {
    XmlRpc::XmlRpcValue topics_list_rpc;
    ros::param::get(topic_param, topics_list_rpc);


    std::vector<XmlRpc::XmlRpcValue> topics_list;

    ROS_INFO_STREAM("Loading topic list");
    ROS_INFO_STREAM("Found " << topics_list_rpc.size() << " topics");

    for (int i = 0; i < topics_list_rpc.size(); i++)
    {
      XmlRpc::XmlRpcValue state_value;
      state_value = topics_list_rpc[i];
      ROS_INFO_STREAM("Topic(state_value): " << state_value);
      topics_list.push_back(state_value);
    }


    for (size_t i = 0; i < topics_list.size(); i++)
    {
      ROS_INFO_STREAM("Loading group: " << topics_list[i]);
      RobotGroup rg;
      std::vector<std::string> rg_joint_names;

      XmlRpc::XmlRpcValue joints;

      joints = topics_list[i]["joints"];
      for (int jt = 0; jt < joints.size(); jt++)
      {
        rg_joint_names.push_back(static_cast<std::string>(joints[jt]));
      }

      XmlRpc::XmlRpcValue group_number;


      group_number = topics_list[i]["group"];
      int group_number_int = static_cast<int>(group_number);

      XmlRpc::XmlRpcValue name;
      std::string name_string;

      name = topics_list[i]["name"];
      name_string = static_cast<std::string>(name);

      XmlRpc::XmlRpcValue ns;
      std::string ns_string;

      ns = topics_list[i]["ns"];

      ns_string = static_cast<std::string>(ns);

      ROS_DEBUG_STREAM("Setting group: ");
      ROS_DEBUG_STREAM("  group number: " << group_number);
      ROS_DEBUG_STREAM("  group number(int): " << group_number_int);
      ROS_DEBUG_STREAM("  joints_names(size): " << rg_joint_names.size());
      ROS_DEBUG_STREAM("  name: " << name_string);
      ROS_DEBUG_STREAM("  ns: " << ns_string);
      rg.set_group_id(group_number_int);
      rg.set_joint_names(rg_joint_names);
      rg.set_name(name_string);
      rg.set_ns(ns_string);

      robot_groups[group_number] = rg;
    }

    ROS_INFO_STREAM("Loaded " << robot_groups.size() << " groups");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Failed to find '" << topic_param << "' parameter");
    return false;
  }
}

}  // namespace motoman_utils
}  // namespace industrial_robot_client

