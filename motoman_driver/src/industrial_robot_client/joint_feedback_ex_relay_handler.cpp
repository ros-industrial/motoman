#include "motoman_driver/industrial_robot_client/joint_feedback_ex_relay_handler.h"
#include "simple_message/log_wrapper.h"

using industrial::joint_data::JointData;
using industrial::shared_types::shared_real;
using namespace industrial::simple_message;

namespace industrial_robot_client
{
namespace joint_feedback_ex_relay_handler
{

bool JointFeedbackExRelayHandler::init(SmplMsgConnection* connection,
                                     std::map<int, RobotGroup> &robot_groups)
{
    this->robot_groups_ = robot_groups;
    this->legacy_mode_ = false;
  bool rtn = JointRelayHandler::init(connection, (int)StandardMsgTypes::JOINT_FEEDBACK_EX, robot_groups);
  // try to read groups_number parameter, if none specified
  if ( (groups_number_ < 0) )
    node_.param("groups_number", groups_number_, 0);

  return rtn;
}

bool JointFeedbackExRelayHandler::init(SmplMsgConnection* connection,
                                     std::vector<std::string> &joint_names)
{
  this->legacy_mode_ = true;
  bool rtn = JointRelayHandler::init(connection, (int)StandardMsgTypes::JOINT_FEEDBACK_EX, joint_names);

  // try to read groups_number parameter, if none specified
  if ( (groups_number_ < 0) )
    node_.param("groups_number", groups_number_, 0);

  return rtn;
}


bool JointFeedbackExRelayHandler::create_messages(SimpleMessage& msg_in,
                                                control_msgs::FollowJointTrajectoryFeedback* control_state,
                                                sensor_msgs::JointState* sensor_state)
{
  // inspect groups_number field first, to avoid "Failed to Convert" message
  JointFeedbackExMessage tmp_msg;

  tmp_msg.init(msg_in);

  LOG_ERROR("IDEX, %d", tmp_msg.getGroupsNumber());

  for(int i=0; i< tmp_msg.getJointMessages().size();i++)
  {
      LOG_ERROR("%d", tmp_msg.getJointMessages()[i].getRobotID());
      create_messages(tmp_msg.getJointMessages()[i], control_state, sensor_state, tmp_msg.getJointMessages()[i].getRobotID());
  }

//  if (tmp_msg.getGroupsNumber() != groups_number_)
//  {
//    LOG_COMM("Ignoring Message: robotID (%d) doesn't match expected (%d)",
//             tmp_msg.getGroupsNumber(), groups_number_);
//    return false;
//  }

// if(this->legacy_mode_)
//    return JointRelayHandler::create_messages(msg_in, control_state, sensor_state);
// else
//    return JointRelayHandler::create_messages(msg_in, control_state, sensor_state, tmp_msg.getGroupsNumber());
}

bool JointFeedbackExRelayHandler::create_messages(JointFeedbackMessage& msg_in,
                                        control_msgs::FollowJointTrajectoryFeedback* control_state,
                                        sensor_msgs::JointState* sensor_state, int robot_id)
{
  DynamicJointPoint all_joint_state;
  if (!JointFeedbackExRelayHandler::convert_message(msg_in, &all_joint_state, robot_id))
  {
    LOG_ERROR("Failed to convert SimpleMessage");
    return false;
  }
  // apply transform, if required
  DynamicJointPoint xform_joint_state;
  if (!transform(all_joint_state, &xform_joint_state))
  {
    LOG_ERROR("Failed to transform joint state");
    return false;
  }

  //TODO: get the joint_names from the robotgroup instead of from the all_joint_names
  // select specific joints for publishing
  DynamicJointPoint pub_joint_state;
  std::vector<std::string> pub_joint_names;
  if (!select(xform_joint_state, robot_groups_[robot_id].get_joint_names(), &pub_joint_state, &pub_joint_names))
  {
    LOG_ERROR("Failed to select joints for publishing");
    return false;
  }

  // assign values to messages
  *control_state = control_msgs::FollowJointTrajectoryFeedback();  // always start with a "clean" message
  control_state->header.stamp = ros::Time::now();
  control_state->joint_names = pub_joint_names;
  control_state->actual.positions = pub_joint_state.positions;
  control_state->actual.velocities = pub_joint_state.velocities;
  control_state->actual.accelerations = pub_joint_state.accelerations;
  control_state->actual.time_from_start = pub_joint_state.time_from_start;

  *sensor_state = sensor_msgs::JointState();  // always start with a "clean" message
  sensor_state->header.stamp = ros::Time::now();
  sensor_state->name = pub_joint_names;
  sensor_state->position = pub_joint_state.positions;
  sensor_state->velocity = pub_joint_state.velocities;

  return true;
}


bool JointFeedbackExRelayHandler::convert_message(JointFeedbackMessage& msg_in, DynamicJointPoint* joint_state, int robot_id)
{
  JointData values;

  int num_jnts = robot_groups_[robot_id].get_joint_names().size();

  // copy position data
  if (msg_in.getPositions(values))
  {
    if (!JointDataToVector(values, joint_state->positions, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->positions.clear();

  // copy velocity data
  if (msg_in.getVelocities(values))
  {
    if (!JointDataToVector(values, joint_state->velocities, num_jnts))
    {
      LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->velocities.clear();

  // copy acceleration data
  if (msg_in.getAccelerations(values))
  {
    if (!JointDataToVector(values, joint_state->accelerations, num_jnts))
    {
      LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->accelerations.clear();

  // copy timestamp data
  shared_real value;
  if (msg_in.getTime(value))
    joint_state->time_from_start = ros::Duration(value);
  else
    joint_state->time_from_start = ros::Duration(0);

  return true;
}

bool JointFeedbackExRelayHandler::JointDataToVector(const JointData &joints,
                                                  std::vector<double> &vec,
                                                  int len)
{
  if ( (len<0) || (len>joints.getMaxNumJoints()) )
  {
    LOG_ERROR("Failed to copy JointData.  Len (%d) out of range (0 to %d)",
              len, joints.getMaxNumJoints());
    return false;
  }

  vec.resize(len);
  for (int i=0; i<len; ++i)
    vec[i] = joints.getJoint(i);

  return true;
}


}//namespace joint_feedback_ex_relay_handler
}//namespace industrial_robot_client




