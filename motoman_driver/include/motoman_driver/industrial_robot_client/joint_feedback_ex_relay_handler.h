#ifndef JOINT_FEEDBACK_EX_RELAY_HANDLER_H
#define JOINT_FEEDBACK_EX_RELAY_HANDLER_H


#include "motoman_driver/industrial_robot_client/joint_relay_handler.h"
#include "motoman_driver/industrial_robot_client/joint_feedback_relay_handler.h"
#include "simple_message/messages/joint_feedback_ex_message.h"
#include "industrial_msgs/DynamicJointPoint.h"
#include "industrial_msgs/DynamicJointTrajectoryFeedback.h"

namespace industrial_robot_client
{
namespace joint_feedback_ex_relay_handler
{

using industrial::joint_feedback_ex_message::JointFeedbackExMessage;
using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_robot_client::joint_feedback_relay_handler::JointFeedbackRelayHandler;
using trajectory_msgs::JointTrajectoryPoint;
using industrial_msgs::DynamicJointPoint;
using industrial_msgs::DynamicJointTrajectoryFeedback;

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackExRelayHandler : public industrial_robot_client::joint_relay_handler::JointRelayHandler
{

public:

  /**
* \brief Constructor
*/
  JointFeedbackExRelayHandler(int groups_number=-1) : groups_number_(groups_number) {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  * \param joint_names list of joint-names for msg-publishing.
  *   - Count and order should match data from robot connection.
  *   - Use blank-name to exclude a joint from publishing.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 virtual bool init(SmplMsgConnection* connection,
                    std::vector<std::string> &joint_names);

  virtual bool init(SmplMsgConnection* connection,
                    std::map<int, RobotGroup> &robot_groups);

protected:
 int groups_number_;
 bool legacy_mode_;

 ros::Publisher pub_joint_control_state_;
 ros::Publisher dynamic_pub_joint_control_state_;
 ros::Publisher pub_joint_sensor_state_;

 /**
  * \brief Convert joint message into intermediate message-type
  *
  * \param[in] msg_in Message from robot connection
  * \param[out] joint_state JointTrajectoryPt message for intermediate processing
  */


 virtual bool convert_message(JointFeedbackMessage& msg_in, DynamicJointPoint* joint_state, int robot_id);

  // override JointRelayHandler::create_messages, to check robot_id w/o error msg
  bool create_messages(SimpleMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state);

  // override JointRelayHandler::create_messages, to check robot_id w/o error msg
  bool create_messages(JointFeedbackMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state, int robot_id);

private:

  static bool JointDataToVector(const industrial::joint_data::JointData &joints,
                                std::vector<double> &vec, int len);


  /**
   * \brief Convert joint feedback message into intermediate message-type
   *
   * \param[in] msg_in JointFeedbackMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointFeedbackExMessage& msg_in, DynamicJointPoint* joint_state, int robot_id);

};//class JointFeedbackExRelayHandler

}//joint_feedback_ex_relay_handler
}//industrial_robot_cliet

#endif // JOINT_FEEDBACK_EX_RELAY_HANDLER_H
