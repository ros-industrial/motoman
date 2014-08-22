#ifndef JOINT_FEEDBACK_EX_H
#define JOINT_FEEDBACK_EX_H

#ifndef FLATHEADERS
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_feedback.h"
#include "simple_message/messages/joint_feedback_message.h"
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "joint_feedback.h"
#include "messages/joint_feedback_message.h"
#endif

#include<vector>

namespace industrial
{
namespace joint_feedback_ex
{

class JointFeedbackEx : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointFeedbackEx(void);
  /**
   * \brief Destructor
   *
   */
  ~JointFeedbackEx(void);

  /**
   * \brief Initializes a empty joint feedback
   *
   */
  void init();

  /**
   * \brief Initializes a complete joint feedback
   *
   */
  void init(industrial::shared_types::shared_int groups_number,
            std::vector<industrial::joint_feedback_message::JointFeedbackMessage> joints_feedback_points );

  /**
   * \brief Sets robot_id.
   *        Robot group # (0-based), for controllers with multiple axis-groups.
   *
   * \param robot_id new robot_id value
   */
  void setGroupsNumber(industrial::shared_types::shared_int groups_number)
  {
    this->groups_number_ = groups_number;
  }

  void setJointMessages(std::vector<industrial::joint_feedback_message::JointFeedbackMessage> joint_feedback_messages)
  {
    this->joint_feedback_messages_ = joint_feedback_messages;
  }

  std::vector<industrial::joint_feedback_message::JointFeedbackMessage> getJointMessages()
  {
      return joint_feedback_messages_;
  }

  /**
   * \brief Gets robot_id.
   *        Robot group # (0-based), for controllers with multiple axis-groups.
   *
   * @return robot_id value
   */
  industrial::shared_types::shared_int getGroupsNumber()
  {
    return this->groups_number_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointFeedbackEx &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointFeedbackEx &rhs);

  /**
   * \brief check the validity state for a given field
   * @param field field to check
   * @return true if specified field contains valid data
   */

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + MAX_NUM_GROUPS*(2*sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_real)
        + 3*(this->positions_.byteLength()));
  }

private:

  /**
   * \brief robot group # (0-based) for controllers that support multiple axis-groups
   */
  industrial::shared_types::shared_int groups_number_;

  std::vector<industrial::joint_feedback_message::JointFeedbackMessage> joint_feedback_messages_;

  industrial::joint_data::JointData positions_;

  static const industrial::shared_types::shared_int MAX_NUM_GROUPS = 4;

};

}
}


#endif // JOINT_FEEDBACK_EX_H
