#ifndef FLATHEADERS
#include "motoman_driver/simple_message/messages/joint_feedback_ex_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include "motoman_driver/simple_message/motoman_simple_message.h"
#else
#include "joint_feedback_ex_message.h"
#include "joint_data.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace motoman::simple_message;
using namespace industrial::joint_feedback;
using namespace industrial::joint_feedback_ex;

namespace industrial
{
namespace joint_feedback_ex_message
{

JointFeedbackExMessage::JointFeedbackExMessage(void)
{
  this->init();
}

JointFeedbackExMessage::~JointFeedbackExMessage(void)
{

}

bool JointFeedbackExMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (data.unload(this->data_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload joint feedback message data");
  }
  return rtn;
}

void JointFeedbackExMessage::init(industrial::joint_feedback_ex::JointFeedbackEx & data)
{
  this->init();
  this->data_.copyFrom(data);
}

void JointFeedbackExMessage::init()
{
  this->setMessageType(MotomanMsgTypes::ROS_MSG_MOTO_JOINT_FEEDBACK_EX);
  this->data_.init();
}

bool JointFeedbackExMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint feedback message load");
  if (buffer->load(this->data_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint feedback message data");
  }
  return rtn;
}

bool JointFeedbackExMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint feedback message unload");

  if (buffer->unload(this->data_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint feedback message data");
  }
  return rtn;
}

}
}


