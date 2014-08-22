#ifndef FLATHEADERS
#include "motoman_driver/simple_message/messages/joint_traj_pt_full_ex_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include "motoman_driver/simple_message/motoman_simple_message.h"
#else
#include "joint_traj_pt_full_ex_message.h"
#include "joint_data.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace motoman::simple_message;
using namespace industrial::joint_traj_pt_full_ex;

namespace industrial
{
namespace joint_traj_pt_full_ex_message
{

JointTrajPtFullExMessage::JointTrajPtFullExMessage(void)
{
  this->init();
}

JointTrajPtFullExMessage::~JointTrajPtFullExMessage(void)
{

}

bool JointTrajPtFullExMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (data.unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

void JointTrajPtFullExMessage::init(industrial::joint_traj_pt_full_ex::JointTrajPtFullEx & point)
{
  this->init();
  this->point_.copyFrom(point);
}

void JointTrajPtFullExMessage::init()
{
  this->setMessageType(MotomanMsgTypes::ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX);
  this->point_.init();
}


bool JointTrajPtFullExMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj. pt. message load");
  if (buffer->load(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint traj. pt data");
  }
  return rtn;
}

bool JointTrajPtFullExMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj pt message unload");

  if (buffer->unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

}
}

