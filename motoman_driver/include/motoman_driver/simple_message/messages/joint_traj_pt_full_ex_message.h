#ifndef JOINT_TRAJ_PT_FULL_EX_MESSAGE_H
#define JOINT_TRAJ_PT_FULL_EX_MESSAGE_H

#ifndef FLATHEADERS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
#include "motoman_driver/simple_message/joint_traj_pt_full_ex.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "joint_traj_pt_full_ex.h"
#endif

namespace industrial
{
namespace joint_traj_pt_full_ex_message
{


/**
 * \brief Class encapsulated joint trajectory point message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::joint_traj_pt_full_ex::JointTrajPtFull data type.
 * The data portion of this typed message matches JointTrajPtFull.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtFullExMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointTrajPtFullExMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtFullExMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint trajectory point structure
   *
   * \param joint trajectory point data structure
   *
   */
  void init(industrial::joint_traj_pt_full_ex::JointTrajPtFullEx & point);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->point_.byteLength();
  }

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence) { point_.setSequence(sequence); }

  industrial::joint_traj_pt_full_ex::JointTrajPtFullEx point_;

private:


};

}
}

#endif /* JOINT_TRAJ_PR_FULL_EX_MESSAGE_H */
