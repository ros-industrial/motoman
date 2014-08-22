#ifndef JOINT_TRAJ_PT_FULL_EX_H
#define JOINT_TRAJ_PT_FULL_EX_H

#ifndef FLATHEADERS
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_traj_pt_full.h"
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

#include<vector>
namespace industrial
{
namespace joint_traj_pt_full_ex
{

namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  START_TRAJECTORY_DOWNLOAD = -1, START_TRAJECOTRY_STREAMING = -2, END_TRAJECTORY = -3, STOP_TRAJECTORY = -4
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt class, but this
 * class provides the full message contents directly to the robot controller,
 * rather than simplifying the velocity duration.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   num_groups          (industrial::shared_types::shared_int)    4  bytes
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   multiJointTrajData  JointTrajPtData[]
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtFullEx : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointTrajPtFullEx(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtFullEx(void);

  /**
   * \brief Initializes a empty joint trajectory group
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory group
   *
   */
  void init(industrial::shared_types::shared_int num_groups,
            industrial::shared_types::shared_int sequence,
            std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points);

  /**
   * \brief Sets robot_id.
   *        Robot group # (0-based), for controllers with multiple axis-groups.
   *
   * \param robot_id new robot_id value
   */
  void setNumGroups(industrial::shared_types::shared_int num_groups)
  {
    this->num_groups_ = num_groups;
  }

  /**
   * \brief Gets robot_id.
   *        Robot group # (0-based), for controllers with multiple axis-groups.
   *
   * @return robot_id value
   */
  industrial::shared_types::shared_int getNumGroups()
  {
    return this->num_groups_;
  }

  void setMultiJointTrajPtData( std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points)
  {
        this->joint_trajectory_points_ = joint_trajectory_points;
  }

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */

  industrial::shared_types::shared_int get_max_groups()
  {
      return MAX_NUM_GROUPS;
  }

  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointTrajPtFullEx &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointTrajPtFullEx &rhs);


  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_int) + MAX_NUM_GROUPS*(this->joint_traj_full_sample_.byteLength()-sizeof(industrial::shared_types::shared_int));
  }

private:

   std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points_;

   industrial::joint_traj_pt_full::JointTrajPtFull joint_traj_full_sample_;
  /**
   * \brief robot group # (0-based) for controllers that support multiple axis-groups
   */
  industrial::shared_types::shared_int num_groups_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;
  /**
   * \brief bit-mask of (optional) fields that have been initialized with valid data
   * \see enum ValidFieldTypes
   */

  static const industrial::shared_types::shared_int MAX_NUM_GROUPS = 4;

};

}
}

#endif // JOINT_TRAJ_PT_FULL_EX_H
