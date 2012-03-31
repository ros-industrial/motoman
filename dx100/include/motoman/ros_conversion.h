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

#ifndef ROS_CONVERSION_H
#define ROS_CONVERSION_H

#ifdef ROS
#include "simple_message/joint_data.h"
#endif

#ifdef MOTOPLUS
#include "joint_data.h"
#endif

namespace motoman
{
namespace ros_conversion
{

/**
 * \brief Enumeration of motoman joint indicies (as returned by the getPos
 * library calls.
 */
namespace MotomanJointIndexes
{
enum MotomanJointIndex
{
  S = 0, 
  L = 1, 
  U = 2,
  R = 3,
  B = 4,
  T = 5,
  E = 6,
  COUNT = E
};
}
typedef MotomanJointIndexes::MotomanJointIndex MotomanJointIndex;

/**
 * \brief Enumeration of ROS joint indicies.  In reality these are not
 * fixed.  The ROS message structure maps joints by name and theoretically
 * would allow for any order.  Generally the order from base to tip is 
 * maintained (This is what the enumeration assumes).
 */
namespace RosJointIndexes
{
enum RosJointIndex
{
  S = 0, 
  L = 1, 
  E = 2,  
  U = 3,
  R = 4,
  B = 5,
  T = 6,
  COUNT = T
};
}
typedef RosJointIndexes::RosJointIndex RosJointIndex;


/**
 * \brief Enumeration of Motoman robot types (models).  Initially these are
 * being used to initialize joint rads->count conversion factors.
 */
namespace MotomanRobotModels
{
enum MotomanRobotModel
{
  SIA_10D,
  SIA_20D
};
}
typedef MotomanRobotModels::MotomanRobotModel MotomanRobotModel;

/**
 * \brief Initializes joint rads->enocder count conversion factors
 * This function must be manually called from within mpUsrRoot.
 * Failure to call this function results in conversion factors of
 * zero.
 *
 * \param robot model (@see MotomanRobotModels)
 */
void initJointConversion(MotomanRobotModel model_number);

//TODO: Standardize function calls such that JointDatas are always in ROS
//      order and MP types are always in motoman order.  The order of the
//      JointData type can change within a function, but when a variable
//      of that type enters or leaves a function is should always be in ROS
//      order.  Move the appropriate function prototypes below to the src
//      file in order to hide the ordering details (essentially make them private)
float toPulses(float radians, MotomanJointIndex joint);
float toRadians(float pulses, MotomanJointIndex joint);


/**
 * \brief Converts a motoplus joint (in motoplus order and pulses) to
 * a ros joint (in ros order and radians)
 *
 * \param motoplus joints to convert
 * \param ros joint (returned)
 */
void toRosJoint(industrial::joint_data::JointData & mp_joints, 
				industrial::joint_data::JointData & ros_joints);

/**
 * \brief Converts a ros joint (in ros order and radians) to
 * a motoplus joint (in motoplus order and pulses)
 * WARNING: This function should only be used at the lowest levels.
 * The assumption is that any JointData type is assumed to be a ROS
 * Joint.
 *
 * \param ros joint (returned)
 * \param motoplus joints to convert
 */
void toMpJoint(industrial::joint_data::JointData & ros_joints,
				industrial::joint_data::JointData & mp_joints);
				
				
				
//DEPRECATED
void toMotomanJointOrder(industrial::joint_data::JointData & joints);
// These functions change the JointData in place which has lead to some dangerous
// bugs when they are called more than once.  Use the functions above
// that swap joint order and perform unit conversion automatically
//DEPRECATED
void toRosJointOrder(industrial::joint_data::JointData & joints);
//DEPRECATED
void toMotomanJointOrder(industrial::joint_data::JointData & joints);
    
    
} //ros_conversion
} //motoman


#endif //ROS_CONVERSION_H
