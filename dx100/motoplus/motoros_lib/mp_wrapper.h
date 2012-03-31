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

#ifndef MP_WRAPPER_H
#define MP_WRAPPER_H

/*
DEPRECATED - The utilities in this file should not be used.  The controller class
in controller.h should be used instead.

The mp wrapper source contains functions for wrapping common MP calls.  The MP
API provides a lot of flexibility that results in a lot of repeated code for
simple operations, like reading a value out of the integer data table
*/

#include "joint_data.h"
#include "motoPlus.h"

using namespace industrial::joint_data;

namespace motoman
{
namespace mp_wrapper
{

 /**
   * \brief Number of ticks to delay between 
   */
const int VAR_POLL_TICK_DELAY = 10;


/**
  * \brief Read integer data from the controller integer data table.  Function
  * blocks until data is read
  *
  * \param index in data table
  *
  * \return integer value
  */
int getInteger(int index);


/**
  * \brief Write integer data to the controller integer data table.  Function
  * blocks until data is written
  *
  * \param index in data table
  * \param value to write
  */
void setInteger(int index, int value);

void getMotomanFbPos(industrial::joint_data::JointData & pos);
void getRosFbPos(industrial::joint_data::JointData & pos);

void toJointData(MP_FB_PULSE_POS_RSP_DATA & src, 
    industrial::joint_data::JointData & dest);
    
void toMpPosVarData(USHORT posVarIndex, industrial::joint_data::JointData & src, 
    MP_POSVAR_DATA & dest);



} //mp_wrapper
} //motoman

#endif //MP_WRAPPER_H