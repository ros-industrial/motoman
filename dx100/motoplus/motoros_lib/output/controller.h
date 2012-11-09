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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "motoPlus.h"
#include "ParameterExtraction.h"  // new motoPlus library
#include "joint_data.h"

namespace motoman
{
namespace controller
{

/**
 * \brief Structure for storing robot parameters
 */
typedef struct
{
  int ctrl_grp;
  int num_axes;
  float pulses_per_rad[MAX_PULSE_AXES];
  FB_AXIS_CORRECTION pulse_correction[MAX_PULSE_AXES];
} RobotParameters;


/**
 * \brief Spedifies the drive name for DRAM (This should be defined in motoPlus.h)
 */
#define MP_DRAM_DEV_DOS "MPRAM1:"  //This macro is supposed to be defined in motoPlus.h

/**
 * \brief Class encapsulates the motoman controller interface.  It also handles
 * higher level functions such as maintining the current motion state.  This class
 * is meant to be a Singleton, but that is not explicitly enforced.  Only one instance
 * of this object should be instantiated.
 */
//* Controller
/**
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE (the methods that simply wrap motoplus calls can
 * be considered thread safe).  Some internal class state data is maintained, which
 * is not thread safe.
 * TODO: This class will likely be shared between threads, it should be made thread
 * safe.
 *
 */

class Controller
{
public:

/**
  * \brief Default controller
  *
  */
 Controller();
 
 /**
  * \brief Destructor (disables motion and stops current job);
  *
  */
 ~Controller();
 
  /**
  * \brief Reads key values from parameter data-list.
  *
  * \param Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  *
  * \return true if parameters successfully read
  */
 static bool initParameters(int ctrl_grp);
 
 /**
  * \brief Read integer data from the controller integer data table.  Function
  * blocks until data is read
  *
  * \param index in data table
  *
  * \return integer value
  */
static int getInteger(int index);


/**
  * \brief Write integer data to the controller integer data table.  Function
  * blocks until data is written
  *
  * \param index in data table
  * \param value to write
  */
static void setInteger(int index, int value);

/**
  * \brief Write position-variable data to the controller data table.  Function
  * blocks until data is written
  *
  * \param index in data table
  * \param joint positions to write (ROS-order, radians)
  *
  * \return true if variable set successfully
  */
static bool setJointPositionVar(int index, industrial::joint_data::JointData ros_joints);

 /**
  * \brief Utility function for setting a digital output in the
  * Universal output data table (most IO is accessible there).
  *
  * \param bit offset in data table (0-2047)
  * \param in incoming message
  *
  */
 void setDigitalOut(int bit_offset, bool value);
 
 /**
  * \brief Utility function for waiting for a digital input
  * in the Universal input data tabel (most IO is accessible there).
  *
  * \param bit offset in data table (0-2047)
  * \param in incoming message
  *
  */
 void waitDigitalIn(int bit_offset, bool wait_value);
 
 /**
  * \brief Get the actual Joint Position from the robot encoders
  * NOTE: use getCmdJointPos to get the commanded joint positions
  *
  * \param array to hold joint positions (in radians)
  *
  * \return true if positions retrieved successfully
  */
 static bool getActJointPos(float* pos);
  
   /**
* \brief return true if motion is enabled (Based on internal class state
* not MotoPlus call)
*
* \return true if motion enabled
*/ 
bool isMotionEnabled(void) {return motionEnabled;};

   /**
* \brief return true if jos has been started (Based on internal class state
* not MotoPlus call)
*
* \return true if job has been stared
*/ 
bool isJobStarted(void) {return jobStarted;};
  
  /**
* \brief Enables motion on the robot controller.  Turns on servo power, turns
* off hold status
*
*/ 
void enableMotion(void);

  /**
* \brief Disables motion on the robot controller.  Turns off servo power, turns
* on hold status
*
*/ 
void disableMotion(void);

 /**
* \brief Starts motion job on the controller.  Enables motion (Job cannot be started
* if motion is not enabled).
*
*/ 
void startMotionJob(char* job_name);
	
 /**
* \brief Stops motion job on the controller.  Disables motion
*
*/ 
void stopMotionJob(char* job_name);

 /**
* \brief Stops motion job on the controller.  Disables motion
*
* \param # of ticks to delay
*/ 
void delayTicks(int ticks) { mpTaskDelay(ticks);};

 /**
  * \brief Utility function for writing a job file in temporary DRAM (the memory
  * supported by all controllers)
  *
  * \param full path and name of file to create
  * \param full job string
  *
  * \return true if file successfully opened
  */
 bool writeJob(char* path, char* job);
 
 /**
  * \brief Utility function for loading a job file from temporary DRAM (the memory
  * supported by all controllers)  WARNING: This function is limited to the DRAM
  * root directory.
  *
  * \param path of the file to load (pass "" if the root directory is used)
  * \param name of file to load (may not work with a full path)
  *
  * \return true if job successfully loaded
  */
 bool loadJob(char* path, char * job);
 
 /**
  * \brief Returns the velocity limit set for the controller.  This is stored within
  * the integer data table at VELOCITY_LIMIT_INDEX
  *
  * \return velocity limit (%)
  */
 double getVelocityLimit()
    {return (double) this->getInteger(VELOCITY_LIMIT_INDEX);};


  /**
  * \brief Reads the number of robot axes from the controller's config parameters.
  *
  * \param Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param Number of robot axes (return)
  *
  * \return true if parameters successfully read
  */static bool getNumRobotAxes(int ctrl_grp, int* numAxes);

 /**
  * \brief Reads the pulse-per-radian scaling factors from the controller's
  * config parameters, based on the arm's gearing ratios.
  *    jntPosInRadians = jntPosInPulses * pulseToRadian
  *
  * \param Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param array of scaling factors (return, in Motoman order, length=MAX_PULSE_AXES)
  *
  * \return true if parameters successfully read
  */
 static bool getPulsesPerRadian(int ctrl_grp, float* pulse_to_radian);

 /**
  * \brief Reads the pulse correction factors from the controller's
  * config parameters, based on physical axis cross-coupling.
  *    pulsePos[ulCorrectionAxis] -= pulsePos[ulSourceAxis] * fCorrectionRatio
  *
  * \param Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param array of correction factors (return, length=MAX_PULSE_AXES)
  *
  * \return true if parameters successfully read
  */
 static bool getFBPulseCorrection(int ctrl_grp, FB_AXIS_CORRECTION* pulse_correction);
 
protected:

/**
* \brief Index within integer data table that holds velocity limit
*/
static const int VELOCITY_LIMIT_INDEX = 94;
	        
 /**
  * \brief Typical MP function call return on error
  */
 static const int MP_ERROR = -1;
 
 /**
  * \brief Typical MP function call return on OK (sometimes a value
  * greater than zero is also returned if it has some meaning, like
  * a file descriptor.
  */
 static const int MP_OK = 0;

  /**
  * \brief Poll delay (in ticks) when querying the motoplus api.
  */
 static const int VAR_POLL_DELAY_ = 10; //ms
 static const int UNIV_IN_DATA_START_ = 10;
 static const int UNIV_OUT_DATA_START_ = 10010;
 static const int UNIV_IO_DATA_SIZE_ = 2048;
 
 // Servo power variables
MP_SERVO_POWER_SEND_DATA servo_power_data;
MP_STD_RSP_DATA servo_power_error;

// Job variables
MP_START_JOB_SEND_DATA job_start_data;
MP_DELETE_JOB_SEND_DATA job_delete_data;
MP_STD_RSP_DATA job_error;


// Hold variables
MP_HOLD_SEND_DATA hold_data;
MP_STD_RSP_DATA hold_error;

 //TODO: motion and job flags are just internal state variables, we may
 //want to make them query the appropriate motoplus API calls instead.
 /**
  * \brief True if motion enabled
  */
 bool motionEnabled;
 
 /**
  * \brief True if job started
  */
 bool jobStarted;
};



} //controller
} //motoman

#endif //CONTROLLER_H