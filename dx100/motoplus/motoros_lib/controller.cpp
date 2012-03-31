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
 
 #include "controller.h"
 #include "log_wrapper.h"
 
 namespace motoman
{
namespace controller
{

Controller::Controller()
{
  this->jobStarted = false;
  this->motionEnabled = false;
}

Controller::~Controller()
{
  this->disableMotion();
  // TODO: The current job should probably be unloaded.
}

void Controller::setInteger(int index, int value)
{
	MP_VAR_DATA data;
	
	data.usType = MP_RESTYPE_VAR_I;
	data.usIndex = index;
	data.ulValue = value;
	
	while (mpPutVarData ( &data, 1 ) == MP_ERROR) 
	{
        LOG_ERROR("Failed to set integer varaible, index: %d, value: %d, retrying...", 
            data.usIndex, data.ulValue);
        mpTaskDelay(VAR_POLL_DELAY_);
    }
}

int Controller::getInteger(int index)
{
    
	MP_VAR_INFO info;
	LONG rtn;
	
	info.usType = MP_RESTYPE_VAR_I;
	info.usIndex = index;
	
	while (mpGetVarData ( &info, &rtn, 1 ) == MP_ERROR) 
	{
        LOG_ERROR("Failed to retreive integer variable index: %d, retrying...", info.usIndex);
        mpTaskDelay(VAR_POLL_DELAY_);
    }
    return rtn;
}

void Controller::enableMotion(void)
{
  
  LOG_INFO("Enabling motion");
  this->motionEnabled = false;

  servo_power_data.sServoPower = ON;
  while(mpSetServoPower(&servo_power_data, &servo_power_error) == MP_ERROR)
  {
    LOG_ERROR("Failed to turn on servo power, error: %d, retrying...", servo_power_error.err_no);
    mpTaskDelay(this->VAR_POLL_DELAY_);
  };
  
  hold_data.sHold = OFF;
  while(mpHold(&hold_data, &hold_error) == MP_ERROR)
  {
    LOG_ERROR("Failed to turn off hold, error: %d, retrying...", hold_error.err_no);
    mpTaskDelay(this->VAR_POLL_DELAY_);
  };
  
  this->motionEnabled = true;
}


void Controller::disableMotion(void)
{
  LOG_INFO("Disabling motion");
  servo_power_data.sServoPower = OFF;
  while(mpSetServoPower(&servo_power_data, &servo_power_error) == MP_ERROR)
  {
    LOG_ERROR("Failed to turn off servo power, error: %d, retrying...", servo_power_error.err_no);
    mpTaskDelay(this->VAR_POLL_DELAY_);
  };
  
  this->motionEnabled = false;
}

void Controller::startMotionJob(char* job_name)
{

  this->jobStarted = false;
  
  this->enableMotion();
  
  // Set up job variables
  job_start_data.sTaskNo = 0;
  strcpy(job_start_data.cJobName, job_name);
 
  
  LOG_INFO("Starting motion job");
  while(mpStartJob(&job_start_data, &job_error) == ERROR)
  {
    LOG_ERROR("Failed to start job, error: %d, retrying...", job_error.err_no);
    mpTaskDelay(this->VAR_POLL_DELAY_);
  }; 
  
  this->jobStarted = true;
}


void Controller::stopMotionJob(char* job_name)
{  
  LOG_INFO("Stopping motion job");
  this->disableMotion();
  
  // delete task
  strcpy(job_delete_data.cJobName, job_name);
  
  while(mpDeleteJob(&job_delete_data, &job_error) == MP_ERROR)
  {
    LOG_ERROR("Failed to delete job, error: %d, retrying...", job_error.err_no);
    mpTaskDelay(this->VAR_POLL_DELAY_);
  };
  
  this->jobStarted = false;
}	


#define FILE_NAM_BUFFER_SIZE 100
bool Controller::writeJob(char* path, char* job)
{
    bool rtn = false;
    int fd = this->MP_ERROR;
    int status = this->MP_ERROR;
    char filename[FILE_NAM_BUFFER_SIZE];  //should be big enough to hold a file name and DRAM drive name
    
    memset(filename, '\0', FILE_NAM_BUFFER_SIZE);  //not sure this is needed, strcpy below also does this.
    strcpy(filename, "MPRAM1:\\");
    strcat(filename, path);
    
    LOG_DEBUG("writeJob: %s", filename);
    
    // Remove the file, if it exists
    LOG_DEBUG("Trying to remove file, if it exists");
    status = mpRemove( filename );
    if (this->MP_ERROR == status)
    {
        LOG_WARN("Failed to remove job file: %s", filename);
    }
    
    
    // Create the file and write the job
    fd = mpCreate( filename, O_WRONLY );
    if (this->MP_ERROR != fd)
    {
        status = mpWrite( fd, job, strlen(job) );
        if ( this->MP_ERROR != status )
        {
            LOG_INFO("Successfully loaded file: %s, bytes written: %d", filename, status);
            rtn = true;
        }
        else
        {
            LOG_ERROR("Failed to write file: %s", filename);
            rtn = false;
        }
        
        
        // Checking file status
        /*
	    struct stat pStat;
	    status = mpFstat(fd, &pStat);
	    {
			LOG_DEBUG("mpFstat Complete");
			LOG_DEBUG("st_dev = %u", pStat.st_dev);
			LOG_DEBUG("st_ino = %u", pStat.st_ino);
			LOG_DEBUG("st_mode = %u", pStat.st_mode);
			LOG_DEBUG("st_nlink = %d", pStat.st_nlink);
			LOG_DEBUG("st_uid = %d", pStat.st_uid);
			LOG_DEBUG("st_gid = %d", pStat.st_gid);
			LOG_DEBUG("st_rdev = %u", pStat.st_rdev);
			LOG_DEBUG("st_size = %u", pStat.st_size);
			LOG_DEBUG("st_atime = %u", pStat.st_atime);
			LOG_DEBUG("st_mtime = %u", pStat.st_mtime);
			LOG_DEBUG("st_ctime = %u", pStat.st_ctime);
			LOG_DEBUG("st_blksize = %u", pStat.st_blksize);
			LOG_DEBUG("st_blocks = %u", pStat.st_blocks);
			LOG_DEBUG("st_attrib = %u", pStat.st_attrib);
		}
		*/
		
        // close file descriptor
	    status = mpClose(fd);
	    if (this->MP_ERROR == status)
	    {
	        LOG_WARN("Failed to close file: %s, ignoring failure", filename);
	    }
    }
    else
    {
        LOG_ERROR("Failed to create job file: %s", filename);
        rtn = false;
    }
    
    
    return rtn;
}
#undef FILE_NAM_BUFFER_SIZE


bool Controller::loadJob(char* path, char * job)
{
    bool rtn = false;
    int status;    
    
    LOG_DEBUG("Refreshing file list");
    status = mpRefreshFileList(MP_EXT_ID_JBI);
    if (this->MP_OK != status)
    {
    	LOG_WARN("Failed to refresh file list: %d, ignoring failure", status);
    }
    LOG_DEBUG("File count before file load: %d", mpGetFileCount());
    
    LOG_DEBUG("Attempting to load file, path: %s, job: %s", path, job);
    status = mpLoadFile (MP_DRV_ID_DRAM, path, job );
    if (this->MP_OK == status)
    {
        LOG_INFO("Loaded job file %s", job);
        rtn = true;
    }
    else
    {
        LOG_ERROR("Failed to load job file: %s, path: %s, returned error code: %d",
                    job, path, status);
        rtn = false;
    }
    
    LOG_DEBUG("Refreshing file list");
    status = mpRefreshFileList(MP_EXT_ID_JBI);
    if (this->MP_OK != status)
    {
    	LOG_WARN("Failed to refresh file list: %d, ignoring failure", status);
    }
    LOG_DEBUG("File count after file load: %d", mpGetFileCount());
    
    return rtn;

}


void Controller::setDigitalOut(int bit_offset, bool value)
{
  LOG_DEBUG("Setting digital out, Bit offset: %d, value: %d", bit_offset, value);
  if ( (bit_offset < this->UNIV_IO_DATA_SIZE_) && 
       ( bit_offset > 0) )
  {  
    MP_IO_DATA data;
    data.ulAddr = this->UNIV_OUT_DATA_START_ + bit_offset;
    data.ulValue = value;
    //TODO: The return result of mpWriteIO is not checked
    mpWriteIO(&data, 1);
  }
  else
  {
    LOG_ERROR("Bit offset: %d, is greater than size: %d", bit_offset, this->UNIV_IO_DATA_SIZE_);
  }
}

 void Controller::waitDigitalIn(int bit_offset, bool wait_value)
 {
   LOG_DEBUG("Waiting for digital in, Bit offset: %d, Wait value: %d", bit_offset, wait_value);
   if ( (bit_offset < this->UNIV_IO_DATA_SIZE_) && 
       ( bit_offset > 0) )
  { 
    MP_IO_INFO info;
    info.ulAddr = this->UNIV_IN_DATA_START_ + bit_offset;
    
    USHORT readValue;
    do
    {
      readValue = !wait_value;  
      //TODO: The return result of mpReadIO is not checked
      mpReadIO (&info, &readValue, 1);
      mpTaskDelay(VAR_POLL_DELAY_);
    } while ( ((bool)readValue) != wait_value);
  }
  else
  {
    LOG_ERROR("Bit offset: %d, is greater than size: %d", bit_offset, this->UNIV_IO_DATA_SIZE_);
  }
 }



} //controller
} //motoman