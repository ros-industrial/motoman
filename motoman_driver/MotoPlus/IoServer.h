// IoServer.h
//
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Yaskawa America, Inc.
* Copyright (c) 2017, Delft University of Technology
* All rights reserved.
*
* Redistribution and use in binary form, with or without modification,
* is permitted provided that the following conditions are met:
*
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Yaskawa America, Inc., nor the names 
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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

#ifndef IOSERVER_H
#define IOSERVER_H

extern void Ros_IoServer_StartNewConnection(Controller* controller, int sd);
extern void Ros_IoServer_StopConnection(Controller* controller, int connectionIndex);

extern void Ros_IoServer_WaitForSimpleMsg(Controller* controller, int connectionIndex);
extern BOOL Ros_IoServer_SimpleMsgProcess(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);

extern int Ros_IoServer_ReadIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
extern int Ros_IoServer_WriteIOBit(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
extern int Ros_IoServer_ReadIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
extern int Ros_IoServer_WriteIOGroup(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
extern int Ros_IoServer_ReadIORegister(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
extern int Ros_IoServer_WriteIORegister(SimpleMsg* receiveMsg, SimpleMsg* replyMsg);

typedef enum
{
	IO_ACCESS_BIT,
	IO_ACCESS_GROUP,
	IO_ACCESS_REGISTER
} IoAccessSize;

extern BOOL Ros_IoServer_IsValidReadAddress(UINT32 address, IoAccessSize size);
extern BOOL Ros_IoServer_IsValidWriteAddress(UINT32 address, IoAccessSize size);
extern BOOL Ros_IoServer_IsValidWriteValue(UINT32 value, IoAccessSize size);

//**********************************************************
#if DX100
#define GENERALINMIN (10)
#define GENERALINMAX (2567)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (12567)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (22567)

#define NETWORKINMIN (25010)
#define NETWORKINMAX (27567)

#define NETWORKOUTMIN (35010)
#define NETWORKOUTMAX (37567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (32567)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (41607)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (52007)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (80647)

#define PSEUDOINPUTMIN (82010)
#define PSEUDOINPUTMAX (82207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif FS100
#define GENERALINMIN (10)
#define GENERALINMAX (1287)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (11287)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (21287)

#define NETWORKINMIN (25010)
#define NETWORKINMAX (26287)

#define NETWORKOUTMIN (35010)
#define NETWORKOUTMAX (36287)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (31287)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (41607)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (52007)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (80647)

#define PSEUDOINPUTMIN (82010)
#define PSEUDOINPUTMAX (82207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif DX200
#define GENERALINMIN (10)
#define GENERALINMAX (5127)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (15127)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (25127)

#define NETWORKINMIN (27010)
#define NETWORKINMAX (29567)

#define NETWORKOUTMIN (37010)
#define NETWORKOUTMAX (39567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (35127)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (41607)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (53007)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (82007)

#define PSEUDOINPUTMIN (82010)
#define PSEUDOINPUTMAX (82207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif YRC1000
#define GENERALINMIN (10)
#define GENERALINMAX (5127)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (15127)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (25127)

#define NETWORKINMIN (27010)
#define NETWORKINMAX (29567)

#define NETWORKOUTMIN (37010)
#define NETWORKOUTMAX (39567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (35127)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (42567)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (55127)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (85127)

#define PSEUDOINPUTMIN (87010)
#define PSEUDOINPUTMAX (87207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif YRC1000u
#define GENERALINMIN (10)
#define GENERALINMAX (5127)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (15127)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (21287)

#define NETWORKINMIN (27010)
#define NETWORKINMAX (29567)

#define NETWORKOUTMIN (37010)
#define NETWORKOUTMAX (39567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (31287)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (42567)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (55127)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (85127)

#define PSEUDOINPUTMIN (87010)
#define PSEUDOINPUTMAX (87207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)


#endif

#define QUANTITY_BIT	(1)
#define QUANTITY_BYTE	(8)

#endif
