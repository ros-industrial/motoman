//debug.h
//
#ifndef _DEBUG_H_
#define _DEBUG_H_

#if (YRC1000||YRC1000u)
extern void Ros_Debug_Init();
#endif
extern void Debug_BroadcastMsg(const char *fmt, ...);

#endif