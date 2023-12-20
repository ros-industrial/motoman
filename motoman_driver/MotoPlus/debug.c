//debug.c
//

#include "motoPlus.h"

#define MAX_DEBUG_MESSAGE_SIZE	1024

#if (YRC1000||YRC1000u)

#define DEBUG_UDP_PORT_NUMBER	21789
#define	SO_BROADCAST	0x0020

#define MP_USER_LAN1	1	/* general LAN interface1 */
#define MP_USER_LAN2	2	/* general LAN interface2(only YRC1000) */

extern STATUS setsockopt(int s, int level, int optname, char* optval, int optlen);
extern int mpNICData(USHORT if_no, ULONG* ip_addr, ULONG* subnet_mask, UCHAR* mac_addr, ULONG* default_gw);

int ros_DebugSocket = -1;
struct sockaddr_in ros_debug_destAddr1;

void Ros_Debug_Init()
{
	ULONG ip_be;
	ULONG subnetmask_be;
	ULONG gateway_be;
	int broadcastVal = 1;
	UCHAR mac[6];

	ros_DebugSocket = mpSocket(AF_INET, SOCK_DGRAM, 0);
	setsockopt(ros_DebugSocket, SOL_SOCKET, SO_BROADCAST, (char*)&broadcastVal, sizeof(broadcastVal));

	mpNICData(MP_USER_LAN1, &ip_be, &subnetmask_be, mac, &gateway_be);

	ros_debug_destAddr1.sin_addr.s_addr = ip_be | (~subnetmask_be);
	ros_debug_destAddr1.sin_family = AF_INET;
	ros_debug_destAddr1.sin_port = mpHtons(DEBUG_UDP_PORT_NUMBER);
}

void Debug_BroadcastBytes(char* bytes, int len)
{
	if (ros_DebugSocket == -1)
		Ros_Debug_Init();

	mpSendTo(ros_DebugSocket, bytes, len, 0, (struct sockaddr*)&ros_debug_destAddr1, sizeof(struct sockaddr_in));
}
#endif


void Debug_BroadcastMsg(const char *fmt, ...)
{
#if defined(YRC1000)||defined(YRC1000u)
	char str[MAX_DEBUG_MESSAGE_SIZE];
	va_list va;

	memset(str, 0x00, MAX_DEBUG_MESSAGE_SIZE);

	va_start(va, fmt);
	vsnprintf(str, MAX_DEBUG_MESSAGE_SIZE, fmt, va);
	va_end(va);

	if (ros_DebugSocket == -1)
		Ros_Debug_Init();

	mpSendTo(ros_DebugSocket, str, strlen(str), 0, (struct sockaddr*)&ros_debug_destAddr1, sizeof(struct sockaddr_in));
#else
	// Broadcast not available, just print to terminal
	va_list va;
	va_start(va, fmt);
	printf(fmt, va);
	va_end(va);
#endif
}


