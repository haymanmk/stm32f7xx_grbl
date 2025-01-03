#ifndef __ETHERNET_IF_H__
#define __ETHERNET_IF_H__

#include "FreeRTOS_Routing.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Disable the unused-variable warning, start from here */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

static uint8_t MACAddr[6] = {0x00, 0x80, 0xE1, 0x01, 0x02, 0x03};

/**
 * Define the network addressing.These parameters will be used if either
 * ipconfigUSE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration
 * failed.
 */
// static uint8_t IPAddr[4] = {172, 16, 0, 10};
static uint8_t NetMask[4] = {255, 240, 0, 0};
// static uint8_t GatewayAddr[4] = {172, 16, 0, 1};
static uint8_t DNSAddr[4] = {8, 8, 8, 8};

#pragma GCC diagnostic pop
/* End of Disable unused-variable warning */

/* Listening Port */
#define LISTENING_PORT 8500

BaseType_t tcp_server_init();
void vStartSimpleTCPServerTasks(uint16_t usStackSize, UBaseType_t uxPriority);

#endif // __ETHERNET_IF_H__