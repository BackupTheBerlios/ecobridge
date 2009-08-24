
//  PUBLIC MODULE VARIABLE DEFINITIONS

// Variables filled from (and some written to) EEPROM 

#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define MACHINE_VER_LOW         0x01
#define MACHINE_VER_HIGH        0x00

typedef unsigned char	uint8_t;
typedef unsigned char	u_char;
typedef unsigned int	u_int;
typedef unsigned short	u_short;
typedef unsigned long	u_long;


typedef struct { 
  uint8_t Safety;		// ignore, can be corrupted at runtime
  uint8_t Initialised;		// set to 0x55 to indicate eeprom programmed
  uint8_t MachineLow; 
  uint8_t MachineHigh; 
  uint8_t ClockMultiplier;	// divider ratio for onboard clock generator, 0 to disable
  uint8_t Econet_Network;	// network number for econet side
  uint8_t Station;		// station number for local econet
  uint8_t Ethernet_Network;	// network number for ethernet side. currently unused
  uint8_t IPAddr_1;		// local IP address
  uint8_t IPAddr_2; 
  uint8_t IPAddr_3; 
  uint8_t IPAddr_4; 
  uint8_t Subnet_1;		// subnet mask for local network: 255, 255, 0, 0 for AUN
  uint8_t Subnet_2; 
  uint8_t Subnet_3; 
  uint8_t Subnet_4; 
  uint8_t Gateway_1;		// default router
  uint8_t Gateway_2; 
  uint8_t Gateway_3; 
  uint8_t Gateway_4; 
  uint8_t MAC[6];		// MAC address for this unit
  uint8_t WANRouter[4];		// IP address for WAN-mode router
  uint8_t EconetIP[4];		// IP address for Econet-side interface
  uint8_t EconetMask[4];	// subnet mask for Econet side
} sDefaults_t; 


extern sDefaults_t eeGlobals; 




#endif /* __GLOBALS_H__ */

