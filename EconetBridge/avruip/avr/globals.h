
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
    uint8_t Safety; 
    uint8_t Initialised; 
    uint8_t MachineLow; 
    uint8_t MachineHigh; 
    uint8_t ClockMultiplier; 
    uint8_t Econet_Network; 
    uint8_t Station; 
    uint8_t Ethernet_Network; 
    uint8_t IPAddr_1; 
    uint8_t IPAddr_2; 
    uint8_t IPAddr_3; 
    uint8_t IPAddr_4; 
    uint8_t Subnet_1; 
    uint8_t Subnet_2; 
    uint8_t Subnet_3; 
    uint8_t Subnet_4; 
    uint8_t Gateway_1; 
    uint8_t Gateway_2; 
    uint8_t Gateway_3; 
    uint8_t Gateway_4; 
    uint8_t MAC_1;
    uint8_t MAC_2; 
    uint8_t MAC_3; 
    uint8_t MAC_4; 
    uint8_t MAC_5; 
    uint8_t MAC_6; 
} sDefaults_t; 


extern sDefaults_t eeGlobals; 




#endif /* __GLOBALS_H__ */

