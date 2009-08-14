
#include <avr/eeprom.h>
#include "globals.h"
#include "eeprom.h"


//sDefaults_t INIT EEMEM =
sDefaults_t INIT =
		{0xAA, EEPROM_VALID, MACHINE_VER_HIGH,MACHINE_VER_LOW,	/* Safety, Initted, VER HIGH, VER LOW */ \
		0, 1, 0x51, 130,				/* Clock, Econet Net, Station, Ethernet Net */\
		1, 2, 128, 10,					/* IP Address */\
		255,255, 0, 0,					/* Subnet Mask */\
		1,2, 128, 9,					/* Default Router */ \
		 { 0x00,0x00, 0xA4,				/* MAC Address 1-3 OUI Acorn */	\
		   0x33, 0x44, 0x55 }};				/* MAC Address 4-6 NUI */




static inline void EEPROM_InitCheck(void)
{

    eeGlobals.Initialised = eeprom_read_byte(EEPROM_INITTED);

    if (eeGlobals.Initialised != EEPROM_VALID) {
	    eeprom_write_block(&INIT, EEPROM_SAFETY, sizeof(INIT));
    }

}

void EEPROM_ReadAll(void)
{
    eeprom_read_block(&eeGlobals, EEPROM_SAFETY,sizeof(eeGlobals));
}


void EEPROM_WriteAll(void)
{
    eeprom_write_block(&eeGlobals, EEPROM_SAFETY, sizeof(eeGlobals));
}

void EEPROM_main (void)
{
   EEPROM_InitCheck();                 // Check EEPROM is valid. If not valid, Init it
   EEPROM_ReadAll ();                  // Get retained Info from EEPROM
}
