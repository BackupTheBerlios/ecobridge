
; Cirrus Logic CS8900a I/O Registers


.equ	CS_DATA_P0		= CS8900_BASE + 0x00			; R/W	Rx / Tx Data Port 0
.equ	CS_DATA_P1		= CS8900_BASE + 0x02			; R/W	Rx / Tx Data Port 1
.equ	CS_TX_CMD_I		= CS8900_BASE + 0x04			; Write	TxCMD (Transmit Command)
.equ	CS_TX_LEN_I		= CS8900_BASE + 0x06			; Write	TxLength (Transmit Length)
.equ	CS_INT_STAT		= CS8900_BASE + 0x08			; Read	Interrupt Status Queue
.equ	CS_PP_PTR		= CS8900_BASE + 0x0A			; R/W	Packet Page Pointer
.equ	CS_PP_DATA0		= CS8900_BASE + 0x0C			; R/W	Packet Page Data (Port 0)
.equ	CS_PP_DATA1		= CS8900_BASE + 0x0E			; R/W	Packet Page Data (Port 1)


; Cirrus Logic CS8900a Packet Page registers

; 0x0000 - 0x0045 Bus Interface Registers

.equ	CS_PROD_ID		= 0x0000	; Read Only				Product Identification code
.equ	CS_IO_BASE		= 0x0020	; Default = 0300h		I/O Base Address	
.equ	CS_INT_NUM		= 0x0022	; XXXX XXXX XXXX X100	Interrupt Number
.equ	CS_DMA_CHAN		= 0x0024	; XXXX XXXX XXXX XX11	DMA Channel
.equ	CS_DMA_SOF		= 0x0026	; 0000h					DMA Start of Frame Offset
.equ	CS_DMA_FCNT		= 0x0028	; X000h					DMA Frame Count
.equ	CS_DMA_RXCNT	= 0x002A	; 0000h					DMA Byte Count
.equ	CS_MEM_BASE		= 0x002C	; XXX0 0000h			Memory Base Address
.equ	CS_BOOT_BASE	= 0x0030	; XXX0 0000h			Boot PROM Base Address
.equ	CS_BOOT_MASK	= 0x0034	; XXX0 0000h			Boot PROM Address Mask
.equ	CS_EE_CMD		= 0x0040	
.equ	CS_EE_DATA		= 0x0042
.equ	CS_RX_FRM_CNT	= 0x0050

; 0x0100 - 0x013F Status and Control Registers

; configuration and control registers
.equ	CS_RX_CFG		= 0x0102	; 0003h					Register 03 - Receiver Configuration
.equ	CS_RX_CTL		= 0x0104	; 0005h					Register 05 - Receiver Control
.equ	CS_TX_CFG		= 0x0106	; 0007h					Register 07 - Transmit Configuration
.equ	CS_TX_CMD_P		= 0x0108	; 0009h					Register 09 - Transmit Command Status
.equ	CS_BUF_CFG		= 0x010A	; 000Bh					Register 0B - Buffer Configuration
;						  0x010C	; Undefined				Reserved
;						  0x010E	; Undefined				Reserved
;						  0x0110	; Undefined				Reserved
.equ	CS_LINE_CTRL	= 0x0112	; 00013h [sic]			Register 13 - Line Control
.equ	CS_SELF_CTRL	= 0x0114	; 0015h					Register 15 - Self Control
.equ	CS_BUS_CTRL		= 0x0116	; 0017h					Register 17 - Bus Control
.equ	CS_TEST_CTRL	= 0x0118	; 0019h					Register 19 - Test Control

;status and event registers
.equ	CS_ISQ			= 0x0120	;						Register 00 - Interrupt Status Queue				
.equ	CS_RX_EVENT		= 0x0124	;						Register 04 - Receiver Event
.equ	CS_TX_EVENT		= 0x0128	;						Register 08 - Transmitter Event
.equ	CS_BUF_EVENT	= 0x012C	;						Register 0C - Buffer Event
.equ	CS_RX_MISS		= 0x0130	;						Register 10 - Receiver Miss Counter
.equ	CS_TX_COLL		= 0x0132	;						Register 12 - Transmit Collision Counter
.equ	CS_LINE_STAT	= 0x0134	;						Register 14 - Line Status
.equ	CS_SELF_STAT	= 0x0136	;						Register 16 - Self Status
.equ	CS_BUS_STAT		= 0x0138	;						Register 18 - Bus Status
.equ	CS_AUI_TDR		= 0x013C	;						Register 1C - AUI Time Domain Reflectometer

; 0x0140 - 0x014F Initiate Transmit Registers

.equ	CS_PP_TX_CMD	= 0x0144	;						Transmit Command Request
.equ	CS_PP_TX_LEN	= 0x0146	;						Transmit Length

; 0x0150 - 0x015D Address Filter Registers

.equ	CS_LOG_ADDR		= 0x0150	;						Logical Address Filter (hash table)
.equ	CS_IEEE_ADDR	= 0x0158	;						Individual Address

; Frame location
.equ	CS_RX_STATUS	= 0x0400	;						Receive Status
.equ	CS_RX_LENGTH	= 0x0402	;						Receive Length in bytes
.equ	CS_RX_FR_LOC	= 0x0404	;						Receive Frame Location
.equ	CS_TX_FR_LOC	= 0x0A00	;						Transmit Frame Location


; =======================================================================
; == CS8900==============================================================
; =======================================================================
;
; CS8900 has 16bit registers. Both bytes should be used when reading or writing


; -----------------------------------------------------------------------------------------
; CS8900 cs_init
; -----------------------------------------------------------------------------------------
;
; initialise CS8900 Ethernet interface
;
; on exit
;			r16 = 0	: Success
;				= 1	: Not Found
;				= 2 : No Response

cs_init:	

	; check for chip
	lds r18, CS_PP_DATA0			; read the Packet Page Data Port
	lds r19, CS_PP_DATA0+1

	ldi	r16, 0x0E					; expected value
	cp	r18, r16
	brne cs8900_not_found			; didn't find it						;

	ldi r17, 0x63 					; expected value
	cp r19, r17
	brne cs8900_not_found			; didn't find it						;


	rcall cs_software_wakeup
	rcall cs_software_reset
	;wait for reset to complete

	; poll register 16 (Self Status) insted of inserting a time delay. 
	; INITD=1 initialisation is complete
	; SIBUSY=0 indicates the EEPROM is no longer being accessed

Initialising:

    ; Read Self Status register into r18 and r1
    ldi r16,CS_SELF_STAT & 0xFF		; LSB
	ldi r17,CS_SELF_STAT >> 8		; MSB
	rcall cs_read_pp


	; read Self Status Register
	; INITD = bit 7 0x80 of LSB
	; SiBUSY = bit 8 0x1 of MSB
	bst	r18, 7					; stores bit7 INITD of r18 in T flag 
	brbs 6, INIT_Complete	 	; if set, then the initialisation is complete
	rjmp Initialising

INIT_Complete:

    ldi r16,CS_SELF_STAT & 0xFF		; LSB
	ldi r17,CS_SELF_STAT >> 8		; MSB
	rcall cs_read_pp

	bst	r19, 1					; stores bit7 SiBusy of r19 in T flag 
	brbs 6, Init_Complete		; if set, still accessing the EEPROM

	; check it responds by writing 0x0102 to Packet Page Pointer Port
	ldi r18, 0x01
	ldi r19, 0x02
	sts	CS_PP_PTR, r18
	sts	CS_PP_PTR+1, r19

	; and read the Paket Page Data Port which should contain 0x03
	lds r18, CS_PP_DATA0			; read the Packet Page Data Port
	lds r19, CS_PP_DATA0+1

	ldi	r16, 0x03					; expected value
	cp	r19, r16
	brne cs8900_no_response			; didn't find it						;

    ; Add the MAC address to the NIC
	ldi r16, CS_IEEE_ADDR & 0xFF	; LSB
    ldi r17, CS_IEEE_ADDR >> 8		; MSB
	ldi r18, 0x00					; Octet 5 of IA
	clr r19
	rcall cs_write_pp

	inc r16							; increment the LSB
	ldi r18,0x06					; Octet 4 of IA
	rcall cs_write_pp

	inc r16							; increment the LSB
	ldi r18,0x98					; Octet 3 of IA
	rcall cs_write_pp

	inc r16							; increment the LSB
	ldi r18,0x00					; Octet 2 of IA
	rcall cs_write_pp

	inc r16							; increment the LSB
	ldi r18,0x00					; Octet 1 of IA
	rcall cs_write_pp

	inc r16							; increment the LSB
	ldi r18,0x00					; Octet 0 of IA
	rcall cs_write_pp


    ; Enable Tx
    ldi r16,CS_LINE_CTRL & 0xFF		; LSB
	ldi r17,CS_LINE_CTRL >> 8		; MSB
	ldi r18, 0xC0					; LSB
	clr r19							; MSB
	rcall cs_write_pp

    ; Enable Rx
	ldi r16,CS_RX_CTL & 0xFF		; LSB
    ldi r17,CS_RX_CTL >> 8			; MSB
	ldi r18, 0x40					; LSB
	ldi r19, 0x0F					; MSB
	rcall cs_write_pp

; return codes

	ldi r16,0x00
	ret
cs8900_not_found:
	ldi r16, 0x01
	ret
cs8900_no_response:
	ldi r16, 0x02
	ret

; -----------------------------------------------------------------------------------------
; CS8900 cs_poll
; -----------------------------------------------------------------------------------------
;
cs_poll:

/* Psuedo code

#define EventMask = 0xFFC0
#define RegisterMask = 0x003F
#define RxEvent = 0x0004
#define TxEvent = 0x0008
#define BufEvent = 0x000C

Poll Chip {

	unsigned short Even;

	Event = Poll_Registers()
	While Event <> 0x0000 {
		Switch (RegisterMask & Event) {
			Case RxEvent:
				result = Process - RxEvent(Event);
				break;
			Case TxEvent:
				result = Process - TxEvent(Event);
				break;
			Case BufEvent:
				result = Process - BufEvent(Event);
				break:
			} // End Switch
			Even = Poll-Registers()
	} // End While
} // End Poll-Chip

Poll-Registers {
	unsigned short Event;

	Event = Read - RxEventRegister();
	If (EventMask & Event) {
			return Event;}

	Event = Read - TxEventRegister();
	If (EventMask & Event) {
			return Event;}

	Return 0x0000
// End Poll Registers
}

*/

	ret


; -----------------------------------------------------------------------------------------
; CS8900 cs_software_wakeup
; -----------------------------------------------------------------------------------------
;
cs_software_wakeup:

	; wake up the CS8900
	ldi r16,CS_SELF_CTRL & 0xFF		; LSB   CS_PP_PTR = CS_SELF_CTRL
    ldi r17,CS_SELF_CTRL >> 8		; MSB
	sts	CS_PP_PTR, r16
	sts	CS_PP_PTR+1, r17

	ret


; -----------------------------------------------------------------------------------------
; CS8900 cs_software_reset
; -----------------------------------------------------------------------------------------
;
cs_software_reset:

    ; send a software reset 
	ldi r16,CS_SELF_CTRL & 0xFF		; LSB   CS_PP_PTR = CS_SELF_CTRL
    ldi r17,CS_SELF_CTRL >> 8		; MSB
	ldi r18, 0x40					; LSB	with CS_DATA_P0 = 0x0040
	clr r19							; MSB
	rcall cs_write_pp

	ret

; -----------------------------------------------------------------------------------------
; CS8900 cs_write_pp
; -----------------------------------------------------------------------------------------
;
; on entry
; 			r16/r17 = address
; 			r18/r19 = data
; on exit
; 			r16/r17 = address
; 			r18/r19 = data

cs_write_pp:
	sts	CS_PP_PTR, 		r16
	sts	CS_PP_PTR+1,	r17
	sts	CS_PP_DATA0, 	r18
	sts	CS_PP_DATA0+1,	r19
	ret

; -----------------------------------------------------------------------------------------
; CS8900 cs_read_pp
; -----------------------------------------------------------------------------------------
;
; on entry
; 			r16/r17 = address
; on exit
; 			r16/r17 = address
; 			r18/r19 = data

cs_read_pp:
	sts	CS_PP_PTR, r16
	sts	CS_PP_PTR+1, r17
	lds	r18, CS_PP_DATA0
	lds	r19, CS_PP_DATA0+1
	ret

; -----------------------------------------------------------------------------------------
; CS8900 cs_debug_pp
; -----------------------------------------------------------------------------------------
;
; Will read the two bytes from the PP data register as pointed to by r16 / r17
;
; on entry
; 			r16/r17 = address
; on exit
; 			r18/r19 = data
;
cs_debug_pp:

	rcall cs_read_pp	
	rcall output_r18_r19
	rcall crlf
	ret


; -----------------------------------------------------------------------------------------
; Some general debugging routines to output some registers to the serial port
; -----------------------------------------------------------------------------------------


output_r16:
	rcall serial_tx_hex
	ret

output_r17:
	mov r16,r17
	rcall serial_tx_hex
	ret

output_r18:
	mov r16, r18
	rcall serial_tx_hex
	ret

output_r19:
	mov r16, r19
	rcall serial_tx_hex
	ret

output_r16_r17:
	rcall output_r16			; output r16
	ldi r16, 0x20				; ouput space
	rcall serial_tx
	rcall output_r17			; output r17
	ret

output_r18_r19:
	rcall output_r18
	ldi r16, 0x20				; ouput space
	rcall serial_tx
	rcall output_r19
	ret

