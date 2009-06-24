; Memory Map

.equ	CS8900_BASE		= 0x8000


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
.equ	CS_PROD_ID		= 0x0000
.equ	CS_IO_BASE		= 0x0020
.equ	CS_INT_NUM		= 0x0022
.equ	CS_DMA_CHAN		= 0x0024
.equ	CS_DMA_SOF		= 0x0026
.equ	CS_DMA_FCNT		= 0x0028
.equ	CS_DMA_RXCNT	= 0x002A
.equ	CS_MEM_BASE		= 0x002C
.equ	CS_BOOT_BASE	= 0x0030
.equ	CS_BOOT_MASK	= 0x0034
.equ	CS_EE_CMD		= 0x0040
.equ	CS_EE_DATA		= 0x0042
.equ	CS_RX_FRM_CNT	= 0x0050

.equ	CS_ISQ			= 0x0120
.equ	CS_RX_CFG		= 0x0102
.equ	CS_RX_EVENT		= 0x0124
.equ	CS_RX_CTL		= 0x0104
.equ	CS_TX_CFG		= 0x0106
.equ	CS_TX_EVENT		= 0x0128
.equ	CS_TX_CMD_P		= 0x0108
.equ	CS_BUF_CFG		= 0x010A
.equ	CS_BUF_EVENT	= 0x012C
.equ	CS_RX_MISS		= 0x0130
.equ	CS_TX_COLL		= 0x0132
.equ	CS_LINE_CTRL	= 0x0112
.equ	CS_LINE_STAT	= 0x0134
.equ	CS_SELF_CTRL	= 0x0114
.equ	CS_SELF_STAT	= 0x0136
.equ	CS_BUS_CTRL		= 0x0116
.equ	CS_BUS_STAT		= 0x0138
.equ	CS_TEST_CTRL	= 0x0118
.equ	CS_AUI_TDR		= 0x013C

.equ	CS_PP_TX_CMD	= 0x0144
.equ	CS_PP_TX_LEN	= 0x0146

.equ	CS_IEEE_ADDR	= 0x0158

.equ	RESETE			= 0x80



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
cs_init:


    // Take CS8900 out of reset and wait for internal reset to complete
;#ifdef NUT_CS8900_OLD
;    outp(inp(PORTD) & ~RESETE, PORTD);


	;wait for reset to complete


	; check for chip
	lds r18, CS_PP_DATA0			; read the Packet Page Data Port
	lds r19, CS_PP_DATA0+1


	mov r16, r18
	rcall serial_tx_hex

	mov r16, r19
	rcall serial_tx_hex

	ldi	r16, 0x63					; expected values
	cp	r18, r16
	brne cs8900_not_found			; didn't find it						;
	ldi r17, 0x0E 
	cp r19, r17
	brne cs8900_not_found			; didn't find it						;


cs8900_not_found:
	ret


/*


    if (CSReadPP16(CS_PROD_ID) != 0x630E)
        return -1;

    //
    //  Copy our MAC address to the NIC
    // 
    for (i = 0; i < 6; i += 2) {
        j = ifn->if_mac[i] << 8;
        j |= ifn->if_mac[i + 1];
        CSWritePP16(CS_IEEE_ADDR + i, j);
        j = CSReadPP16(CS_IEEE_ADDR + i);
#if 0
        if (tcp_trace) {
            NutPrintFormat_P(dev_debug, PSTR("ADDR = %x\r\n"), j);
            NutPrintFlush(dev_debug);
        }
#endif
    }

    //
    // Enable the Transmitter and Receiver
    //
    CSWritePP16(CS_LINE_CTRL, 0x00C0);
    //i = CSReadPP16(CS_LINE_CTRL);
    //NutPrintFormat_P(dev_debug,PSTR("CS_LINE_CTRL = %x\r\n"), i);

    CSWritePP16(CS_RX_CTL, 0x0F40);
    //i = CSReadPP16(CS_RX_CTL);
    //NutPrintFormat_P(dev_debug,PSTR("CS_RX_CTL = %x\r\n"), i);


*/

; -----------------------------------------------------------------------------------------
; CS8900 cs_write_pp
; -----------------------------------------------------------------------------------------

; r16/r17 = address
; r18/r19 = data

cs_write_pp:
	sts	CS_PP_PTR, 		r16
	sts	CS_PP_PTR+1,	r17
	sts	CS_PP_DATA0, 	r18
	sts	CS_PP_DATA0+1,	r19
	ret

; -----------------------------------------------------------------------------------------
; CS8900 cs_read_pp
; -----------------------------------------------------------------------------------------

; r16/r17 = address
; r18/r19 = data

cs_read_pp:
	sts	CS_PP_PTR, r16
	sts	CS_PP_PTR+1, r17
	lds	r18, CS_PP_DATA0
	lds	r19, CS_PP_DATA0+1
	ret

