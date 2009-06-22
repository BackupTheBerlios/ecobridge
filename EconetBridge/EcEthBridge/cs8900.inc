#ifndef _CS8900_INC_
#define _CS8900_INC_

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


#endif  /* _CS8900_INC_ */
