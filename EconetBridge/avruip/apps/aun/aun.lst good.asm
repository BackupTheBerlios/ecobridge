   1               		.file	"aun.c"
   2               	__SREG__ = 0x3f
   3               	__SP_H__ = 0x3e
   4               	__SP_L__ = 0x3d
   5               	__CCP__  = 0x34
   6               	__tmp_reg__ = 0
   7               	__zero_reg__ = 1
  15               	.Ltext0:
  16               	.global	aun_appcall
  18               	aun_appcall:
  19               	.LFB3:
  20               	.LM1:
  21 0000 EF92      		push r14
  22 0002 FF92      		push r15
  23 0004 0F93      		push r16
  24 0006 1F93      		push r17
  25 0008 CF93      		push r28
  26 000a DF93      		push r29
  27               	/* prologue: function */
  28               	/* frame size = 0 */
  29               	.LM2:
  30 000c E091 0000 		lds r30,uip_udp_conn
  31 0010 F091 0000 		lds r31,(uip_udp_conn)+1
  32 0014 8681      		ldd r24,Z+6
  33 0016 9781      		ldd r25,Z+7
  34 0018 8038      		cpi r24,128
  35 001a 9105      		cpc r25,__zero_reg__
  36 001c 01F4      		brne .L6
  37               	.LM3:
  38 001e 8091 0000 		lds r24,uip_flags
  39 0022 81FF      		sbrs r24,1
  40 0024 00C0      		rjmp .L6
  41               	.LBB4:
  42               	.LBB5:
  43               	.LM4:
  44 0026 E090 0000 		lds r14,uip_appdata
  45 002a F090 0000 		lds r15,(uip_appdata)+1
  46               	.LVL0:
  47               	.LM5:
  48 002e 8701      		movw r16,r14
  49               	.LVL1:
  50 0030 C0E0      		ldi r28,lo8(0)
  51 0032 D0E0      		ldi r29,hi8(0)
  52               	.LVL2:
  53               	.L3:
  54               	.LM6:
  55 0034 F801      		movw r30,r16
  56 0036 8191      		ld r24,Z+
  57 0038 8F01      		movw r16,r30
  58 003a 90E0      		ldi r25,lo8(0)
  59 003c 0E94 0000 		call serial_tx_hex
  60               	.LM7:
  61 0040 80E2      		ldi r24,lo8(32)
  62 0042 90E0      		ldi r25,hi8(32)
  63 0044 0E94 0000 		call serial_tx
  64               	.LM8:
  65 0048 2196      		adiw r28,1
  66 004a CE30      		cpi r28,14
  67 004c D105      		cpc r29,__zero_reg__
  68 004e 01F4      		brne .L3
  69               	.LM9:
  70 0050 0E94 0000 		call serial_crlf
  71               	.LM10:
  72 0054 8091 0000 		lds r24,uip_len
  73 0058 9091 0000 		lds r25,(uip_len)+1
  74 005c 892B      		or r24,r25
  75 005e 01F0      		breq .L4
  76 0060 C0E0      		ldi r28,lo8(0)
  77 0062 D0E0      		ldi r29,hi8(0)
  78               	.LVL3:
  79               	.L5:
  80               	.LM11:
  81 0064 F701      		movw r30,r14
  82 0066 EC0F      		add r30,r28
  83 0068 FD1F      		adc r31,r29
  84 006a 8081      		ld r24,Z
  85 006c 90E0      		ldi r25,lo8(0)
  86 006e 0E94 0000 		call serial_tx_hex
  87               	.LM12:
  88 0072 80E2      		ldi r24,lo8(32)
  89 0074 90E0      		ldi r25,hi8(32)
  90 0076 0E94 0000 		call serial_tx
  91               	.LM13:
  92 007a 2196      		adiw r28,1
  93 007c 8091 0000 		lds r24,uip_len
  94 0080 9091 0000 		lds r25,(uip_len)+1
  95 0084 C817      		cp r28,r24
  96 0086 D907      		cpc r29,r25
  97 0088 00F0      		brlo .L5
  98               	.L4:
  99               	.LM14:
 100 008a 0E94 0000 		call serial_crlf
 101               	.LM15:
 102 008e 0E94 0000 		call serial_crlf
 103               	.L6:
 104               	/* epilogue start */
 105               	.LBE5:
 106               	.LBE4:
 107               	.LM16:
 108 0092 DF91      		pop r29
 109 0094 CF91      		pop r28
 110               	.LVL4:
 111 0096 1F91      		pop r17
 112 0098 0F91      		pop r16
 113               	.LVL5:
 114 009a FF90      		pop r15
 115 009c EF90      		pop r14
 116               	.LVL6:
 117 009e 0895      		ret
 118               	.LFE3:
 120               	.global	aun_init
 122               	aun_init:
 123               	.LFB2:
 124               	.LM17:
 125 00a0 DF93      		push r29
 126 00a2 CF93      		push r28
 127 00a4 00D0      		rcall .
 128 00a6 00D0      		rcall .
 129 00a8 CDB7      		in r28,__SP_L__
 130 00aa DEB7      		in r29,__SP_H__
 131               	/* prologue: function */
 132               	/* frame size = 4 */
 133               	.LM18:
 134 00ac 1092 0000 		sts s+2,__zero_reg__
 135               	.LM19:
 136 00b0 1A82      		std Y+2,__zero_reg__
 137 00b2 1982      		std Y+1,__zero_reg__
 138 00b4 1C82      		std Y+4,__zero_reg__
 139 00b6 1B82      		std Y+3,__zero_reg__
 140               	.LM20:
 141 00b8 CE01      		movw r24,r28
 142 00ba 0196      		adiw r24,1
 143 00bc 60E8      		ldi r22,lo8(128)
 144 00be 70E0      		ldi r23,hi8(128)
 145 00c0 0E94 0000 		call uip_udp_new
 146 00c4 FC01      		movw r30,r24
 147 00c6 9093 0000 		sts (s)+1,r25
 148 00ca 8093 0000 		sts s,r24
 149               	.LM21:
 150 00ce 0097      		sbiw r24,0
 151 00d0 01F0      		breq .L11
 152               	.LM22:
 153 00d2 80E8      		ldi r24,lo8(128)
 154 00d4 90E0      		ldi r25,hi8(128)
 155 00d6 9583      		std Z+5,r25
 156 00d8 8483      		std Z+4,r24
 157               	.L11:
 158               	/* epilogue start */
 159               	.LM23:
 160 00da 0F90      		pop __tmp_reg__
 161 00dc 0F90      		pop __tmp_reg__
 162 00de 0F90      		pop __tmp_reg__
 163 00e0 0F90      		pop __tmp_reg__
 164 00e2 CF91      		pop r28
 165 00e4 DF91      		pop r29
 166 00e6 0895      		ret
 167               	.LFE2:
 169               		.lcomm s,3
 202               	.Letext0:
DEFINED SYMBOLS
                            *ABS*:00000000 aun.c
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:2      *ABS*:0000003f __SREG__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:3      *ABS*:0000003e __SP_H__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:4      *ABS*:0000003d __SP_L__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:5      *ABS*:00000034 __CCP__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:6      *ABS*:00000000 __tmp_reg__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:7      *ABS*:00000001 __zero_reg__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:18     .text:00000000 aun_appcall
C:\DOCUME~2\Mark\LOCALS~1\Temp/cc2CQjV3.s:122    .text:000000a0 aun_init
                             .bss:00000000 s

UNDEFINED SYMBOLS
uip_udp_conn
uip_flags
uip_appdata
serial_tx_hex
serial_tx
serial_crlf
uip_len
uip_udp_new
__do_clear_bss
