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
  21 0000 0F93      		push r16
  22 0002 1F93      		push r17
  23 0004 CF93      		push r28
  24 0006 DF93      		push r29
  25               	/* prologue: function */
  26               	/* frame size = 0 */
  27               	.LM2:
  28 0008 E091 0000 		lds r30,uip_udp_conn
  29 000c F091 0000 		lds r31,(uip_udp_conn)+1
  30 0010 8681      		ldd r24,Z+6
  31 0012 9781      		ldd r25,Z+7
  32 0014 8038      		cpi r24,128
  33 0016 9105      		cpc r25,__zero_reg__
  34 0018 01F4      		brne .L6
  35               	.LM3:
  36 001a 8091 0000 		lds r24,uip_flags
  37 001e 81FF      		sbrs r24,1
  38 0020 00C0      		rjmp .L6
  39               	.LBB4:
  40               	.LBB5:
  41               	.LM4:
  42 0022 0091 0000 		lds r16,uip_appdata
  43 0026 1091 0000 		lds r17,(uip_appdata)+1
  44               	.LVL0:
  45 002a C0E0      		ldi r28,lo8(0)
  46 002c D0E0      		ldi r29,hi8(0)
  47               	.LVL1:
  48               	.L3:
  49               	.LM5:
  50 002e 80E0      		ldi r24,lo8(0)
  51 0030 90E0      		ldi r25,hi8(0)
  52 0032 0E94 0000 		call serial_tx_hex
  53               	.LM6:
  54 0036 80E2      		ldi r24,lo8(32)
  55 0038 90E0      		ldi r25,hi8(32)
  56 003a 0E94 0000 		call serial_tx
  57               	.LM7:
  58 003e 2196      		adiw r28,1
  59 0040 CE30      		cpi r28,14
  60 0042 D105      		cpc r29,__zero_reg__
  61 0044 01F4      		brne .L3
  62               	.LM8:
  63 0046 0E94 0000 		call serial_crlf
  64               	.LM9:
  65 004a 8091 0000 		lds r24,uip_len
  66 004e 9091 0000 		lds r25,(uip_len)+1
  67 0052 892B      		or r24,r25
  68 0054 01F0      		breq .L4
  69 0056 C0E0      		ldi r28,lo8(0)
  70 0058 D0E0      		ldi r29,hi8(0)
  71               	.LVL2:
  72               	.L5:
  73               	.LM10:
  74 005a F801      		movw r30,r16
  75 005c EC0F      		add r30,r28
  76 005e FD1F      		adc r31,r29
  77 0060 8081      		ld r24,Z
  78 0062 90E0      		ldi r25,lo8(0)
  79 0064 0E94 0000 		call serial_tx_hex
  80               	.LM11:
  81 0068 80E2      		ldi r24,lo8(32)
  82 006a 90E0      		ldi r25,hi8(32)
  83 006c 0E94 0000 		call serial_tx
  84               	.LM12:
  85 0070 2196      		adiw r28,1
  86 0072 8091 0000 		lds r24,uip_len
  87 0076 9091 0000 		lds r25,(uip_len)+1
  88 007a C817      		cp r28,r24
  89 007c D907      		cpc r29,r25
  90 007e 00F0      		brlo .L5
  91               	.L4:
  92               	.LM13:
  93 0080 0E94 0000 		call serial_crlf
  94               	.LM14:
  95 0084 0E94 0000 		call serial_crlf
  96               	.L6:
  97               	/* epilogue start */
  98               	.LBE5:
  99               	.LBE4:
 100               	.LM15:
 101 0088 DF91      		pop r29
 102 008a CF91      		pop r28
 103               	.LVL3:
 104 008c 1F91      		pop r17
 105 008e 0F91      		pop r16
 106               	.LVL4:
 107 0090 0895      		ret
 108               	.LFE3:
 110               	.global	aun_init
 112               	aun_init:
 113               	.LFB2:
 114               	.LM16:
 115 0092 DF93      		push r29
 116 0094 CF93      		push r28
 117 0096 00D0      		rcall .
 118 0098 00D0      		rcall .
 119 009a CDB7      		in r28,__SP_L__
 120 009c DEB7      		in r29,__SP_H__
 121               	/* prologue: function */
 122               	/* frame size = 4 */
 123               	.LM17:
 124 009e 1092 0000 		sts s+2,__zero_reg__
 125               	.LM18:
 126 00a2 1A82      		std Y+2,__zero_reg__
 127 00a4 1982      		std Y+1,__zero_reg__
 128 00a6 1C82      		std Y+4,__zero_reg__
 129 00a8 1B82      		std Y+3,__zero_reg__
 130               	.LM19:
 131 00aa CE01      		movw r24,r28
 132 00ac 0196      		adiw r24,1
 133 00ae 60E8      		ldi r22,lo8(128)
 134 00b0 70E0      		ldi r23,hi8(128)
 135 00b2 0E94 0000 		call uip_udp_new
 136 00b6 FC01      		movw r30,r24
 137 00b8 9093 0000 		sts (s)+1,r25
 138 00bc 8093 0000 		sts s,r24
 139               	.LM20:
 140 00c0 0097      		sbiw r24,0
 141 00c2 01F0      		breq .L11
 142               	.LM21:
 143 00c4 80E8      		ldi r24,lo8(128)
 144 00c6 90E0      		ldi r25,hi8(128)
 145 00c8 9583      		std Z+5,r25
 146 00ca 8483      		std Z+4,r24
 147               	.L11:
 148               	/* epilogue start */
 149               	.LM22:
 150 00cc 0F90      		pop __tmp_reg__
 151 00ce 0F90      		pop __tmp_reg__
 152 00d0 0F90      		pop __tmp_reg__
 153 00d2 0F90      		pop __tmp_reg__
 154 00d4 CF91      		pop r28
 155 00d6 DF91      		pop r29
 156 00d8 0895      		ret
 157               	.LFE2:
 159               		.lcomm s,3
 192               	.Letext0:
DEFINED SYMBOLS
                            *ABS*:00000000 aun.c
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:2      *ABS*:0000003f __SREG__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:3      *ABS*:0000003e __SP_H__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:4      *ABS*:0000003d __SP_L__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:5      *ABS*:00000034 __CCP__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:6      *ABS*:00000000 __tmp_reg__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:7      *ABS*:00000001 __zero_reg__
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:18     .text:00000000 aun_appcall
C:\DOCUME~2\Mark\LOCALS~1\Temp/cciF3osZ.s:112    .text:00000092 aun_init
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
