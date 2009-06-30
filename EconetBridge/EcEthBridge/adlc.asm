; =======================================================================
; == ADLC ===============================================================
; =======================================================================

; -----------------------------------------------------------------------------------------
; ADLC Read
; -----------------------------------------------------------------------------------------
;
; r16=address
; r17=data

adlc_read:
	cbi	PORTD, ADLC_RS0			; clear RS0 on PORTD
	cbi	PORTD, ADLC_RS1			; clear RS1 on PORTD
	sbrc	r16, 0				; if bit 0 in the address field is not clear
	sbi	PORTD, ADLC_RS0			;    set bit 0
	sbrc	r16, 1				; if bit 1 in the address field is not clear
	sbi	PORTD, ADLC_RS1			;    set bit 1
	sbi	PORTD, ADLC_RnW			; Read from ADLC

	ldi	r16, 0x1				; make sure the ADLC_CLK at PB0 stays set
	out	DDRB, r16				; clear all other lines on PORTB
	cbi	DDRE, ADLC_D0			; clear ADLC_D0 line on PORTE

	clc
	
	in	r18, GICR				; save the contents of the General Interrupt Control Register
	ldi	tmp, (1 << INT1)		; External INT1 enable
	out	GIFR, tmp				; write to the General Interrupt Flag Register
	out	GICR, tmp				; write to the General Interrupt Control Register


adlc_read_wait:					; wait for adlc_access to run
	brcc	adlc_read_wait		; r16 and r17 will return with PortB and PortE
	
	bst	r16, ADLC_D0			; Store the bit from ADLC_D0 in the T flag
	bld	r17, 0					; load T into bit 0 of r17 to leave the byte read in r17

	ldi	r16, 0xff				; set pins back to output
	out	DDRB, r16				; Port B
	sbi	DDRE, ADLC_D0			; ADLC_DO pin on PORT E
	ret


; -----------------------------------------------------------------------------------------
; ADLC Write
; -----------------------------------------------------------------------------------------
;
; r16=address
; r17=data

adlc_write:
	; set the addressing
	cbi	PORTD, ADLC_RS0			; clear RS0 on PORTD -
	cbi	PORTD, ADLC_RS1			; clear RS1 on PORTD - ADLC Rx
	sbrc	r16, 0				; if bit 0 in the address field is not clear
	sbi	PORTD, ADLC_RS0			;    set bit 0
	sbrc	r16, 1				; if bit 1 in the address field is not clear
	sbi	PORTD, ADLC_RS1			;    set bit 1
	cbi	PORTD, ADLC_RnW			; write to ADLC

	; set the data
	out	PORTB, r17				; put the data on PORT B
	cbi	PORTE, ADLC_D0			; clear bit 2 on PORT E
	sbrc	r17, 0				; if bit 0 in the data field is not clear
	sbi	PORTE, ADLC_D0			;    set bit to 1

	clc
	
	in	r18, GICR				; save the contents of the General Interrupt Control Register
	ldi	tmp, (1 << INT1)		; External INT1 enable
	out	GIFR, tmp				; write to the General Interrupt Flag Register
	out	GICR, tmp				; write to the General Interrupt Control Register

adlc_write_wait:				; wait for adlc_access to run
	brcc adlc_write_wait		; by checking if carry is cleared
	ret


; -----------------------------------------------------------------------------------------
; ADLC access
; -----------------------------------------------------------------------------------------
;
; perform one read or write access cycle

adlc_access:
	nop
	nop
	cbi	PORTE, ADLC_nCE			; Clear the ADLC_nCE
	nop	
	nop
	nop
	nop
	nop
	in	r17, PINB				; read PortB to r17
	in	r16, PINE				; read PortE to r16
	nop
	sbi	PORTE, ADLC_nCE			; set ADLC_nCE to flag read completed
	out	GICR, r18				; restore the contents of the Global Interrupt Control Register
	sec							; set carry to flag interrupt is handled
	reti						; return from interrupt

; -----------------------------------------------------------------------------------------
; ADLC Tx frame
; -----------------------------------------------------------------------------------------
;

adlc_tx_frame:
	ret							; do nothing

; -----------------------------------------------------------------------------------------
; ADLC Tx IRQ
; -----------------------------------------------------------------------------------------
;

adlc_tx_irq:
	rjmp	adlc_irq_ret		; return from interrupt


; -----------------------------------------------------------------------------------------
; ADLC process_fv Frame Valid
; -----------------------------------------------------------------------------------------
;
;
; exit : adlc_state = FRAME COMPLETE
; 
process_fv:
	ldi	r16, 0x76				; "v" CRC is  Valid
	call	serial_tx			; write to serial

	rcall	adlc_rx_flush		; clear remaining bytes in the input buffer

	ldi	r16, FRAME_COMPLETE		;
	sts	adlc_state, r16			; set ADLC state to FRAME COMPLETE

	ldi	r16, ADLC_CR1			; write to Control Register 1
	ldi	r17, CR1_TXRESET		; Tx Reset
	rcall	adlc_write			; write to ADLC

	rjmp	adlc_irq_ret		; return from interrupt


; -----------------------------------------------------------------------------------------
; ADLC adlc_reg_debug
; -----------------------------------------------------------------------------------------
;
; used temporarily to write the status register values to an area of memory
;
; entry : r16 - register value
;         r18 - register
;
adlc_reg_debug:

	lds	XL, adlc_tmp_ptr		; put the Tx buffer pointer address in X
	lds	XH, adlc_tmp_ptr + 1
	st  X+, r18					; store Reg number
	st	X+, r16					; store the data read in the temp Tx buffer
	sts	adlc_tmp_ptr, XL		; store the incremented Rx buffer pointer
	sts	adlc_tmp_ptr + 1, XH	; 
	ret

; -----------------------------------------------------------------------------------------
; ADLC IRQ
; -----------------------------------------------------------------------------------------
;

adlc_irq:	
	push	tmp					; save register values to the stack
	push	r16
	push	r17
;	push    r18					; temp for debug
	in		r16, SREG			; Save the status register
	push	r16

	; disable adlc irq and re-enable global interrupts
	clr	tmp
	out	GICR, tmp
	sei

	lds	tmp, adlc_state			; get the ADLC state
	;bst	tmp, 7				; bit store bit 7 in of tmp in T
	;brbs	6, adlc_tx_irq		; if T is set, ADLC state is Tx - branch to adlc_tx_irq

	ldi	r16, ADLC_SR1			; set read address to ADLC Status Register1
	rcall	adlc_read			; read the ADLC Status Register into r17
/*
; temp for debugging, Store the read register in the Tx buffer
	ldi r18, 0x01				; Status Reg 01
	rcall adlc_reg_debug
*/
	bst	r17, 0					; RDA? Receive data available
	brbs	6, process_rda		; yes
	bst	r17, 1					; S2RQ - Status #2 Read Request ?
	brbs	6, process_s2rq		; yes

;this hasn't occurred so far
	ldi	r16, 0x56				; "V" - valid CRC is Valid
	rcall	serial_tx			; write to serial

	rjmp	adlc_irq_ret		; clean up routine for interrupt handling


process_s2rq:
	ldi	r16, ADLC_SR2			; set address to Status Register2
	rcall	adlc_read			; read Status Register2
/*
; temp for debugging, Store the read register in the Tx buffer
	ldi r18, 0x02				; Status Reg 01
	rcall adlc_reg_debug
*/

	bst	r17, 0					; 1 ;AP: Address Present?
	brbs	6, process_ap		;   ;yes
	bst	r17, 7					; 80;RDA? Receive Data Available
	brbs	6, process_rda		;   ;yes
	bst	r17, 6					; 40;overrun? Rx Overrun
	brbs	6, rx_overrun		;   ;yes
	bst	r17, 5					; 20;DCD? signifies presence of clock
	brbs	6, process_dcd		;   ;yes
	bst	r17, 4					; 10;CRC error?
	brbs	6, rx_error			;   ;yes
	bst	r17, 3					; 8 ;Abort? Abort Received?
	brbs	6, abandon_rx		;   ;yes
	bst	r17, 2					; 4 ;Idle?: Receive Idle?
	brbs	6, process_idle		;   ;yes
	bst	r17, 1					; 2 ;FV: Frame Valid?
	brbs	6, process_fv		;   ;yes



; -----------------------------------------------------------------------------------------
; ADLC process_ap Address Present
; -----------------------------------------------------------------------------------------
;
; The address field is the first 8 bits following the opening flag, so will always be the
; first byte of the packet. This byte being present causes the Rx Interrupt if enabled.

process_ap:
;	ldi	r16, 0x61				; "a"
;	call	serial_tx			; send to serial

	lds	XL, adlc_rx_ptr			; put the Rx buffer pointer address in X
	lds	XH, adlc_rx_ptr + 1

	; Read 1 byte to the buffer
	ldi	r16, ADLC_RX			; set read address to ADLC_RX (2) 
	rcall	adlc_read			; read ADLC, r17 = data read

	st	X+, r17					; store the data read in the Rx buffer

	sts	adlc_rx_ptr, XL			; store the incremented Rx buffer pointer
	sts	adlc_rx_ptr + 1, XH		; 

	lds	r16, adlc_state			; add 1 to adlc_state
	inc	r16
	sts	adlc_state, r16

;delay	
	ldi		r16, 160			; set up delay loop
delay:
	dec		r16					; minus 1
	brne	delay				; loop until finished

	rjmp	process_s2rq		; continue processing Status Register 2 until finished


; -----------------------------------------------------------------------------------------
; ADLC process_rda Receive Data Available
; -----------------------------------------------------------------------------------------
;

process_rda:
;	ldi		r16, 0x72			; "r" - rda entered
;	rcall 	serial_tx			; 

	lds	XL, adlc_rx_ptr		; put the Rx buffer pointer address in X
	lds	XH, adlc_rx_ptr + 1

	; read 2 bytes to the buffer

	; 1st byte
	ldi	r16, ADLC_RX			; set read address to ADLC_RX (2) 
	rcall	adlc_read			; read ADLC, r17 = data read
	st	X+, r17					; store the data read in the Rx buffer

	; 2nd byte
	ldi	r16, ADLC_RX			; set read address to ADLC_RX (2) 
	rcall	adlc_read			; read ADLC, r17 = data read
	st	X+, r17					; store the data read in the Rx buffer

	sts	adlc_rx_ptr, XL			; store the incremented Rx buffer pointer
	sts	adlc_rx_ptr + 1, XH		; 

	rjmp	adlc_irq_ret		; return from interrupt



; -----------------------------------------------------------------------------------------
; ADLC rx_overrun
; -----------------------------------------------------------------------------------------
;
rx_overrun:
	ldi		r16, 0x6F			; "o" - overrun
	rcall 	serial_tx			; 
	rjmp 	abandon_rx

; -----------------------------------------------------------------------------------------
; ADLC process_dcd - No Clock
; -----------------------------------------------------------------------------------------
;
process_dcd:
	ldi		r16, 0x6E			; "n" - no clock
	rcall 	serial_tx			; 
;	rcall 	noclock
	rjmp	adlc_irq_ret		; return from interrupt


; -----------------------------------------------------------------------------------------
; ADLC rx_error	Receive Error
; -----------------------------------------------------------------------------------------
;
rx_error:
	ldi		r16, 0x65			; "e" - error
	rcall 	serial_tx			; 
	rcall	adlc_rx_flush		; clear any remaing bytes

; -----------------------------------------------------------------------------------------
; ADLC abandon_rx Abandon Rx
; -----------------------------------------------------------------------------------------
;
; exit: adlc_state = 0
; 
abandon_rx:
	; abandon in-progress reception
	clr	tmp
	sts	adlc_state, tmp			; clear the adlc_state
	rcall	adlc_clear_rx		; clear rx status
	rjmp	process_s2rq		; might be AP as well

; -----------------------------------------------------------------------------------------
; ADLC process_idle
; -----------------------------------------------------------------------------------------
;
; exit: adlc_state = 0
; 
process_idle:					
	ldi	r16, 0x69				; append "i" - network is idle
	call	serial_tx			; write to serial
	rcall	crlf

	clr	tmp						; tmp=0
	sts	adlc_state, tmp			; ADLC state=0

	rcall	adlc_clear_rx		; clear Rx Status Register
	rjmp	adlc_irq_ret		; clean up routine for interrupt handling



; -----------------------------------------------------------------------------------------
; ADLC Rx Flush
; -----------------------------------------------------------------------------------------
	
adlc_rx_flush:
	; switch to 1-byte mode, no PSE
	ldi	r16, ADLC_CR2			; set to write to Control Register 2
	clr	r17						; clear all bits
	rcall	adlc_write			; write to Control Register

	; read SR2
	ldi	r16, ADLC_SR2			; read Status Register 2
	rcall	adlc_read			; read Satus Register into r17

	;if there is a byte to read, read it first before clearing
	
	bst	r17, 7					; RDA? is receive data available
	brbc	6, no_rda			; no

	lds	XL, adlc_rx_ptr			; put the Rx buffer pointer address in X
	lds	XH, adlc_rx_ptr + 1

	; Read 1 byte to the buffer
	ldi	r16, ADLC_RX			; set read address to ADLC_RX (2) 
	rcall	adlc_read			; read ADLC, r17 = data read

	st	X+, r17					; store the data read in the Rx buffer

	sts	adlc_rx_ptr, XL			; store the incremented Rx buffer pointer
	sts	adlc_rx_ptr + 1, XH		; 


no_rda:
	rjmp	adlc_clear_rx		; clear rx status


; -----------------------------------------------------------------------------------------
; ADLC clear Rx
; -----------------------------------------------------------------------------------------
;
; clear rx status and switch back to 1-byte mode
;

adlc_clear_rx:
	ldi	r16, ADLC_CR2			; set to write to Control Register 2
	ldi	r17, CR2val				; reset Receieve status bits
	rjmp	adlc_write			; write to ADLC

; -----------------------------------------------------------------------------------------
; ADLC IRQ ret
; -----------------------------------------------------------------------------------------

adlc_irq_ret:
	ldi	tmp, (1 << INT0)		; restore the interrupt flag
	out	GICR, tmp
	pop	r16						; restore Status register from the Stack
	out	SREG, r16		
;	pop r18						; temp for debug
	pop	r17						; restore other registers from the Stack
	pop	r16
	pop	tmp
	reti						; return from interrupt

; -----------------------------------------------------------------------------------------
; ADLC init
; -----------------------------------------------------------------------------------------


adlc_init:
	; During a power-on sequence, the ADLC is reset via the RESET input and internally 
	; latched in a reset condition to prevent erroneous output transitions. The four control
	; registers must be programmed prior to the release of the reset condition. The release
	; of the reset condition is peformed by software by writing a "0" into the Rx RS control
	; bit (receiver) and/or Tx RS control bit (transmitter). The release of the reset condition
	; must be done after the RESET input has gone high.
	; At any time during operation, writing a "1" into the Rx RS control bit or Tx RS control
	; bit causes the reset condition of the receiver or the transmitter.


	; raise the reset line on the ADLC
	ldi	r16, EGPIO_ADLC_RESET | EGPIO_SET		; set the ADLC reset to 1
	rcall	egpio_write							; write to ADLC

	; set the initial values of Control Register 1
	ldi	r17, CR1_AC | CR1_TXRESET | CR1_RXRESET	; Switch controlling access to CR3
												; Resets all Tx, Rx status and FIFO registers
												; these will automatically go back to 0
	ldi	r16, ADLC_CR1							; set to ADLC Control Register 1
	rcall	adlc_write							; write to ADLC

	;new
	ldi r17, 0x1E								
	ldi r16, ADLC_TXTERMINATE					; write to TXTerminate address
	rcall adlc_write
	;end new

	; set the initial values of Control Register 3 (CR1_AC previously set)
	clr	r17										; Set initial value to 0
	ldi	r16, ADLC_CR3							; set to ADLC Control Register 3
	rcall	adlc_write							; write to ADLC


	; set the initial values of Control Register 4
	ldi	r17, CR4_TXWLS | CR4_RXWLS				; Set Tx & Rx Word lengths to 8 bits
	ldi	r16, ADLC_CR4							; set to ADLC Control Register 4
	rcall	adlc_write							; write to ADLC


; -----------------------------------------------------------------------------------------
; ADLC ready to receive
; -----------------------------------------------------------------------------------------
;
; exit	: adlc_state = 0
; 		: adlc_rx_ptr    = ECONET RX Buffer LSB
; 		: adlc_rx_ptr + 1    = ECONET RX Buffer MSB

adlc_ready_to_receive:
 	
	;initialise the adlc_state
	
	clr	tmp							; tmp = 0
	sts	adlc_state, tmp				; adlc_state=0

	; initialise the Rx Buffer pointer

	ldi	XL, ECONET_RX_BUF & 0xff	; set XL with the lowbyte of the Econet receive buffer
	ldi	XH, ECONET_RX_BUF >> 8		; set XH with the highbyte of the Econet receive buffer
	
	sts	adlc_rx_ptr, XL				; set ALDC receive ptr to the start of the Rx buffer
	sts	adlc_rx_ptr + 1, XH

/*
	; temp for debugging. Store a copy of the ADLC status Register
	; use the Tx buffer because that is not currently being used
	ldi	XL, ECONET_TX_BUF & 0xff	; set XL with the lowbyte of the Econet receive buffer
	ldi	XH, ECONET_TX_BUF >> 8		; set XH with the highbyte of the Econet receive buffer
	
	sts	adlc_tmp_ptr, XL				; set ALDC receive ptr to the start of the Rx buffer
	sts	adlc_tmp_ptr + 1, XH
*/
	
	ldi	r17, CR1_RIE | CR1_TXRESET	; Enable Receive interrupts | Reset the TX status
	ldi	r16, ADLC_CR1				; set to write to Control Register 1
	rcall	adlc_write				; write to the ADLC

	ldi	r17, CR2val					; Status bit prioritised
	ldi	r16, ADLC_CR2				; set to write to Control Register 2
	rjmp	adlc_write				; write to the ADLC
