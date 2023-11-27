// HW2 _ SLAVE
.equ BUTTON_MASK = (1<<PA4)|(1<<PA5)|(1<<PA6)|(1<<PA7) ; mask for checking button state
.org 0x0000 ; interrupt vector table
rjmp rst_handler ; reset

// port A: scanning 
// port D: PD0 - output IRQ signal (low active), PD1 - input SS' check
// port B: SPI 
rst_handler:
	call SPI_inital
	call keypad_prepare_port

	// inital output IRQ = HIGH
	sbi ddrd,0
	sbi portd,0

	// master's busy signal PD1
	cbi ddrd,1

main:
	call pinchange_handler ; output value in R23
	// output to portb for debug
	mov R17,R23
	CPI R23,0xff
	BREQ non_pressing
	jmp pressing
non_pressing:
	jmp main
pressing:
	call SPI_send
	call DELAY_50MS
	call DELAY_50MS
	call DELAY_50MS
	call DELAY_50MS
	jmp main


SPI_inital:
	push r16
	// SPI MISO output and another input port
	ldi r16, (1<<PB6)
	out DDRB, r16
	// enable SPI slave
	ldi r16, (1<<SPE0)
	out SPCR0,r16
	pop r16
	ret

SPI_send:
	push R16	
	cbi portD, 0	// send IRQ (LOW)
wait_sysn:
	sbic pinb,4
	jmp wait_sysn

	out SPDR0,R17	// send data to SPI data reg
wait_master_done:
	SBIS PIND,1
	jmp wait_master_done
	
	sbi portD, 0	// off IRQ (High)
	pop R16
	ret






;Set PA7..PA4 as input, PA3..PA0 as output
;set PA3..PA0 to 0
keypad_prepare_port:
	push r16
	ldi r16, 0b00001111 ; set upper 4 bits of PORTD as input with pull-up, lower 4 
	;bits as output
	out DDRA, r16
	ldi r16, 0b11110000 ; enable pull up resistor 
	out PORTA, r16
	pop r16
	ret

; check if PA7, PA6, PA5, or PA4 has changed
; store value into R23, (inital R23 = 0xff)
pinchange_handler:
	push r16
	in r16, PINA
	andi r16, BUTTON_MASK
	breq pinchange_handler_exit ; exit if none of the buttons have changed
	call keypad_scan ;scan and shiftout keycode
pinchange_handler_exit:
	pop r16
	ret 



; ATmega324PA keypad scan function
; Scans a 4x4 keypad connected to PORTA
;C3-C0 connect to PA3-PA0
;R3-R0 connect to PA7-PA4
;Returns the R23 key value (0-15) or 0xFF if no key is pressed
keypad_scan:
	ldi r20, 0b00001111 ; set upper 4 bits of PORTD as input with pull-up, lower 4 bits as output
	out DDRA, r20
	ldi r20, 0b11111111 ; enable pull up resistor 
	out PORTA, r20
	ldi r22, 0b11110111 ; initial col mask
	ldi r23, 0 ; initial pressed row value
	ldi r24,3 ; scanning col index
keypad_scan_loop:
	out PORTA, r22 ; scan current col
	nop 
	sbic PINA, 4 ; check row 0
	rjmp keypad_scan_check_col2
	rjmp keypad_scan_found ; row 0 is pressed
keypad_scan_check_col2:
	sbic PINA, 5 ; check row 1
	rjmp keypad_scan_check_col3
	ldi r23, 1 ; row1 is pressed
	rjmp keypad_scan_found
keypad_scan_check_col3:
	sbic PINA, 6 ; check row 2
	rjmp keypad_scan_check_col4
	ldi r23, 2 ; row 2 is pressed
	rjmp keypad_scan_found
keypad_scan_check_col4:
	sbic PINA, 7 ; check row 3
	rjmp keypad_scan_next_row
	ldi r23, 3 ; row 3 is pressed
	rjmp keypad_scan_found
keypad_scan_next_row:
	; check if all rows have been scanned
	cpi r24,0
	breq keypad_scan_not_found
	; shift row mask to scan next row
	ror r22
	dec r24 ;increase row index
	rjmp keypad_scan_loop
keypad_scan_found:
	; combine row and column to get key value (0-15)
	;key code = row*4 + col
	lsl r23 ; shift row value 4 bits to the left
	lsl r23
	add r23, r24 ; add row value to column value
	ret
keypad_scan_not_found:
	ldi r23, 0xFF ; no key pressed
	ret


DELAY_10MS:
	push R16
	push R17
	LDI R16,100
LOOP0:			;total ~400 cycle >> *200 = 80000 cycle * T(125ns) = 10ms
	LDI R17,200
LOOP1:			;total  ~4 cycle >>  * 100 = 400 cycle
	DEC R17		;1 cycle
	NOP			;1 cycle
	BRNE LOOP1	;~2 cycle
	DEC R16		;1 cycle
	BRNE LOOP0	;~2 cycle
	pop R17
	pop R16
	RET

DELAY_50MS:
	call DELAY_10MS
	call DELAY_10MS
	call DELAY_10MS
	call DELAY_10MS
	call DELAY_10MS
	ret


