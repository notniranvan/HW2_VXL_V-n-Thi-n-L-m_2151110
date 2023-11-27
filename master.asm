; HW2 MASTER
.org 0x0000
rjmp rst_handle
.org 0x0002
rjmp INT0_handle

; Replace with your application code
rst_handle:
    call SPI_inital
	call USART_inital
	call INT_inital
	call LCD_Init
	sei
main:
    rjmp main


SPI_inital:
	push r16
	// SPI MOSI and SCK and SS output port
	ldi r16, (1<<PB4)|(1<<PB5)|(1<<PB7)
	out DDRB, r16
	ldi r16, (1<<SPE0)|(1<<MSTR0)|(1<<SPR00)
	out SPCR0,r16
	// inital SS
	sbi PORTB,4
	pop r16
	ret

USART_inital:
	push r16
	; Set baud rate to 9600 bps with 1 MHz clock
	ldi r16, 103
	sts UBRR0L, r16
	;set double speed
	ldi r16, (1 << U2X0)
	sts UCSR0A, r16
	; Set frame format: 8 data bits, no parity, 1 stop bit
	ldi r16, (1 << UCSZ01) | (1 << UCSZ00)
	sts UCSR0C, r16
	; Enable  transmit 
	ldi r16, (1 << TXEN0) 
	sts UCSR0B, r16
	pop r16
	ret


;send out 1 byte in r20
USART_SendChar:
	push r16
	; Wait for the transmitter to be ready
USART_SendChar_Wait:
	lds r16, UCSR0A
	sbrs r16, UDRE0 ;check USART Data Register Empty bit
	rjmp USART_SendChar_Wait
	sts UDR0, r20 ;send out
	pop r16
	ret


INT_inital:
	push R16
	ldi r16,(1<<ISC01)
	sts EICRA,R16
	sbi portd,2
	// INT0 enable, falling active
	ldi R16, (1<<INT0)
	out EIMSK, R16
	// busy signal PB0
	SBI DDRB,0
	SBI PORTB,0	;high when idle

	pop R16
	ret

INT0_handle:
	push r16
	push r19
	ldi r19,0
	cbi PORTB,0		; LOW PB0 when working INT	 
	cbi portb,4		; enable slave device
	nop
	nop
	nop
	ldi r16,0x00	; load X to get data
	out SPDR0,r16	
Wait_SPI_send_X:
	in R16,SPSR0
	sbrs r16, SPIF0
	rjmp Wait_SPI_send_X
	sbi portb,4		;disable slave device
	in r20,SPDR0	; get data

	ldi ZH,high(table*2)
	ldi ZL,low(table*2)
	add ZL,r20
	adc ZH,r19
	lpm r20,Z
	call USART_SendChar
	mov R16,r20
	call LCD_Send_Data
	ldi r16,0
	ldi r17,0
	call LCD_Move_Cursor
	sbi PORTB,0		// HIGH PB0 when done/idle
	pop r19
	pop r16
	reti
.CSEG 
table: .db  '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'


 //////////////////////////LCD//////////

 ;Command code in r16 

;LCD_D7..LCD_D4 connect to PA7..PA4 

;LCD_RS connect to PA0 

;LCD_RW connect to PA1 

;LCD_EN connect to PA2 

LCD_Send_Command: 

push r17 

call LCD_wait_busy ; check if LCD is busy  

mov r17,r16 ;save the command 
; Set RS low to select command register 
; Set RW low to write to LCD 
andi r17,0xF0 
; Send command to LCD 
out LCDPORT, r17  
nop 
nop 
; Pulse enable pin 
sbi LCDPORT, LCD_EN 
nop 
nop 
cbi LCDPORT, LCD_EN 
swap r16 
andi r16,0xF0 
; Send command to LCD 
out LCDPORT, r16  
; Pulse enable pin 
sbi LCDPORT, LCD_EN 
nop 
nop 
cbi LCDPORT, LCD_EN 
pop r17 
ret 

 

 

 //////////////////////////////////////////// Subroutine to send data to LCD/////////////////////////////////////////////////// 

LCD_Send_Data: 
push r16
push r17 
call LCD_wait_busy ;check if LCD is busy 
mov r17,r16 ;save the command 

; Set RS high to select data register 

; Set RW low to write to LCD 

andi r17,0xF0 
ori r17,0x01 

; Send data to LCD 
out LCDPORT, r17  
nop 

; Pulse enable pin 

sbi LCDPORT, LCD_EN 
nop 
cbi LCDPORT, LCD_EN 

; Delay for command execution 

;send the lower nibble 
nop 
swap r16 
andi r16,0xF0 

; Set RS high to select data register 

; Set RW low to write to LCD 
andi r16,0xF0 
ori r16,0x01 

; Send command to LCD 
out LCDPORT, r16 
nop 

; Pulse enable pin 
sbi LCDPORT, LCD_EN 
nop 
cbi LCDPORT, LCD_EN 
pop r17 
pop r16
ret 

 

 

 //////////////////////////////////////////// Subroutine to check busy flag /////////////////////////////////////////////////// 

LCD_wait_busy: 
push r16 
ldi r16, 0b00000111 ; set PA7-PA4 as input, PA2-PA0 as output 
out LCDPORTDIR, r16 
ldi r16,0b11110010 ; set RS=0, RW=1 for read the busy flag 
out LCDPORT, r16 
nop 
LCD_wait_busy_loop: 
sbi LCDPORT, LCD_EN 
nop 
nop 
in r16, LCDPORTPIN 
cbi LCDPORT, LCD_EN 
nop 
sbi LCDPORT, LCD_EN 
nop 
nop 
cbi LCDPORT, LCD_EN 
nop
andi r16,0x80 
cpi r16,0x80 
breq LCD_wait_busy_loop 
ldi r16, 0b11110111 ; set PA7-PA4 as output, PA2-PA0 as output 
out LCDPORTDIR, r16 
ldi r16,0b00000000 ; set RS=0, RW=1 for read the busy flag 
out LCDPORT, r16 
pop r16 
ret 


 
 
 ; Function to move the cursor to a specific position on the LCD 
; Assumes that the LCD is already initialized 
; Input: Row number in R16 (0-based), Column number in R17 (0-based) 
LCD_Move_Cursor: 
cpi r16,0 ;check if first row 
brne LCD_Move_Cursor_Second 
andi r17, 0x0F 
ori r17,0x80  
mov r16,r17 
; Send command to LCD 
call LCD_Send_Command 
ret 
LCD_Move_Cursor_Second: 
cpi r16,1 ;check if second row 
brne LCD_Move_Cursor_Exit ;else exit  
andi r17, 0x0F 
ori r17,0xC0  
mov r16,r17  
; Send command to LCD 
call LCD_Send_Command 
LCD_Move_Cursor_Exit: 
; Return from function 

ret 
; Replace with your application code 
;init the LCD 
;LCD_D7..LCD_D4 connect to PA7..PA4 
;LCD_RS connect to PA0 
;LCD_RW connect to PA1 
;LCD_EN connect to PA2 
.equ LCDPORT = PORTA ; Set signal port reg to PORTA 
.equ LCDPORTDIR = DDRA ; Set signal port dir reg to PORTA 
.equ LCDPORTPIN = PINA ; Set clear signal port pin reg to PORTA 
.equ LCD_RS = PINA0 
.equ LCD_RW = PINA1 
.equ LCD_EN = PINA2 
.equ LCD_D7 = PINA7 
.equ LCD_D6 = PINA6 
.equ LCD_D5 = PINA5 
.equ LCD_D4 = PINA4 


LCD_Init:
; Set up data direction register for Port A 
ldi r16, 0b11110111 ; set PA7-PA4 as outputs, PA2-PA0 as output 
out LCDPORTDIR, r16 
; Wait for LCD to power up 
call DELAY_10MS 
call DELAY_10MS 
; Send initialization sequence 
ldi r16, 0x02 ; Function Set: 4-bit interface 
call LCD_Send_Command 
ldi r16, 0x28 ; Function Set: enable 5x7 mode for chars  
call LCD_Send_Command 
ldi r16, 0x0E ; Display Control: Display OFF, Cursor ON 
call LCD_Send_Command 
ldi r16, 0x01 ; Clear Display 
call LCD_Send_Command 
ldi r16, 0x80 ; Clear Display 
call LCD_Send_Command 
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
