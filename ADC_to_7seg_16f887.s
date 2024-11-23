;TO ENABLE CORRECT OPERATION OF RESET AND INTERRUPT VECTORS: --HIGHLY IMPORTANT--
   ;under Project Properties, pic-as Linker, under Custom linker options, include a copy of the following line:
   ;"-PRES_VECT=0x00;-PINT_VECT=0x04" 

PROCESSOR 16F887 ;Datasheet: https://ww1.microchip.com/downloads/en/devicedoc/41291d.pdf
   
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = ON            ; RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = ON            ; Brown Out Reset Selection bits (BOR enabled)
  CONFIG  IESO = ON             ; Internal External Switchover bit (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)
   
#include <xc.inc>
   
CLONE	EQU 0x20	;value used to refer to memory store in interrupt routine

WAIT1	EQU 0x21	;values used to refer to memory stores used in wait delays
WAIT10	EQU 0x22	
WAIT100	EQU 0x23	
WAIT1k	EQU 0x24
	
ADCTEMP	EQU 0x31	;variables used to point to memory stores for the readadc routines
ADC0	EQU 0x32
ADC1	EQU 0x33
ADC2	EQU 0x34
	
BINARY_INPUT EQU 0x40
NUM_100s    EQU 0x41
NUM_10s	EQU 0x42
NUM_1s	EQU 0x43
	
RPZERO	EQU 0x05	;refers to bit RP0 in the STATUS Register
INTFL	EQU 0x01	;interrupt 0 flag - INT0IF in INTCON
INTEN	EQU 0x04	;interrupt 0 enable - INT0IE in INTCON
GI	EQU 0x07	;global interrupt enable - GIE in INTCON
	
;NUM	EQU abcdefg0B
NINE	EQU 11110110B
EIGHT	EQU 11111110B
SEVEN	EQU 11100000B
SIX	EQU 10111110B
FIVE	EQU 10110110B
FOUR	EQU 01100110B
THREE	EQU 11110010B
TWO	EQU 11011010B
ONE	EQU 01100000B
ZER0	EQU 11111100B
	
;================================== RESET VECTORS  =============================	
psect RES_VECT,class=CODE,delta=2 ; PIC10/12/16
RES_VECT:
    nop
    goto start
    
PSECT INT_VECT,class=CODE,delta=2
INT_VECT:
    movwf CLONE		    ;create a copy of the current working register
    btfss INTCON,INTFL	    ;check interrupt has occured, skips if true
    retfie		    ;set GI and RETURN in a single clock cycle
    call your_interrupt
    bcf INTCON,INTFL	    ;reset the interrupt flag
    movf CLONE,0x00	    ;return the saved working register back for use (working register address 0x00)
    retfie

your_interrupt:    
    nop
    return

psect code
    
wait1ms:    ;@4MHz, the exe rate = 1MIP, t=1us. 1000 total instructions needed to make 1ms.
    movlw 255	    ;+1 instruction
    movwf WAIT1	    ;+1
loop1ms:
	decfsz WAIT1,1	;+1*255
	goto loop1ms	;+2(255-1)
	movlw 77		;+1
	movwf WAIT1		;+1
loop1ms2:
	decfsz WAIT1,1	;+1*77
	goto loop1ms2	;+2(77-1)
	nop		;+1
	nop		;+1
	return	;+2 = 1000 total instructions

wait10ms: ;requires 10,000 instructions
	movlw 10	;+1
	movwf WAIT10	;+1
loop10ms:
	call wait1ms ;10*1000 instructions
	decfsz WAIT10,1 ;+10
	goto loop10ms ;+2*9
	return ;+2 = 10,030 total

wait100ms: ;requires 100,000 instructions
	movlw 100	    ;+1
	movwf WAIT100	    ;+1
loop100ms:
	call wait1ms	    ;+100*1000
	decfsz WAIT100,1    ;+100
	goto loop100ms	    ;+99*2
	return		    ;+2 = 100,302

wait1000ms:		;requires 1,000,000 instructions
	movlw 10	;+1
	movwf WAIT1k	;+1
loop1000ms:
	call wait100ms	;+10*100,302
	decfsz WAIT1k,1	;+10
	goto loop1000ms	;+2*9
	return		;+2 = 1,003,052
readadc0:
	movlw 00000001B	    ;PORTA,0
	bsf STATUS,RPZERO
	movwf ANSEL
	bcf STATUS,RPZERO
	movlw 01000001B	    ;01XXX001, three blank bits tell ADCON0 which analog input to check
	movwf ADCON0
	bsf ADCON0,2	    ;start adc conversion
loopadc0:
	clrwdt		    ;Pat the watchdog
	btfsc ADCON0,2	    ;check if conversion finished
	goto loopadc0
	movf ADRESH,w	    ;take result from ADRESH
	movwf ADC0	    ;move result to ADC0
	return

readadc1:
	movlw 00000010B	    ;PORTA,1
	bsf STATUS,RPZERO
	movwf ANSEL
	bcf STATUS,RPZERO
	movlw 01001001B	    ;01XXX001, three blank bits tell ADCON0 which analog input to check 
	movwf ADCON0
	bsf ADCON0,2	    ;start adc conversion
loopadc1:
	clrwdt		    ;Pat the watchdog
	btfsc ADCON0,2	    ;check if conversion finished
	goto loopadc1
	movf ADRESH,w
	movwf ADC1
	return
	
readadc2:
	movlw 00000100B	    ;PORTA,2
	bsf STATUS,RPZERO
	movwf ANSEL
	bcf STATUS,RPZERO
	movlw 01010001B	    ;01XXX001, three blank bits tell ADCON0 which analog input to check 
	movwf ADCON0
	bsf ADCON0,2	    ;start adc conversion
loopadc2:
	clrwdt		    ;Pat the watchdog
	btfsc ADCON0,2	    ;check if conversion finished
	goto loopadc2
	movf ADRESH,w
	movwf ADC2
	return

;================================ INITIALISATION ===============================
;7 seg connections for PORTB,C,D: abcdefg0B - the LSB is not used
	
IOPORTA	EQU 00100001B ;set the PORTA data direction. PORTA,0 = ADC.
IOPORTB	EQU 00000001B ;set the PORTB data direction. PORTB,0 must be 1. 
IOPORTC EQU 00000000B ;set the PORTC data direction
IOPORTD EQU 00000000B ;set the PORTD data direction
IOPORTE EQU 00001000B ;set the PORTE data direction. PORTE has only 4 usable bits.
	
;PORTB,0 is the default interrupt trigger - ensure this is set as 1 if used as such.
;PORTA,5 must ALWAYS be an input.
;PORTA are all usable as analog inputs.
;PORTE,3 must be an INPUT.

start:
    clrf PORTA
    clrf PORTB
    clrf PORTC
    clrf PORTD
    clrf PORTE
    bsf STATUS,RPZERO	;change RP0 to 1 to select BANK1
    movlw 01100000B	;set the clock speed to 4MHz https://docs.rs-online.com/a168/0900766b81382af8.pdf pg40
    movwf OSCCON 	;move the new clock speed to the clock controller.	
    clrf ANSEL		;disable adc
    movlw IOPORTA	;call on the decided Inputs/outputs for PORTA
    movwf TRISA		
    movlw IOPORTB	;call on the decided i/o for PORTB
    movwf TRISB
    movlw IOPORTC
    movwf TRISC
    movlw IOPORTD
    movwf TRISD
    movlw IOPORTE
    movwf TRISE
    movlw 00000000B	;Vref is power supply voltage & left justified
    movwf ADCON1	;configured ready to use readadc subroutines
    bcf STATUS,RPZERO	;change RP0 to 0 to select BANK0
    
;uncomment the next lines to enable interrupt routines
    ;bsf INTCON,INTEN	;set the external interrupt enable
    ;bsf INTCON,GI	;enable all interruptions
    
main:
    ;call readadc0 
    ;movf ADC0,w
    movlw 217
    movwf BINARY_INPUT
    bsf STATUS,0
    clrf NUM_100s
    clrf NUM_10s
    clrf NUM_1s
dec_100_loop:
    movlw 100
    subwf BINARY_INPUT	    ;subtract 100 from value
    btfss STATUS,0	    ;check if is <0 - check carry flag in STATUS register
    goto reset_value_100s   ;if is <0, move on to test num 10s
    incf NUM_100s	    ;else +1 to 100s counter
    goto dec_100_loop	    ;repeat to test again
reset_value_100s:
    movlw 100
    addwf BINARY_INPUT
dec_10_loop:
    movlw 10
    subwf BINARY_INPUT	    ;subtract 10 from the remainder value after removing 100s
    btfss STATUS,0	    ;check if <0
    goto reset_value_10s    ;if <0, move on to get num 1s
    incf NUM_10s	    ;else +1 to 10s counter
    goto dec_10_loop	    ;repeat to test 10s again
reset_value_10s:
    movlw 10
    addwf BINARY_INPUT
units:
    movf BINARY_INPUT,w	    ;put number of units into its own memory
    movwf NUM_1s
    
check_100s_digit_is_2:
    movf NUM_100s,w
    sublw 2
    btfss STATUS,2
    goto check_100s_digit_is_1
    movlw TWO
    movwf PORTB
    goto check_10s_digit_is_9
    
check_100s_digit_is_1:
    movf NUM_100s,w
    sublw 1
    btfss STATUS,2
    goto check_100s_digit_is_0
    movlw ONE
    movwf PORTB
    goto check_10s_digit_is_9
    
check_100s_digit_is_0:
    movf NUM_100s,w
    sublw 0
    btfss STATUS,2
    goto error_occurred
    movlw ZER0
    movwf PORTB
    goto check_10s_digit_is_9
    
check_10s_digit_is_9:
    movf NUM_10s,w
    sublw 9
    btfss STATUS,2
    goto check_10s_digit_is_8
    movlw NINE
    movwf PORTC
    goto check_1s_digit_is_9

check_10s_digit_is_8:
    movf NUM_10s,w
    sublw 8
    btfss STATUS,2
    goto check_10s_digit_is_7
    movlw EIGHT
    movwf PORTC
    goto check_1s_digit_is_9

check_10s_digit_is_7:
    movf NUM_10s,w
    sublw 7
    btfss STATUS,2
    goto check_10s_digit_is_6
    movlw SEVEN
    movwf PORTC
    goto check_1s_digit_is_9
    
check_10s_digit_is_6:
    movf NUM_10s,w
    sublw 6
    btfss STATUS,2
    goto check_10s_digit_is_5
    movlw SIX
    movwf PORTC
    goto check_1s_digit_is_9
    
check_10s_digit_is_5:
    movf NUM_10s,w
    sublw 5
    btfss STATUS,2
    goto check_10s_digit_is_4
    movlw FIVE
    movwf PORTC
    goto check_1s_digit_is_9
    
check_10s_digit_is_4:
    movf NUM_10s,w
    sublw 4
    btfss STATUS,2
    goto check_10s_digit_is_3
    movlw FOUR
    movwf PORTC
    goto check_1s_digit_is_9

check_10s_digit_is_3:
    movf NUM_10s,w
    sublw 3
    btfss STATUS,2
    goto check_10s_digit_is_2
    movlw THREE
    movwf PORTC
    goto check_1s_digit_is_9
    
check_10s_digit_is_2:
    movf NUM_10s,w
    sublw 2
    btfss STATUS,2
    goto check_10s_digit_is_1
    movlw TWO
    movwf PORTC
    goto check_1s_digit_is_9
    
check_10s_digit_is_1:
    movf NUM_10s,w
    sublw 1
    btfss STATUS,2
    goto check_10s_digit_is_0
    movlw ONE
    movwf PORTC
    goto check_1s_digit_is_9
    
check_10s_digit_is_0:
    movf NUM_10s,w
    sublw 0
    btfss STATUS,2
    goto error_occurred
    movlw ZER0
    movwf PORTC
    goto check_1s_digit_is_9

check_1s_digit_is_9:
    movf NUM_1s,w
    sublw 9
    btfss STATUS,2
    goto check_1s_digit_is_8
    movlw NINE
    movwf PORTD
    goto finished

check_1s_digit_is_8:
    movf NUM_1s,w
    sublw 8
    btfss STATUS,2
    goto check_1s_digit_is_7
    movlw EIGHT
    movwf PORTD
    goto finished

check_1s_digit_is_7:
    movf NUM_1s,w
    sublw 7
    btfss STATUS,2
    goto check_1s_digit_is_6
    movlw SEVEN
    movwf PORTD
    goto finished
    
check_1s_digit_is_6:
    movf NUM_1s,w
    sublw 6
    btfss STATUS,2
    goto check_1s_digit_is_5
    movlw SIX
    movwf PORTD
    goto finished
    
check_1s_digit_is_5:
    movf NUM_1s,w
    sublw 5
    btfss STATUS,2
    goto check_1s_digit_is_4
    movlw FIVE
    movwf PORTD
    goto finished
    
check_1s_digit_is_4:
    movf NUM_1s,w
    sublw 4
    btfss STATUS,2
    goto check_1s_digit_is_3
    movlw FOUR
    movwf PORTD
    goto finished

check_1s_digit_is_3:
    movf NUM_10s,w
    sublw 3
    btfss STATUS,2
    goto check_1s_digit_is_2
    movlw THREE
    movwf PORTD
    goto finished
    
check_1s_digit_is_2:
    movf NUM_1s,w
    sublw 2
    btfss STATUS,2
    goto check_1s_digit_is_1
    movlw TWO
    movwf PORTD
    goto finished
    
check_1s_digit_is_1:
    movf NUM_1s,w
    sublw 1
    btfss STATUS,2
    goto check_1s_digit_is_0
    movlw ONE
    movwf PORTD
    goto finished
    
check_1s_digit_is_0:
    movf NUM_1s,w
    sublw 0
    btfss STATUS,2
    goto error_occurred
    movlw ZER0
    movwf PORTD
    goto finished
    
finished:
    call wait1000ms
    call wait1000ms
    goto main
    
error_occurred:
    movlw 11111110B
    movwf PORTB
    movwf PORTC
    movwf PORTD
    call wait1000ms
    call wait1000ms
    call wait1000ms
    call wait1000ms
    goto main
    
END