;TO ENABLE CORRECT OPERATION OF RESET AND INTERRUPT VECTORS: --HIGHLY IMPORTANT--
   ;under Project Properties, pic-as Linker, under Custom linker options, include a copy of the following line:
   ;"-PRES_VECT=0x00;-PINT_VECT=0x04" 

PROCESSOR 16F886 ;Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/41291d.pdf
   
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
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
	
RPZERO	EQU 0x05	;refers to bit RP0 in the STATUS Register
INTFL	EQU 0x01	;interrupt 0 flag - INT0IF in INTCON
INTEN	EQU 0x04	;interrupt 0 enable - INT0IE in INTCON
GI	EQU 0x07	;global interrupt enable - GIE in INTCON

DEFPORTA EQU 00000001B
DEFPORTB EQU 10100110B
DEFPORTC EQU 01000110B
 
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
	bsf ADCON0,1	    ;start adc conversion
loopadc0:
	clrwdt		    ;Pat the watchdog
	btfsc ADCON0,1	    ;check if conversion finished
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
	bsf ADCON0,1	    ;start adc conversion
loopadc1:
	clrwdt		    ;Pat the watchdog
	btfsc ADCON0,1	    ;check if conversion finished
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
	bsf ADCON0,1	    ;start adc conversion
loopadc2:
	clrwdt		    ;Pat the watchdog
	btfsc ADCON0,1	    ;check if conversion finished
	goto loopadc2
	movf ADRESH,w
	movwf ADC2
	return

;================================ INITIALISATION ===============================

IOPORTA	EQU 11111110B ;set the PORTA data direction
IOPORTB	EQU 00000001B ;set the PORTB data direction
IOPORTC EQU 00000000B ;set the PORTC data direction
IOPORTE EQU 1000B ;set the PORTE data direction. PORTE has only 4 bits.
	
;PORTB,0 is the default interrupt trigger - ensure this is set as 1 if used as such.
;PORTA,5 must ALWAYS be an input.
;PORTA are all usable as analog inputs.
;PORTE,3 must be an INPUT - MCLR

start:
    clrf PORTA
    clrf PORTB
    clrf PORTC
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
    movlw IOPORTE
    movwf TRISE
    movlw 00000000B	;Vref is power supply voltage & left justified
    movwf ADCON1	;configured ready to use readadc subroutines
    bcf STATUS,RPZERO	;change RP0 to 0 to select BANK0
    
;uncomment the next lines to enable interrupt routines
    ;bsf INTCON,INTEN	;set the external interrupt enable
    ;bsf INTCON,GI	;enable all interruptions
    
main:
    movlw DEFPORTA
    movwf PORTA
    movlw DEFPORTB
    movwf PORTB
    movlw DEFPORTC
    movwf PORTC
smileyloop:
    call triggerB3
    call triggerB6
    call triggerC3
    call triggerC6
    call triggerD3
    call triggerD6
    call triggerF2
    call triggerF7
    call triggerG3
    call triggerG4
    call triggerG5
    call triggerG6
    goto smileyloop

triggerA1:
    bsf PORTC,3
    bcf PORTB,5
    call wait10ms
    bcf PORTC,3
    bsf PORTB,5
    return
triggerA2:
    bsf PORTC,3
    bcf PORTB,2
    call wait10ms
    bcf PORTC,3
    bsf PORTB,2
    return
triggerA3:
    bsf PORTC,3
    bcf PORTB,1
    call wait10ms
    bcf PORTC,3
    bsf PORTB,1
    return
triggerA4:
    bsf PORTC,3
    bcf PORTC,2
    call wait10ms
    bcf PORTC,3
    bsf PORTC,2
    return
triggerA5:
    bsf PORTC,3
    bcf PORTC,6
    call wait10ms
    bcf PORTC,3
    bsf PORTC,6
    return
triggerA6:
    bsf PORTC,3
    bcf PORTC,1
    call wait10ms
    bcf PORTC,3
    bsf PORTC,1
    return
triggerA7:
    bsf PORTC,3
    bcf PORTB,7
    call wait10ms
    bcf PORTC,3
    bsf PORTB,7
    return
triggerA8:
    bsf PORTC,3
    bcf PORTA,0
    call wait10ms
    bcf PORTC,3
    bsf PORTA,0
    return
triggerB1:
    bsf PORTB,6
    bcf PORTB,5
    call wait10ms
    bcf PORTB,6
    bsf PORTB,5
    return
triggerB2:
    bsf PORTB,6
    bcf PORTB,2
    call wait10ms
    bcf PORTB,6
    bsf PORTB,2
    return
triggerB3:
    bsf PORTB,6
    bcf PORTB,1
    call wait10ms
    bcf PORTB,6
    bsf PORTB,1
    return
triggerB4:
    bsf PORTB,6
    bcf PORTC,2
    call wait10ms
    bcf PORTB,6
    bsf PORTC,2
    return
triggerB5:
    bsf PORTB,6
    bcf PORTC,6
    call wait10ms
    bcf PORTB,6
    bsf PORTC,6
    return
triggerB6:
    bsf PORTB,6
    bcf PORTC,1
    call wait10ms
    bcf PORTB,6
    bsf PORTC,1
    return
triggerB7:
    bsf PORTB,6
    bcf PORTB,7
    call wait10ms
    bcf PORTB,6
    bsf PORTB,7
    return
triggerB8:
    bsf PORTB,6
    bcf PORTA,0
    call wait10ms
    bcf PORTB,6
    bsf PORTA,0
    return
triggerC1:
    bsf PORTC,4
    bcf PORTB,5
    call wait10ms
    bcf PORTC,4
    bsf PORTB,5
    return
triggerC2:
    bsf PORTC,4
    bcf PORTB,2
    call wait10ms
    bcf PORTC,4
    bsf PORTB,2
    return
triggerC3:
    bsf PORTC,4
    bcf PORTB,1
    call wait10ms
    bcf PORTC,4
    bsf PORTB,1
    return
triggerC4:
    bsf PORTC,4
    bcf PORTC,2
    call wait10ms
    bcf PORTC,4
    bsf PORTC,2
    return
triggerC5:
    bsf PORTC,4
    bcf PORTC,6
    call wait10ms
    bcf PORTC,4
    bsf PORTC,6
    return
triggerC6:
    bsf PORTC,4
    bcf PORTC,1
    call wait10ms
    bcf PORTC,4
    bsf PORTC,1
    return
triggerC7:
    bsf PORTC,4
    bcf PORTB,7
    call wait10ms
    bcf PORTC,4
    bsf PORTB,7
    return
triggerC8:
    bsf PORTC,4
    bcf PORTA,0
    call wait10ms
    bcf PORTC,4
    bsf PORTA,0
    return
triggerD1:
    bsf PORTC,0
    bcf PORTB,5
    call wait10ms
    bcf PORTC,0
    bsf PORTB,5
    return
triggerD2:
    bsf PORTC,0
    bcf PORTB,2
    call wait10ms
    bcf PORTC,0
    bsf PORTB,2
    return
triggerD3:
    bsf PORTC,0
    bcf PORTB,1
    call wait10ms
    bcf PORTC,0
    bsf PORTB,1
    return
triggerD4:
    bsf PORTC,0
    bcf PORTC,2
    call wait10ms
    bcf PORTC,0
    bsf PORTC,2
    return
triggerD5:
    bsf PORTC,0
    bcf PORTC,6
    call wait10ms
    bcf PORTC,0
    bsf PORTC,6
    return
triggerD6:
    bsf PORTC,0
    bcf PORTC,1
    call wait10ms
    bcf PORTC,0
    bsf PORTC,1
    return
triggerD7:
    bsf PORTC,0
    bcf PORTB,7
    call wait10ms
    bcf PORTC,0
    bsf PORTB,7
    return
triggerD8:
    bsf PORTC,0
    bcf PORTA,0
    call wait10ms
    bcf PORTC,0
    bsf PORTA,0
    return
triggerE1:
    bsf PORTB,4
    bcf PORTB,5
    call wait10ms
    bcf PORTB,4
    bsf PORTB,5
    return
triggerE2:
    bsf PORTB,4
    bcf PORTB,2
    call wait10ms
    bcf PORTB,4
    bsf PORTB,2
    return
triggerE3:
    bsf PORTB,4
    bcf PORTB,1
    call wait10ms
    bcf PORTB,4
    bsf PORTB,1
    return
triggerE4:
    bsf PORTB,4
    bcf PORTC,2
    call wait10ms
    bcf PORTB,4
    bsf PORTC,2
    return
triggerE5:
    bsf PORTB,4
    bcf PORTC,6
    call wait10ms
    bcf PORTB,4
    bsf PORTC,6
    return
triggerE6:
    bsf PORTB,4
    bcf PORTC,1
    call wait10ms
    bcf PORTB,4
    bsf PORTC,1
    return
triggerE7:
    bsf PORTB,4
    bcf PORTB,7
    call wait10ms
    bcf PORTB,4
    bsf PORTB,7
    return
triggerE8:
    bsf PORTB,4
    bcf PORTA,0
    call wait10ms
    bcf PORTB,4
    bsf PORTA,0
    return
triggerF1:
    bsf PORTC,5
    bcf PORTB,5
    call wait10ms
    bcf PORTC,5
    bsf PORTB,5
    return
triggerF2:
    bsf PORTC,5
    bcf PORTB,2
    call wait10ms
    bcf PORTC,5
    bsf PORTB,2
    return
triggerF3:
    bsf PORTC,5
    bcf PORTB,1
    call wait10ms
    bcf PORTC,5
    bsf PORTB,1
    return
triggerF4:
    bsf PORTC,5
    bcf PORTC,2
    call wait10ms
    bcf PORTC,5
    bsf PORTC,2
    return
triggerF5:
    bsf PORTC,5
    bcf PORTC,6
    call wait10ms
    bcf PORTC,5
    bsf PORTC,6
    return
triggerF6:
    bsf PORTC,5
    bcf PORTC,1
    call wait10ms
    bcf PORTC,5
    bsf PORTC,1
    return
triggerF7:
    bsf PORTC,5
    bcf PORTB,7
    call wait10ms
    bcf PORTC,5
    bsf PORTB,7
    return
triggerF8:
    bsf PORTC,5
    bcf PORTA,0
    call wait10ms
    bcf PORTC,5
    bsf PORTA,0
    return    
triggerG1:
    bsf PORTB,3
    bcf PORTB,5
    call wait10ms
    bcf PORTB,3
    bsf PORTB,5
    return
triggerG2:
    bsf PORTB,3
    bcf PORTB,2
    call wait10ms
    bcf PORTB,3
    bsf PORTB,2
    return
triggerG3:
    bsf PORTB,3
    bcf PORTB,1
    call wait10ms
    bcf PORTB,3
    bsf PORTB,1
    return
triggerG4:
    bsf PORTB,3
    bcf PORTC,2
    call wait10ms
    bcf PORTB,3
    bsf PORTC,2
    return
triggerG5:
    bsf PORTB,3
    bcf PORTC,6
    call wait10ms
    bcf PORTB,3
    bsf PORTC,6
    return
triggerG6:
    bsf PORTB,3
    bcf PORTC,1
    call wait10ms
    bcf PORTB,3
    bsf PORTC,1
    return
triggerG7:
    bsf PORTB,3
    bcf PORTB,7
    call wait10ms
    bcf PORTB,3
    bsf PORTB,7
    return
triggerG8:
    bsf PORTB,3
    bcf PORTA,0
    call wait10ms
    bcf PORTB,3
    bsf PORTA,0
    return
triggerH1:
    bsf PORTC,7
    bcf PORTB,5
    call wait10ms
    bcf PORTC,7
    bsf PORTB,5
    return
triggerH2:
    bsf PORTC,7
    bcf PORTB,2
    call wait10ms
    bcf PORTC,7
    bsf PORTB,2
    return
triggerH3:
    bsf PORTC,7
    bcf PORTB,1
    call wait10ms
    bcf PORTC,7
    bsf PORTB,1
    return
triggerH4:
    bsf PORTC,7
    bcf PORTC,2
    call wait10ms
    bcf PORTC,7
    bsf PORTC,2
    return
triggerH5:
    bsf PORTC,7
    bcf PORTC,6
    call wait10ms
    bcf PORTC,7
    bsf PORTC,6
    return
triggerH6:
    bsf PORTC,7
    bcf PORTC,1
    call wait10ms
    bcf PORTC,7
    bsf PORTC,1
    return
triggerH7:
    bsf PORTC,7
    bcf PORTB,7
    call wait10ms
    bcf PORTC,7
    bsf PORTB,7
    return
triggerH8:
    bsf PORTC,7
    bcf PORTA,0
    call wait10ms
    bcf PORTC,7
    bsf PORTA,0
    return
    
END