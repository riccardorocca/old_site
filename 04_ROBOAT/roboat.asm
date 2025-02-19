* Circuit Cellar - Design99 - Project number: 9908
* Program "ROBOAT"
* Author: Riccardo Rocca
* e-mail: riccardo.rocca@iol.it
* http://users.iol.it/riccardo.rocca
* Date: May 1999
*
* Assembler for MC68HC908GP20CFB.
* The program controls a boat model that sails on its own, following a
* pre-planned course with the help of a GPS and a Digital Compass.
*
* Pins assignments (see schematic layout):
* - PTA1: SW5 - disables GPS course updates
* - PTA2: SW4 - disables Compass course updates
* - PTA3: SW3 - copy the present Compass course into the GPS course
* - PTB0: analog input to adjust the central position of the rudder
* - PTB1: analog input from digital compass hall sensor 1
* - PTB2: analog input from digital compass hall sensor 2
* - PTC2: output, switches the electric motor and propeller (1=off, 0=on)
* - PTC4: output, powers the GPS (1=down, 0=up)
* - PTD4: output, controls the servo prop. and rudder
* - PTD6: output, blinks the red led when acquires correct data from GPS
* - PTE1: input, receives serial data from GPS

*============================================================

* Memory Map:
RAMStart     EQU  $0040
RAMEnd+1     EQU  $0240
FLASHStart   EQU  $B000
PROGStart    EQU  $C000

*============================================================

* Vectors Addresses:
 org $FFDC
 FDB $0000   ;    IF16 - Timebase
 FDB IntADC  ;    IF15 - ADC Conversion Complete
 FDB IntKBD  ;    IF14 - Keyboard
 FDB $0000   ;    IF13 - SCI Transmit
 FDB IntSCI  ;    IF12 - SCI Receive
 FDB $0000   ;    IF11 - SCI Error
 FDB $0000   ;    IF10 - SPI Transmit
 FDB $0000   ;    IF9  - SPI Receive
 FDB $0000   ;    IF8  - TIM2 Overflow
 FDB $0000   ;    IF7  - TIM2 Channel 1
 FDB $0000   ;    IF6  - TIM2 Channel 0
 FDB IntTIM1 ;    IF5  - TIM1 Overflow
 FDB $0000   ;    IF4  - TIM1 Channel 1
 FDB $0000   ;    IF3  - TIM1 Channel 0
 FDB $0000   ;    IF2  - PLL
 FDB $0000   ;    IF1  - IRQ
 FDB $0000   ;           SWI
 FDB Init    ;           Reset

*============================================================

* MC68HC908GP20 Control, Status and Data Registers
PTA      EQU $0000    ; Ports and data direction
PTC      EQU $0002
PTD      EQU $0003
DDRC     EQU $0006
DDRD     EQU $0007
PTAPUE   EQU $000D    ; Port pull-up enables
SCC1     EQU $0013    ; SCI (Asyncronous communications)
SCC2     EQU $0014
SCS1     EQU $0016
SCDR     EQU $0018
SCBR     EQU $0019
INTKBSCR EQU $001a    ; Keyboard interrupt control/status
INTKBIER EQU $001b
T1SC     EQU $0020    ; Timer 1
T1MODH   EQU $0023
T1MODL   EQU $0024
T1SC0    EQU $0025
T1CH0H   EQU $0026
T1CH0L   EQU $0027
T1SC1    EQU $0028
T1CH1H   EQU $0029
T1CH1L   EQU $002a
T2SC     EQU $002b    ; Timer 2
T2CNTH   EQU $002c
T2CNTL   EQU $002d
T2MODH   EQU $002e
T2MODL   EQU $002f
T2SC0    EQU $0030
T2CH0H   EQU $0031
T2CH0L   EQU $0032
ADSCR    EQU $003C    ; AD converter
ADR      EQU $003D
ADCLK    EQU $003E

*============================================================

* Program Registers
 org RAMStart

REGA    RMB 4    ; four bytes register A
REGB    RMB 4    ; four bytes register B
REGC    RMB 4    ; four bytes register C
REGD    RMB 4    ; four bytes register D
GPSMSG  RMB !70  ; space for the ASCII message from GPS
GPSHX   RMB 2    ; address where to store the next character from GPS
GPSCK   RMB 1    ; GPS checksum byte
NLL     RMB 1    ; number of the last reached waypoint (range: 1 - TOTLL)
LONG    RMB 4    ; longitude from GPS (expressed in minutes/10000)
LAT     RMB 4    ; latitude from GPS (expressed in minutes/10000)
X0      RMB 4    ; X relative coordinate computed from GPS longitude
Y0      RMB 4    ; Y relative coordinate computed from GPS latitude
X1      RMB 4    ; X relative coordinate of the next waypoint
Y1      RMB 4    ; Y relative coordinate of the next waypoint
DX      RMB 4    ; X range, |X1 - X0|
DY      RMB 4    ; Y range, |Y1 - Y0|
ATANKEY RMB 1    ; keyword for computing atan
ATANI   RMB 1    ; pointer to select keyword in ATANTAB
CRSGPS  RMB 1    ; course to the next waypoint computed with GPS data
CRSCMPS RMB 1    ; present course computed with compass data
ADC0    RMB 1    ; analog data from rudder trimmer
ADC1    RMB 1    ; analog data from digital compass sensor 1
ADC2    RMB 1    ; analog data from digital compass sensor 2

*============================================================

* Data for the planned course
 org FLASHStart
XCF     FDB !2625 ; X/LONG ratio, meters / (minutes/10000)
YCF     FDB !3650 ; Y/LAT ratio, meters / (minutes/10000)
RANGE   FDB !50   ; tolerance to accept a waypoint when reached
TOTLL   FCB 4     ; total number of waypoints
*           Longitude    Latitude
LLDATA  FDB $0055,$0AF5,$01A0,$36BA
        FDB $0055,$0A0F,$01A0,$2ECF
        FDB $0055,$0C76,$01A0,$2686
        FDB $0055,$0EB7,$01A0,$25BB

*============================================================

* Main program
 org PROGStart
Init:
 ldhx #RAMEnd+1
 txs                    ; set stack pointer to end of RAM

 clr NLL                ; set NLL = 0
 bset 2,DDRC            ;
 mov #%00000100,PTC     ; set propeller off (pin 2 of port C = 1)
                        ; power up GPS (pin 4 of port C = 0)
 mov #%00010100,DDRC    ; set pins 2 & 4 of port C as output
 jsr InitSCI            ; initialize SCI
 jsr InitADC            ; initialize ADC
 jsr InitT1             ; initialize Timer 1
 jsr InitT2             ; initialize Timer 2
 jsr InitKBD            ; initialize Keyboard and Port A
 cli                    ; enable interrupts

Main_Loop:
 lda NLL
 cmp TOTLL              ; check if last waypoint has been reached
 blo Main_Loop          ; if no, then continue looping
 sei                    ; disables interrupts
 mov #%00010100,PTC     ; stop propeller (pin 2 of port C = 1)
                        ; power down GPS (pin 4 of port C = 1)
 stop                   ; set the processor in stop mode

*============================================================

ASC2DEC:
* Convert the ASCII value in (hx) into a value in base10.
* Return the result in (a).
 lda #$0A
 bra ASC
ASC2HEX:
* Convert the ASCII value in (hx) into a value in base16.
* Return the result in (a).
 lda #$10
ASC:
 psha             ; save base (10 or 16) in stack
 pshh             ; save (h) in stack
 txa              ; work on ASCII value in (x)
 ldhx #$000F
ASC1:
 cmp ASCTAB,x     ; compare with values in ASCTAB ...
 beq ASC2
 decx
 bne ASC1         ; ... until finds the value of the low nibble
ASC2:
 pula             ; retrieve (h) ASCII value from stack
 pshx             ; save first nibble in stack
 ldhx #$000F
ASC3:
 cmp ASCTAB,x     ; compare with values in ASCTAB ...
 beq ASC4
 decx
 bne ASC3         ; ... until finds the value of the high nibble
ASC4:
 pulh
 pula             ; retrieve base (10 or 16) from stack
 pshh
 mul              ; compute: high_nibble * base(10 or 16)
 add 1,sp         ; compute: high_nibble + low_nibble
 ais #1           ; restore a proper stack value
 rts
ASCTAB  FCB '0123456789ABCDEF'

*============================================================

ADDMULASC:
* Computes: REGA = ASC2DEC(hx) * a
 psha
 jsr ASC2DEC      ; converts ASCII to base10
 clr REGB
 clr REGB+1
 clr REGB+2
 sta REGB+3
 jsr ADDAB        ; REGA + ASC2DEC(hx)
 pula
 sta REGB+3
 jsr MULAB        ; REGA * a
 rts

*============================================================

COMPAB:
* Compare four bytes REGA with REGB.
* Return result in CCR (Condition Code register)
 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack
 lda #4
 psha             ; set index in stack for a four time loop
 ldhx #0
COMPAB1:
 incx
 lda REGA-1,x
 ora REGB-1,x
 beq COMPAB2      ; jump if REGA=0 and REGB=0 ...
 lda REGA-1,x
 cmp REGB-1,x     ; ... else compare the two bytes of REGA and REGB
 bne COMPAB3      ; jump if REGA <> REGB
COMPAB2:
 dbnz 1,sp,COMPAB1 ; loop four times at most
COMPAB3:
 ais #1           ; restore a proper stack value
 pulh             ; retrieve (h) from stack
 pulx             ; retrieve (x) from stack
 pula             ; retrieve (a) from stack
 rts

*============================================================

SWAPAB:
* Swap four bytes REGA with REGB
 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack
 ldhx #4          ; set (hx) as index for a four time loop
SWAPAB1:
 lda REGA-1,x
 sta REGC-1,x     ; A -> C
 lda REGB-1,x
 sta REGA-1,x     ; B -> A
 lda REGC-1,x
 sta REGB-1,x     ; C -> B
 dbnzx SWAPAB1    ; loop until all four and four bytes are swapped
 pulh             ; retrieve (h) from stack
 pulx             ; retrieve (x) from stack
 pula             ; retrieve (a) from stack
 rts

*============================================================

ADDAB:
* Computes: REGA = REGA + REGB
 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack
 ldhx #4          ; set (hx) as index for a four time loop
 clc
ADDAB1:
 lda REGA-1,x
 adc REGB-1,x
 sta REGA-1,x     ; A = A + B
 dbnzx ADDAB1     ; loop until all four and four bytes are added
 pulh             ; retrieve (h) from stack
 pulx             ; retrieve (x) from stack
 pula             ; retrieve (a) from stack
 rts

*============================================================

SUBAB:
* Computes: REGA = REGA - REGB
 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack
 ldhx #4          ; set (hx) as index for a four time loop
 clc
SUBAB1:
 lda REGA-1,x
 sbc REGB-1,x
 sta REGA-1,x     ; A = A - B
 dbnzx SUBAB1     ; loop until all four and four bytes are subtracted
 pulh             ; retrieve (h) from stack
 pulx             ; retrieve (x) from stack
 pula             ; retrieve (a) from stack
 rts

*============================================================

MULAB:
* Computes: REGA = REGA * REGB
* Multiplication is performed through the following steps:
*  1 - clear REGC
*  2 - REGC = REGC + REGA3 * REGB
*  3 - shift REGA left one byte
*  4 - shift REGB right one byte
*  5 - repeat 2,3,4 four times
*  6 - REGA = REGC

 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack

 clr REGC
 clr REGC+1
 clr REGC+2
 clr REGC+3       ; clear REGC

 ldx #4
 pshx             ; set index in stack for a four times loop
MULAB1:

 lda REGA+3
 ldx REGB+3
 mul
 add REGC+3
 sta REGC+3
 txa
 adc REGC+2
 sta REGC+2
 clra
 adc REGC+1
 sta REGC+1        ; REGC123 = REGC123 + (REGA3 * REGB3)

 lda REGA+3
 ldx REGB+2
 mul
 add REGC+2
 sta REGC+2
 txa
 adc REGC+1
 sta REGC+1
 clra
 adc REGC
 sta REGC          ; REGC012 = REGC012 + (REGA3 * REGB2)

 lda REGA+3
 ldx REGB+1
 mul
 add REGC+1
 sta REGC+1
 txa
 adc REGC
 sta REGC          ; REGC01 = REGC01 + (REGA3 * REGB1)

 lda REGA+3
 ldx REGB
 mul
 add REGC
 sta REGC          ; REGC0 = REGC0 + (REGA3 * REGA0)

 mov REGA+2,REGA+3
 mov REGA+1,REGA+2
 mov REGA,REGA+1
 clr REGA          ; REGA3 <- REGA2 <- REGA1 <- REGA0 <- 0

 mov REGB+1,REGB
 mov REGB+2,REGB+1
 mov REGB+3,REGB+2
 clr REGB+3        ; 0 -> REGB3 -> REGB2 -> REGB1 -> REGB0

 dbnz 1,sp,MULAB1  ; loop four times
 ais #1            ; restore stack with the value before loop

 mov REGC,REGA
 mov REGC+1,REGA+1
 mov REGC+2,REGA+2
 mov REGC+3,REGA+3 ; REGA = REGC

 pulh              ; retrieve (h) from stack
 pulx              ; retrieve (x) from stack
 pula              ; retrieve (a) from stack
 rts

*============================================================

DIVAB:
* Compute: REGA = REGA / REGB
* Division is performed through the following steps:
*  1 - shift REGA and REGB right until REGB is reduced to one byte only
*  2 - shift REGC right one byte
*  3 - REGC3 = (REGA0 / REGB3)
*  4 - shift REGA left one byte
*  5 - repeat 2,3,4 four times
*  6 - REGA = REGC
*  7 - REGB = Remainder

 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack

DIVAB1:
 lda REGB
 ora REGB+1
 ora REGB+2
 beq DIVAB2       ; branch if REGB012 = 000

 lsr REGA
 ror REGA+1
 ror REGA+2
 ror REGA+3       ; shift REGA right one bit

 lsr REGB
 ror REGB+1
 ror REGB+2
 ror REGB+3       ; shift REGB right one bit

 bra DIVAB1       ; loop until REGB is reduced to one byte only

DIVAB2:
 lda #4
 psha              ; set index in stack for a four times loop
 clrh              ; clear (h) as higher byte of the Dividend

DIVAB3:
 mov REGC+1,REGC
 mov REGC+2,REGC+1
 mov REGC+3,REGC+2 ; REGC0 <- REGC1 <- REGC2 <- REGC3

 lda REGA
 ldx REGB+3
 div
 sta REGC+3        ; REGC3 = (h)_REGA0 / REGB3; (h) = Remainder

 mov REGA+1,REGA
 mov REGA+2,REGA+1
 mov REGA+3,REGA+2 ; REGA0 <- REGA1 <- REGA2 <- REGA3

 dbnz 1,sp,DIVAB3  ; loop four times
 ais #1            ; restore stack with the value before loop

 pshh
 pula
 sta REGB+3        ; save the Remainder in REGB3

 mov REGC,REGA
 mov REGC+1,REGA+1
 mov REGC+2,REGA+2
 mov REGC+3,REGA+3 ; REGA = REGC

 pulh             ; retrieve (h) from stack
 pulx             ; retrieve (x) from stack
 pula             ; retrieve (a) from stack
 rts

*============================================================

ADDAHX:
* Compute: (hx) = (hx) + (a)

 psha             ; save (a) in stack
 pshx             ; save (x) in stack
 pshh             ; save (h) in stack

 add 2,sp
 sta 2,sp         ; (x) = (x) + (a)
 clra
 adc 1,sp
 sta 1,sp         ; (h) = (h) + Carry

 pulh             ; retrieve (h) from stack
 pulx             ; retrieve (x) from stack
 pula             ; retrieve (a) from stack
 rts

*============================================================

ATAN:
* Compute: DX = |X0 - X1|
*          DY = |Y0 - Y1|
*          atan (DY / DX)
*
* ArcTangent is expressed with a one byte angular value:
*  North=$00, East=$40, South=$80, West=$C0
*
* ArcTangent is computed through the following steps:
*  1 - clear ATANKEY
*  2 - if (X0 < X1) then (swap X0 and X1; set flag bit ATANKEY2)
*  3 - DX = |X0 - X1|
*  4 - if (Y0 < Y1) then (swap Y0 and Y1; set flag bit ATANKEY1)
*  5 - DY = |Y0 - Y1|
*  6 - if (DX < DY) then (swap DX and DY; set flag bit ATANKEY0)
*  7 - if (DX<DY) then compute ($20*DX/DY) else compute ($20*DY/DX)
*  8 - find REF_ANGLE in ATANTAB according to ATANKEY
*  9 - if key is even, then: atan (DY / DX) = (REF_ANGLE + $20 * DX/DY)
*      if key is odd, then: atan (DY / DX) = (REF_ANGLE - $20 * DY/DX)

 clr ATANKEY
 mov X0,REGA            ; (REGA = X0)
 mov X0+1,REGA+1
 mov X0+2,REGA+2
 mov X0+3,REGA+3
 mov X1,REGB            ; (REGB = X1)
 mov X1+1,REGB+1
 mov X1+2,REGB+2
 mov X1+3,REGB+3
 jsr COMPAB
 bcc ATAN1              ; branch if (X0 > X1)
 bset 2,ATANKEY         ; set flag that (X0 < X1)
 jsr SWAPAB             ; swap X0 and X1
ATAN1:
 jsr SUBAB              ; (X_DIFF = |X0-X1|)
 mov REGA,DX            ; (DX = X_DIFF)
 mov REGA+1,DX+1
 mov REGA+2,DX+2
 mov REGA+3,DX+3

 mov Y0,REGA            ; (REGA = Y0)
 mov Y0+1,REGA+1
 mov Y0+2,REGA+2
 mov Y0+3,REGA+3
 mov Y1,REGB            ; (REGB = Y1)
 mov Y1+1,REGB+1
 mov Y1+2,REGB+2
 mov Y1+3,REGB+3
 jsr COMPAB
 bcc ATAN2              ; branch if (Y0 > Y1)
 bset 1,ATANKEY         ; set flag that (Y0 < Y1)
 jsr SWAPAB             ; swap Y0 and Y1
ATAN2:
 jsr SUBAB              ; (Y_DIFF = |Y0-Y1|)
 mov REGA,DY            ; (DY = Y_DIFF)
 mov REGA+1,DY+1
 mov REGA+2,DY+2
 mov REGA+3,DY+3

 mov DY,REGA            ; (REGA = DY)
 mov DY+1,REGA+1
 mov DY+2,REGA+2
 mov DY+3,REGA+3
 mov DX,REGB            ; (REGB = DX)
 mov DX+1,REGB+1
 mov DX+2,REGB+2
 mov DX+3,REGB+3
 jsr COMPAB
 bcs ATAN3              ; branch if (DY < DX)
 bset 0,ATANKEY         ; set flag that (DX < DY)
 jsr SWAPAB             ; swap DX and DY

ATAN3:
* In order to compute: $20 * (REGA  / REGB), the program attempts to
* increase REGA first, by shifting it five bits left
* (REGA * $20 = REGA * %100000 = 5 shifts left)
* but if during the process REGA becomes too big to be represented in four
* bytes (REGA >= $80000000), then the program starts decreasing REGB
* by shifting it right for the remaining number of shifts that were not
* applied to REGA

 lda #5                 ; set index for 5 shifts (REGA/REGB * %100000)
ATAN4:
 brset 7,REGA,ATAN5     ; if (REGA < $80000000) shift REGA left
 lsl REGA+3
 rol REGA+2
 rol REGA+1
 rol REGA
 bra ATAN6
ATAN5:
 lsr REGB               ; if (REGA >= $80000000) shift REGB right
 ror REGB+1
 ror REGB+2
 ror REGB+3
ATAN6:
 dbnza ATAN4
 jsr DIVAB              ; if (DX<DY) then ($20*DX/DY)
                        ; if (DX>DY) then ($20*DY/DX)

 ldhx #ATANTAB          ; load start address of keys table
 clr ATANI
ATAN7:
 lda ,x
 cmp ATANKEY
 beq ATAN8              ; loop until ATANKEY is found in table
 aix #2
 inc ATANI
 bra ATAN7
ATAN8:
 aix #1
 lda ,x                 ; load reference angle value from table
 brset 0,ATANI,ATAN9    ; jump if ATANI is odd
 add REGA+3             ; (REF_ANGLE + $20 * DX/DY)
 rts
ATAN9:
 sub REGA+3             ; (REF_ANGLE - $20 * DY/DX)
 rts

ATANTAB FCB %001,$00 ; even
        FCB %000,$40 ; odd
        FCB %010,$40 ; even
        FCB %011,$80 ; odd
        FCB %111,$80 ; even
        FCB %110,$C0 ; odd
        FCB %100,$C0 ; even
        FCB %101,$00 ; odd

*============================================================

InitADC:
* Initialize ADC
 mov #%01000000,ADSCR ; CPU interrupt (IDMAS = bit 7 = 0)
                      ; ADC interrupt enabled (AIEN = bit 6 = 1)
                      ; One ADC conversion (ADCO = bit 5 = 0)
                      ; channel 0 selected (ADCH4-0 = bits 4-0 = 00000)
 mov #%01000000,ADCLK ; select the external clock as ADC input clock
                      ;  ADICLK = bit 4 = 0
                      ; select prescaler as: (ADC_input_clock / 4)
                      ;  ADIV2,1,0 = bits 7,6,5 = 010
                      ; as suggested: 4.9152 MHz / 4 = 1 MHz (approx.)
 rts

*============================================================

IntADC:
* Acknowledge interrupts from ADC.
* Store the ADC value into ADC0, ADC1, ADC2 circularly.
* After having stored ADC2, computes the course with the Compass data.

 pshh
 lda ADSCR
 and #%00000111 ; computes which channel was active
 ldhx #ADC0
 jsr ADDAHX
 mov ADR,X+     ; store the ADC value in the appropriate memory location
                ;  ADC0, ADC1 or ADC2
 inca           ; compute the next ADC channel
 cmp #3
 bne IntADC2    ; branch if not 3
 jsr CMPSXY     ; compute X and Y from Compass values
 jsr ATAN       ; compute Compass course
 sta CRSCMPS    ; store the Compass course
 clra
IntADC2:
 ora #%01000000
 sta ADSCR      ; selects the next ADC channel
 pulh
 rti

*============================================================

CMPSXY:
* Computes X0, X1, Y0, Y1 from the Compass ADC values,
*  to be used by the routine ATAN.
* X1 and Y1 are loaded with the values from the Compass sensors 1 and 2
*  (ADC1 and ADC2)
* X0 and Y0 are loaded with the center value of the Compass sensors range
*  (2.5 volts = $80)

 clr X0
 clr X0+1
 clr X0+2
 mov #$80,X0+3 ; X0 = $80
 clr X1
 clr X1+1
 clr X1+2
 mov ADC1,X1+3 ; X1 = ADC1
 clr Y0
 clr Y0+1
 clr Y0+2
 mov #$80,Y0+3 ; Y0 = $80
 clr Y1
 clr Y1+1
 clr Y1+2
 mov ADC2,Y1+3 ; Y1 = ADC2
 rts

*============================================================

InitT1:
* Initialize Timer1 for Buffered PWM mode with output toggle on overflow,
*  in order to generate a pulse to control the rudder servo through the
*  PTD4/T1CH0 output pin.
* The pulse width varies between 0.2 msec ($0200) and 2.3 msec ($1600).
* The servo center position is achieved with 1.25 msec ($0C00).

 mov #%10110000,T1SC  ; stop TIM1 (TSTOP = bit 5 = 1)
                      ; reset TIM1 (TRST = bit 4 = 1)
 mov #$C0,T1MODH      ; write PWM period ($C000) in TMOD1
 clr T1MODL
 mov #$0C,T1CH0H      ; write initial pulse width ($0C00) in T1CH0
 clr T1CH0L
 mov #$0C,T1CH1H      ; write initial pulse width ($0C00) in T1CH1
 clr T1CH1L
 mov #%10111110,T1SC0 ; buffered PWM mode (MS0B,MS0A = bits 5,4 = 1X)
                      ; set output on compare (ELS0B,0A = bits 3,2 = 11)
                      ; toggle on overflow (TOV0 = bit 1 = 1)
 mov #%10000000,T1SC1 ; clear bit 1 that will be used as a flag of the
                      ; channel register (0 or 1) that will control the
                      ; next pulse width in PWM buffered mode
 mov #%11000000,T1SC  ; start TIM1 (TSTOP = bit 5 = 0)
                      ; prescaler = internal_bus_clock
                      ;  (PS2-PS0 = bits 2-0 = 000)
 rts

*============================================================

IntTIM1:
* Acknowledge interrupts from Timer1.
* Compute the value for the pulse width to control the rudder servo
* and store it in T1CH0.
* The servo rotation ranges from -90° to +90° ($1600-$0200 pulse width).
* This whole range is covered with the contribution of both the rudder
* trigger and the computation of the difference in azimuth between GPS and
* Compass courses.
* The rudder trigger sets an initial value of +/-45° ($0500 - $1100 PWM).
* The course difference ranges from -180° to + 180°, but it is limited
* within +/-45° (+/-$20 in angular value).
*
*  1 - compute pulse value from rudder trigger
*  2 - jump to -4- if Compass Course is disabled by SW4
*  3 - compute pulse value from the azimuth between GPS and Compass courses
*      course_diff = GPS_course - Compass_course
*      if (course_diff >= 0) then
*       if (course_diff > $20) then compute: course_diff = $20
*       pulse = rudder_trim - (course_diff * $28)
*      else if (course_diff < 0) then
*       if (course_diff > $20) then compute: course_diff = $20
*       pulse = rudder_trim + (course_diff * $28)
*  4 - store pulse value in T1CH0 or T1CH1; depending on which one has
*       controlled the Overflow interrupt last, then the pulse value is
*       written in the other one
*  5 - clear Timer1 Overflow Flag Bit

 pshh
 lda ADC0            ; load rudder trimmer value
 ldx #$0A
 mul                 ; normalize it for the servo pulse width (ADC0 * $0A)
 clr REGA
 clr REGA+1
 stx REGA+2
 sta REGA+3
 clr REGB
 clr REGB+1
 mov #$07,REGB+2
 clr REGB+3
 jsr ADDAB           ; RUDDER_TRIM = (ADC0 * $0A) + $0700
                     ; when the trimmer is centered: ADC0 = $80,
                     ; then the servo reaches the center position:
                     ;  $0C00 = ($80 * $0A) + $0700
 brclr 2,PTA,IntT1_4 ; jump if Compass Course is disabled by SW4 (PtA-Pin2)
 lda CRSGPS
 sub CRSCMPS         ; COURSE_DIFF = COURSE_GPS - COURSE_COMPASS
 cmp #$80
 bhs IntT1_2         ; branch if (COURSE_DIFF >= $80)
 cmp #$20
 bls IntT1_1
 lda #$20            ; if (COURSE_DIFF > $20) then COURSE_DIFF = $20
IntT1_1:
 ldx #$28
 mul                 ; COURSE_DIFF * $28
 stx REGB+2
 sta REGB+3
 jsr SUBAB           ; PULSE_WIDTH = RUDDER_TRIM - (COURSE_DIFF * $28)
 bra IntT1_4
IntT1_2:
 nega                ; COURSE_DIFF = $00 - COURSE_DIFF
 cmp #$20
 bls IntT1_3
 lda #$20            ; if (COURSE_DIFF > $20) then COURSE_DIFF = $20
IntT1_3:
 ldx #$28
 mul                 ; COURSE_DIFF * $28
 stx REGB+2
 sta REGB+3
 jsr ADDAB           ; PULSE_WIDTH = RUDDER_TRIM + (COURSE_DIFF * $28)

IntT1_4:
 brclr 1,T1SC1,IntT1_5 ; jump if previous Overflow was set by T1CH0
 bclr 1,T1SC1          ; flag indicates that next Overflow is set by T1CH0
 mov REGA+2,T1CH0H
 mov REGA+3,T1CH0L     ; store PULSE_WIDTH in T1CH0
 bra IntT1_6
IntT1_5:
 bset 1,T1SC1          ; flag indicates that next Overflow is set by T1CH1
 mov REGA+2,T1CH1H
 mov REGA+3,T1CH1L     ; store PULSE_WIDTH in T1CH1

IntT1_6:
 bclr 7,T1SC           ; clear Timer1 Overflow Flag Bit (TOF = bit 7 = 0)
 pulh
 rti

*============================================================

LATLONG:
* Converts Latitude and Longitude data from the GPS ASCII strings
* (ddMM,mmnn and DddMM,mmnn) into 4 bytes integers: LAT and LONG
* expressed in "minutes/10000"
* LAT  = ((dd*60 + MM)*60 + mm)*100 +nn
* LONG = (((D*100 + dd)*60 + MM)*60 + mm)*100 +nn
*
* An example of a GPS ASCII string is:
* 0         1         2         3         4         5         6
* 0123456789012345678901234567890123456789012345678901234567890123456789
* ======================================================================
* $GPRMC,145055,A,4453.6083,N,00944.9533,E,000.0,000.0,070399,000.3,E*7F
*  -----        - ddMM.mmnn   DddMM.mmnn                              --
*  label       val latitude    longitude                           checksum

 ldhx #4
LATLONG1:
 clr REGA-1,x     ; clear REGA
 clr REGB-1,x     ; clear REGB
 dbnzx LATLONG1

 ldhx GPSMSG+!16  ; load LAT degrees (dd)
 lda #!60
 jsr ADDMULASC    ; convert to numbers and multiply by 60
 ldhx GPSMSG+!18  ; load LAT minutes (MM)
 lda #!60
 jsr ADDMULASC    ; convert to numbers and multiply by 60
 ldhx GPSMSG+!21  ; load LAT minutes/100 (mm)
 lda #!100
 jsr ADDMULASC    ; convert to numbers and multiply by 100
 ldhx GPSMSG+!23  ; load LAT minutes/10000 (nn)
 lda #1
 jsr ADDMULASC    ; convert to numbers

 mov REGA,LAT
 mov REGA+1,LAT+1
 mov REGA+2,LAT+2
 mov REGA+3,LAT+3 ; LAT = REGA

 ldhx #4
LATLONG2:
 clr REGA-1,x     ; clear REGA
 clr REGB-1,x     ; clear REGB
 dbnzx LATLONG2

 clrh
 ldx GPSMSG+!28   ; load LONG degrees*100 (D)
 lda #!100
 jsr ADDMULASC    ; convert to numbers and multiply by 100
 ldhx GPSMSG+!29  ; load LONG degrees (dd)
 lda #!60
 jsr ADDMULASC    ; convert to numbers and multiply by 60
 ldhx GPSMSG+!31  ; load LONG minutes (MM)
 lda #!60
 jsr ADDMULASC    ; convert to numbers and multiply by 60
 ldhx GPSMSG+!34  ; load LONG minutes/100 (mm)
 lda #!100
 jsr ADDMULASC    ; convert to numbers and multiply by 100
 ldhx GPSMSG+!36  ; load LONG minutes/10000 (nn)
 lda #1
 jsr ADDMULASC    ; convert to numbers and multiply by 100

 mov REGA,LONG
 mov REGA+1,LONG+1
 mov REGA+2,LONG+2
 mov REGA+3,LONG+3 ; LONG = REGA
 rts

*============================================================

GPSXY:
* Compute X & Y range between GPS-point and next waypoint.
*
* Inputs:
*  LONG_GPS       LONG received by the GPS
*  LAT_GPS        LAT received by the GPS
*  LLDATA         start address of LONG & LAT waypoints data
*  NLL            index of the Nth LONG & LAT waypoint data
*  LONG_COURSE_POINT = LLDATA + (NLL-1)*8
*  LAT_COURSE_POINT  = LLDATA + (NLL-1)*8+4
*  XCF            meters / 1 minute of LONG
*  YCF            meters / 1 minute of LAT
*
* Outputs:
*  if (LONG_GPS < LONG_WAYPOINT) then (X0 = 0 ; X1 = LONG_DIFF)
*                                    else (X0 = LONG_DIFF ; X1 = 0)
*  if (LAT_GPS < LAT_WAYPOINT) then (Y0 = 0 ; Y1 = LAT_DIFF)
*                                  else (Y0 = LAT_DIFF ; Y1 = 0)

 lda NLL                ; compute the displacement to the address of
 ldx #8                 ; the first byte of the LONG coordinate of the
 mul                    ; next waypoint: (NLL-1)*8
 ldhx #LLDATA           ; load waypoints start address
 jsr ADDAHX             ; compute address of next waypoint
 mov x+,REGB            ; load REGB with LONG of next waypoint
 mov x+,REGB+1
 mov x+,REGB+2
 mov x+,REGB+3
 mov LONG,REGA          ; load REGA with LONG from GPS
 mov LONG+1,REGA+1
 mov LONG+2,REGA+2
 mov LONG+3,REGA+3
 jsr COMPAB
 bcc GPSXY1             ; branch if (LONG_GPS > LONG_WAYPOINT)
 jsr SWAPAB             ; else swap LONG_GPS and LONG_WAYPOINT
 jsr SUBAB              ; (LONG_DIFF = LONG_WAYPOINT - LONG_GPS)
 clr REGB               ; load REGB with XCF
 clr REGB+1
 ldhx #XCF
 mov x+,REGB+2
 mov x+,REGB+3
 jsr MULAB              ; (X_DIFF = LONG_DIFF * XCF ...
 clr REGB
 clr REGB+1
 ldhx #!10000
 sthx REGB+2
 jsr DIVAB              ; ... / 10000)
 clr X0                 ; (X0 = 0)
 clr X0+1
 clr X0+2
 clr X0+3
 mov REGA,X1            ; (X1 = X_DIFF)
 mov REGA+1,X1+1
 mov REGA+2,X1+2
 mov REGA+3,X1+3
 bra GPSXY2
GPSXY1:
 jsr SUBAB              ; (LONG_DIFF = LONG_GPS - LONG_WAYPOINT)
 clr REGB               ; load REGB with XCF
 clr REGB+1
 ldhx #XCF
 mov x+,REGB+2
 mov x+,REGB+3
 jsr MULAB              ; (X_DIFF = LONG_DIFF * XCF ...
 clr REGB
 clr REGB+1
 ldhx #!10000
 sthx REGB+2
 jsr DIVAB              ; ... / 10000)
 mov REGA,X0            ; (X0 = X_DIFF)
 mov REGA+1,X0+1
 mov REGA+2,X0+2
 mov REGA+3,X0+3
 clr X1                 ; (X1 = 0)
 clr X1+1
 clr X1+2
 clr X1+3
GPSXY2:
 lda NLL                ; compute the displacement to the address of
 ldx #8                 ; the first byte of the LAT coordinate of the
 mul                    ; next waypoint: (NLL-1)*8+4
 add #4                 ;
 ldhx #LLDATA           ; load waypoints start address
 jsr ADDAHX             ; calculate address of next waypoint
 mov x+,REGB            ; load REGB with LAT of the next waypoint
 mov x+,REGB+1
 mov x+,REGB+2
 mov x+,REGB+3
 mov LAT,REGA           ; load REGA with LAT from GPS
 mov LAT+1,REGA+1
 mov LAT+2,REGA+2
 mov LAT+3,REGA+3
 jsr COMPAB
 bcc GPSXY3             ; branch if (LAT_GPS > LAT_WAYPOINT)
 jsr SWAPAB             ; else swap LAT_GPS and LAT_WAYPOINT
 jsr SUBAB              ; (LAT_DIFF = LAT_WAYPOINT - LAT_GPS)
 clr REGB               ; load REGB with YCF
 clr REGB+1
 ldhx #YCF
 mov x+,REGB+2
 mov x+,REGB+3
 jsr MULAB              ; (Y_DIFF = LAT_DIFF * YCF ...
 clr REGB
 clr REGB+1
 ldhx #!10000
 sthx REGB+2
 jsr DIVAB              ; ... / 10000)
 clr Y0                 ; (Y0 = 0)
 clr Y0+1
 clr Y0+2
 clr Y0+3
 mov REGA,Y1            ; (Y1 = Y_DIFF)
 mov REGA+1,Y1+1
 mov REGA+2,Y1+2
 mov REGA+3,Y1+3
 bra GPSXY4
GPSXY3:
 jsr SUBAB              ; (LAT_DIFF = LAT_GPS - LAT_WAYPOINT)
 clr REGB               ; load REGB with YCF
 clr REGB+1
 ldhx #YCF
 mov x+,REGB+2
 mov x+,REGB+3
 jsr MULAB              ; (Y_DIFF = LAT_DIFF * YCF ...
 clr REGB
 clr REGB+1
 ldhx #!10000
 sthx REGB+2
 jsr DIVAB              ; ... / 10000)
 mov REGA,Y0            ; (Y0 = Y_DIFF)
 mov REGA+1,Y0+1
 mov REGA+2,Y0+2
 mov REGA+3,Y0+3
 clr Y1                 ; (Y1 = 0)
 clr Y1+1
 clr Y1+2
 clr Y1+3
GPSXY4:
 rts

*============================================================

InitKBD:
* Initialize Port A:
* - enable pullups for pins 1,2,3
* - clear fault keyboard interrupts
* - configure pin 2 as a keyboard interrupt pin

 bset 1,INTKBSCR         ; mask keyboard interrupts (IMASKK = 1)
 mov #%00001110,PTAPUE   ; enable pullup for pins 1,2,3 of port A (SW5,4,3)
*                        ; - pin 1 : disable GPS course
*                        ; - pin 2 : disable Compass course
*                        ; - pin 3 : store present Compass course
 mov #%00001000,INTKBIER ; enable pin 3 as a keyboard interrupt pin
 bset 2,INTKBSCR         ; clear false interrupts (ACKK = 1)
 bclr 1,INTKBSCR         ; enable keyboard interrupts (IMASKK = 0)
 rts

*============================================================

IntKBD:
* Acknowledges interrupts from Keyboard.

 brset 3,PTA,IntKBD2     ; jump if SW3 is not pushed
 mov CRSCMPS,CRSGPS      ; copy the Compass Course in the GPS Course
IntKBD2:
 rti

*============================================================

InitT2:
* Initialize Timer 2 to control the red led blinking.
*
 mov #%10000000,T2SC0 ; set pin 6 of port D as general-purpose I/O pin
                      ; Output preset mode (ELS0B,0A = bits 3,2 = 00)
 bclr 6,PTD           ; preset pin 6 of port D = 0 (led off)
 bset 6,DDRD          ; configure pin 6 of port D as output
 mov #%10110000,T2SC  ; stop TIM2 (TSTOP = bit 5 = 1)
                      ; reset TIM2 (TRST = bit 4 = 1)
 mov #$FF,T2MODH
 mov #$FF,T2MODL      ; load Timer2 Modulo registers with $FFFF
 mov #%10000110,T2SC  ; start TIM2 (TSTOP = bit 5 = 0)
                      ; prescaler = (internal_bus_clock / 64)
                      ;  (PS2-PS0 = bits 2-0 = 110)
 rts

*============================================================

BLINK:
* Set Timer2-Ch0 for Output Compare mode, in order to generate a negative
* pulse on the PTD6/T2CH0 output pin and switch the led on for 0.5 sec.

 mov #%10000000,T2SC0 ; Initial output high (MS0B,MS0A = bits 5,4 = X0)
*                     ; Output Preset mode (ELS0B,0A = bits 3,2 = 00)
 mov #%10011000,T2SC0 ; Output Compare mode (MS0B,MS0A = bits 5,4 = 01)
*                     ; clear output on compare (ELS0B,0A = bits 3,2 = 10)
 ldhx T2CNTH          ; load present value of Time2 ...
 pshh
 lda #$4B
 add 1,sp
 sta 1,sp             ; ... add $4B (0.5 sec) ...
 pulh
 sthx T2CH0H          ; ... and store it in T2-Ch0 Output Compare registers
 rts

*============================================================

InitSCI:
* Initialize SCI.

 mov #%00000100,SCBR ; prescaler divisor = 1 (SCP1,0 = bits 5,4 = 00)
                     ; baud rate divisor = 16 (SCR210 = bits 210 = 100)
                     ; baud rate = Fbus / (64 * PD * BD)
                     ;           = 4.9152 MHz / 1024 = 4800 bps
 mov #%01000000,SCC1 ; enable SCI (ENSCI = bit 6 = 1)
                     ; 8-bit characters (M = bit 4 = 0)
                     ; no parity (PEN = bit 1 = 0)
 mov #%00100100,SCC2 ; enable receive interrupts (SCRIE = bit 5 = 1)
                     ; enable receiver (RE = bit 2 = 1)
 ldhx #GPSMSG
 stx GPSHX           ; set the location address index for the next
                     ; character to be received from GPS,
                     ; at the beginning of the GPS message space
 rts

*============================================================

IntSCI:
* Acknowledge interrupts from SCI.
* - receive character and store it in the next memory location
* - if (char = Carriage Return) then:
*   - check if (label = "GPRMC")
*   - check checksum byte
*   - check if (val = "A")
*   - if GPS data are correct then:
*     - compute LAT and LONG
*     - compute X & Y range between GPS and next waypoint
*     - compute GPS course, DX and DY
*     - if GPS Course is enabled by SW5, then store GPS course
*     - compute distance to the next waypoint
*
* An example of a GPS ASCII string is:
* 0         1         2         3         4         5         6
* 0123456789012345678901234567890123456789012345678901234567890123456789
* ======================================================================
* $GPRMC,145055,A,4453.6083,N,00944.9533,E,000.0,000.0,070399,000.3,E*7F
*  -----        - ddMM.mmnn   DddMM.mmnn                              --
*  label       val latitude    longitude                           checksum
*

 pshh
 ldhx GPSHX        ; load location address for the next character from GPS
 lda SCS1          ; load SCS1 to clear Receiver Full Bit (SCRF = bit 5)
 lda SCDR          ; load the received character
 cmp #$0D
 beq IntSCI1       ; branch if (char = Carriage Return)
 cmp #$0A
 beq IntSCI4       ; branch if (char = Line Feed)
 sta ,x            ; store character in the current message location
 aix #1            ; increase index to point the next message location
 bra IntSCI5
IntSCI1:
 lda GPSMSG+1
 cmp #'G'
 bne IntSCI4
 lda GPSMSG+2
 cmp #'P'
 bne IntSCI4
 lda GPSMSG+3
 cmp #'R'
 bne IntSCI4
 lda GPSMSG+4
 cmp #'M'
 bne IntSCI4
 lda GPSMSG+5
 cmp #'C'
 bne IntSCI4       ; branch if (GPS_message_label = GPSMSG1-5 != "GPRMC")

 ldhx #!66         ; load (hx) with the GPS main message length (66)
 clra
IntSCI2:
 eor GPSMSG,x      ; compute checksum for the current GPS message
 dbnzx IntSCI2     ; loop 66 times
 sta GPSCK
 ldhx GPSMSG+44    ; load checksum character from the current GPS message
 jsr ASC2HEX       ; convert it from ASCII to an hex-number
 cmp GPSCK         ; compare it with the computed checksum value
 bne IntSCI4       ; branch if checksum is not correct

 lda GPSMSG+!14
 cmp #'A'
 bne IntSCI4       ; branch if GPS acquisition is not 2D

 jsr BLINK         ; blink red led when a correct GPS message is received
 bclr 2,PTC        ; switch the propeller on (pin 2 in port C = 0)
 jsr LATLONG       ; compute LAT and LONG
 jsr GPSXY         ; compute X & Y range between GPS and next waypoint
 jsr ATAN          ; compute GPS course, DX and DY
 brclr 1,PTA,IntSCI3 ; branch if GPS Course is disabled by SW5 (PtA-Pin1)
 sta CRSGPS        ; store GPS course
IntSCI3:
 jsr DIST          ; compute distance to the next waypoint
IntSCI4:
 ldhx #GPSMSG      ; restore index to the beginning of the message space
IntSCI5:
 sthx GPSHX        ; save location for the next character to be received
 pulh
 rti

*============================================================

DIST:
* Computes the distance to the next waypoint.
* If the distance is shorter than "RANGE", then head to the next waypoint.
* In order to avoid dealing with "square-root" operation, the formula
*  actually used is: (DX^2 + DY^2) COMPARED (RANGE^2)

 mov DX,REGA
 mov DX+1,REGA+1
 mov DX+2,REGA+2
 mov DX+3,REGA+3
 mov DX,REGB
 mov DX+1,REGB+1
 mov DX+2,REGB+2
 mov DX+3,REGB+3
 jsr MULAB
 mov REGA,REGD
 mov REGA+1,REGD+1
 mov REGA+2,REGD+2
 mov REGA+3,REGD+3      ; DX^2 = DX * DX

 mov DY,REGA
 mov DY+1,REGA+1
 mov DY+2,REGA+2
 mov DY+3,REGA+3
 mov DY,REGB
 mov DY+1,REGB+1
 mov DY+2,REGB+2
 mov DY+3,REGB+3
 jsr MULAB              ; DY^2 = DY * DY

 mov REGD,REGB
 mov REGD+1,REGB+1
 mov REGD+2,REGB+2
 mov REGD+3,REGB+3
 jsr ADDAB
 mov REGA,REGD
 mov REGA+1,REGD+1
 mov REGA+2,REGD+2
 mov REGA+3,REGD+3      ; DISTANCE^2 = DX^2 + DY^2

 clr REGA
 clr REGA+1
 ldhx #RANGE
 mov x+,REGA+2
 mov x+,REGA+3
 clr REGB
 clr REGB+1
 ldhx #RANGE
 mov x+,REGB+2
 mov x+,REGB+3
 jsr MULAB              ; RANGE^2 = RANGE * RANGE

 mov REGD,REGB
 mov REGD+1,REGB+1
 mov REGD+2,REGB+2
 mov REGD+3,REGB+3
 jsr COMPAB             ; (RANGE^2) COMPARED (DISTANCE^2)
 blo DIST2              ; jump if ( RANGE < DISTANCE )
 inc NLL                ; else select next waypoint
DIST2:
 rts

