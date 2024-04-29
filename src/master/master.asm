; Half-Duplex USART Master Communication for ATmega32U4

; Include necessary definitions and variable declarations
.include "atmega32u4_def.inc"

; ------------------------------------------------------------------------------
; Macro: subtract_immediate_16bit_check
; Description: Subtracts a 16-bit immediate value from a 16-bit register pair and checks
;              the zero flag to branch if the result is not zero.
; Usage: subtract_immediate_16bit_check dest_low_reg, dest_high_reg, immediate_value, label_not_zero
; Parameters:
;   @1 - Low byte of the destination register pair.
;   @2 - High byte of the destination register pair.
;   @0 - 16-bit immediate value to subtract.
;   @3 - Label to branch to if the result is not zero.
; ------------------------------------------------------------------------------
.macro subtract_immediate_16bit_check
    subi    @1, LOW(@0)                    ; Subtract immediate from low byte
    sbci    @2, HIGH(@0)                   ; Subtract with carry from high byte
    brne    @3                             ; Branch if the result is not zero
.endmacro

;-------------------------------------------------------------------------------
; Interrupt Vector Table
.org OVF0addr    ; Timer 0 Overflow Interrupt Vector (0x2E)
    rjmp timer0_overflow_handler

.org OVF4addr    ; Timer 4 Overflow Interrupt Vector (0x52)
    rjmp timer4_overflow_handler
    reti

;-------------------------------------------------------------------------------
; Interrupt Handling Routine
timer0_overflow_handler:
    in SREG_Storage, SREG     ; Save Status Register
    dec tick_count            ; Decrement tick count for Timer0
    brne exit_timer0
    ldi tick_count, TICKS_PER_SECOND    ; Reload for 1-second interval
    inc seconds_counter       ; Increment system seconds counter
exit_timer0:
    rjmp restore_state

timer4_overflow_handler:
    in SREG_Storage, SREG     ; Save Status Register
    dec tick_count4           ; Decrement tick count for Timer4
    cpi tick_count4, 0
    brne exit_timer4
    ldi tick_count4, TICKS_PER_SECOND   ; Reload tick count for timer4
    cpi adc_read_seconds, ADC_STARTUP_DELAY
    brsh exit_timer4
    inc adc_read_seconds      ; Increment seconds counter for ADC
exit_timer4:
    rjmp restore_state

restore_state:
    out SREG, SREG_Storage    ; Restore Status Register
    reti

;-------------------------------------------------------------------------------
; System Initalization
initial_setup:
    ldi temp, LOW(RAMEND)     ; Initialize Stack Pointer
    out SPL, temp
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, 0x80
    out MCUCR, temp           ; Disable JTAG

;-------------------------------------------------------------------------------
; Port Configuration
; Configure PORTB for Serial Communication
    ldi temp, 0xE7           ; Set PORTB direction for USART
    out DDRB, temp
    ldi temp, 0x11
    out PORTB, temp
    sbi PORTB, 6             ; Set BEn high to disable

; Configure PORTC for general I/O
    ldi temp, 0x80
    out DDRC, temp
    ldi temp, 0x40
    out PORTC, temp

; Configure PORTD for additional functionality
    ldi temp, 0x2C
    out DDRD, temp
    ldi temp, 0xD0
    out PORTD, temp
    sbi PORTD, 5             ; Turn off Green LED (control indicator)

; Configure PORTE for USART Extended Controls
    ldi temp, 0x40
    out DDRE, temp
    ldi temp, 0x04
    out PORTE, temp

; Configure PORTF for ADC and sensor inputs
    ldi temp, 0x23
    out DDRF, temp
    ldi temp, 0xD0
    out PORTF, temp

; Timer 0 Configuration for Basic Timing Operations
    ldi temp, 0x00
    out TCCR0A, temp
    ldi temp, 0x04
    out TCCR0B, temp
    ldi temp, 0x01
    sts TIMSK0, temp         ; Enable overflow interrupt

; Timer 1 Configuration for Pulse Generation
    ldi temp, 0x00
    sts TCCR1A, temp         ; Set to CTC mode, PWM off
    ldi temp, 0x09
    sts TCCR1B, temp
    ldi temp, motor_speed    ; Set motor speed
    sts OCR1AL, temp
    ldi temp, 0x00
    sts OCR1AH, temp
    ldi temp, 0x00
    sts TIMSK1, temp

; Timer 4 Configuration for ADC Timing
    ldi temp, 0x00
    sts TCCR4A, temp
    ldi temp, 0x09
    sts TCCR4B, temp
    ldi temp, 0x00
    sts TCCR4C, temp
    sts TCCR4D, temp
    sts TCCR4E, temp
    ldi temp, 0x04
    sts TIMSK4, temp

; ADC Setup
    ldi temp, 0x20          ; Configure ADC for left-adjust result
    sts ADMUX, temp
    ldi temp, 0x00
    sts ADCSRB, temp
    ldi temp, 0x01
    sts DIDR0, temp         ; Disable digital input on ADC0 to save power

; Start of main application
main:
    ; Setup complete, main loop begins
    rcall debounce_switches
    rcall check_switch_states
    sei                     ; Enable global interrupts

;-------------------------------------------------------------------------------
; USART Initialization
USART_Init:
    ldi temp, 0x06                   ; Frame format: 8 data, no parity, 1 stop
    sts UCSR1C, temp
    ldi temp, BAUD_LOW               ; Low byte of baud rate
    sts UBRR1L, temp
    ldi temp, BAUD_HIGH              ; High byte of baud rate
    sts UBRR1H, temp
    ldi temp, 0x98                   ; Enable receiver, transmitter and RX complete interrupt
    sts UCSR1B, temp
    ret

;-------------------------------------------------------------------------------
; USART Transmit Complete Handler
USART_Transmit:
    wait_clear_buffer:
        lds temp, UCSR1A
        sbrs temp, UDRE1
        rjmp wait_clear_buffer       ; Wait until the buffer is empty

    sts UDR1, r16                    ; Send data
    wait_tx_complete:
        lds temp, UCSR1A
        sbrs temp, TXC1
        rjmp wait_tx_complete        ; Wait for transmission to complete

    ldi temp, 0x40                   ; Clear TX complete flag
    sts UCSR1A, temp
    ret

;-------------------------------------------------------------------------------
; USART Receive Complete Handler
USART_Receive:
    lds     temp_reg, UDR1                  ; Load received data
    mov     temp_reg2, temp_reg
    andi    temp_reg2, 0xF0                 ; Check if address matches
    cpi     temp_reg2, usart_device_address
    brne    not_addressed_to_me
    addressed_to_me:
    andi    temp_reg, 0x0F                  ; Process command part
    rjmp    continue_processing
    not_addressed_to_me:
    continue_processing:
    ldi     temp_reg2, 0x80                 ; Reset RXC1 bit
    sts     UCSR1A, temp_reg2
    ret
;-------------------------------------------------------------------------------
; Timer Initialization for System Timing and Delays
TIMER_INITIAL:
    ldi temp, ticks_per_sec          ; Load tick count constant
    sts tick_cnt, temp               ; Initialize timer0 tick count
    clr temp                         ; Clear seconds counter
    sts seconds, temp
    ret

TIMER_INITIAL4:
    ldi temp, ticks_per_sec          ; Load tick count constant for timer4
    sts tick_cnt4, temp              ; Initialize timer4 tick count
    clr temp                         ; Clear seconds counter for timer4
    sts seconds4, temp
    ret

;-------------------------------------------------------------------------------
; Delay Subroutines for Timing Adjustments
DELAY_IT:
    ldi temp, DELAY_2S               ; Load delay constant for 2 seconds
    call DELAY_SUB                   ; Call generic delay subroutine
    ret

DELAY_IT_Short:
    ldi temp, DELAY_500MS            ; Load delay constant for 0.5 seconds
    call DELAY_SUB                   ; Call generic delay subroutine
    ret

DELAY_SUB:
    ; Generic delay subroutine using system timer
    ; Assumes 'temp' contains the number of seconds to delay
    ; Uses 'seconds' as the counter (needs to be cleared before calling)
    clr seconds                      ; Clear the seconds counter
delay_loop:
    cpse seconds, temp               ; Compare and skip if equal
    rjmp delay_loop                  ; Loop until delay is complete
    ret


;-------------------------------------------------------------------------------
; Manage motor operations based on command and sensor feedback
MOTOR_ON:
    cbi     PORTC, 7                         ; Ensure yellow LED is off indicating motor is running
    ldi     temp_reg, 0x0C
    ldi     temp_reg2, 0x1B
    rcall   spi_write_to_register            ; Write configurations via SPI
    ldi     temp_reg, 0x00
    sts     TCCR1A, temp_reg                 ; Set Timer 1 for PWM off
    rcall   delay_short                      ; Short delay before starting motor
    ldi     temp_reg2, 0x40
    sts     TCCR1A, temp_reg2                ; PWM on
    ret

MOTOR_OFF:
    ldi     temp_reg, 0x00
    sts     TCCR1A, temp_reg                 ; Disable PWM, motor off
    ldi     temp_reg, 0x0C
    ldi     temp_reg2, 0x1A
    rcall   spi_write_to_register            ; Disable motor through SPI
    ret
;-------------------------------------------------------------------------------
