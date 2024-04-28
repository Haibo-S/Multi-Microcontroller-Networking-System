; Half-Duplex USART Slave Communication for ATmega32U4

; Include microcontroller-specific definitions and universal constants
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
.org 0x00
.macro subtract_immediate_16bit_check
    subi    @1, LOW(@0)   ; Subtract immediate from low byte
    sbci    @2, HIGH(@0)  ; Subtract with carry from high byte
    brne    @3            ; Branch if the result is not zero (status check integrated)
.endmacro

; Initialize program with a jump to the main routine
rjmp    main_routine_start

; ------------------------------------------------------------------------------
; Interrupt Service Routines
.org 0x2E                               ; Address for TIMER 0 Overflow Interrupt
rjmp    handle_timer0_overflow_interrupt
.org 0x52                               ; Address for TIMER 4 Overflow Interrupt
rjmp    handle_timer4_overflow_interrupt
reti                                    ; Return from interrupt

; Timer0 overflow interrupt service routine
handle_timer0_overflow_interrupt:
in      status_reg_backup, SREG         ; Backup status register
dec     timer0_tick_count               ; Decrement timer0 tick count
brne    exit_timer0_overflow            ; Branch if not zero
ldi     timer0_tick_count, TICKS_PER_SECOND  ; Reset tick count for one second
inc     seconds_count_timer0            ; Increment seconds count
exit_timer0_overflow:
rjmp    restore_interrupt_status        ; Restore status and return from interrupt

; Timer4 overflow interrupt service routine
handle_timer4_overflow_interrupt:
in      status_reg_backup, SREG         ; Backup status register
dec     timer4_tick_count               ; Decrement timer4 tick count
cpi     timer4_tick_count, 0x00
brne    exit_timer4_overflow
ldi     timer4_tick_count, TICKS_PER_SECOND  ; Reset tick count for timer4
cpi     seconds_count_timer4, ADC_START_DELAY
brsh    exit_timer4_overflow
inc     seconds_count_timer4            ; Increment seconds count
exit_timer4_overflow:
rjmp    restore_interrupt_status        ; Restore status and return from interrupt

; Routine to restore status register from backup
restore_interrupt_status:
out     SREG, status_reg_backup         ; Restore status register from backup
reti                                    ; Return from interrupt
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
; Main program starts here
main_routine_start:
ldi     temp_reg, LOW(RAMEND)           ; Initialize stack pointer
out     SPL, temp_reg
ldi     temp_reg, HIGH(RAMEND)
out     SPH, temp_reg
ldi     temp_reg, 0x80
out     MCUCR, temp_reg                 ; Set MCU Control Register (disable JTAG)

; Initialize digital I/O ports
setup_digital_io_ports:
; PORT B: Used for Serial Communication
ldi     temp_reg, 0xA7
out     DDRB, temp_reg                  ; Set PORT B Data Direction
ldi     temp_reg, 0x50
out     PORTB, temp_reg                 ; Set PORT B Output

; PORT C: General Purpose I/O
ldi     temp_reg, 0x80
out     DDRC, temp_reg                  ; Set PORT C Data Direction
ldi     temp_reg, 0x40
out     PORTC, temp_reg                 ; Set PORT C Output

; Additional setups for PORT D, PORT E, and PORT F (These can be Optional)

; ------------------------------------------------------------------------------
; Configure Timer 0
setup_timers:
configure_timer0:
ldi     temp_reg, 0x00
out     TCCR0A, temp_reg                ; Configure Timer 0 Control Register A
ldi     temp_reg, 0x04
out     TCCR0B, temp_reg                ; Configure Timer 0 Control Register B
ldi     temp_reg, 0x01
sts     TIMSK0, temp_reg                ; Enable Timer 0 Overflow Interrupt
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
; USART initialization and configuration
initialize_usart:
in      temp_reg, PINF                  ; Read address from PINF
andi    temp_reg, 0xF0                  ; Filter out lower 4 bits
mov     usart_device_address, temp_reg  ; Store USART device address
cpi     temp_reg, 0x10
breq    configure_group1
cpi     temp_reg, 0x20
breq    configure_group1
; Additional configuration based on USART address

configure_group1:
ldi     start_number, 0x10              ; Set start number for group 1
rjmp    set_usart_parameters

set_usart_parameters:
ldi     temp_reg2, 0x80                 ; Clear RXC1 bit
sts     UCSR1A, temp_reg2
ldi     temp_reg, 0x06                  ; Set frame format
sts     UCSR1C, temp_reg
ldi     temp_reg, BAUD_RATE_LOW         ; Set baud rate
ldi     temp_reg2, BAUD_RATE_HIGH
sts     UBRR1H, temp_reg
sts     UBRR1L, temp_reg2
ret

; USART transmission enable subroutine
enable_usart_transmitter:
sbi     PORTE, 6                        ; Enable USART transmitter
ldi     temp_reg, 0x08
sts     UCSR1B, temp_reg
ret

; USART reception enable subroutine
enable_usart_receiver:
cbi     PORTE, 6                        ; Enable USART receiver
ldi     temp_reg, 0x10
sts     UCSR1B, temp_reg
ret

; USART data receive subroutine
usart_receive_data:
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

; USART data transmit subroutine
usart_transmit_data:
wait_for_buffer_clear:
lds     temp_reg2, UCSR1A
sbrs    temp_reg2, UDRE1
rjmp    wait_for_buffer_clear           ; Wait until buffer is clear
sts     UDR1, temp_reg                  ; Send data
wait_for_transmission_complete:
lds     temp_reg2, UCSR1A
sbrs    temp_reg2, TXC1
rjmp    wait_for_transmission_complete  ; Wait until transmission is complete
ldi     temp_reg2, 0x60                 ; Clear transmit complete flag
sts     UCSR1A, temp_reg2
ret
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
; Initialize Timer 4 for ADC timing
initialize_timer4:
ldi     timer4_tick_count, TICKS_PER_SECOND
ldi     seconds_count_timer4, 0x00
ret

; Initialize a generic timer with flexible second intervals
initialize_generic_timer:
ldi     generic_timer_tick_count, TICKS_PER_SECOND
ldi     generic_seconds_count, 0x00
ret

; Short delay subroutine, approx. 500 ms
delay_short:
ldi     generic_timer_tick_count, TICKS_PER_SECOND
ldi     generic_seconds_count, 0x00
ldi     temp_reg, 0x01
sei
delay_loop_short:
cp      generic_seconds_count, temp_reg
brlo    delay_loop_short
ret

; Long delay subroutine, approx. 2 seconds
delay_long:
ldi     generic_timer_tick_count, TICKS_PER_SECOND
ldi     generic_seconds_count, 0x00
ldi     temp_reg, 0x04
sei
delay_loop_long:
cp      generic_seconds_count, temp_reg
brlo    delay_loop_long
ret

; Check ADC for motor speed and stall detection
monitor_adc_for_stall:
cli                                             ; Disable interrupts during ADC check
cpi     seconds_count_timer4, ADC_START_DELAY
brsh    continue_adc_check                      ; Skip initial delay period
sei                                             ; Re-enable interrupts
ret

continue_adc_check:
ldi     temp_reg, 0xC3                   		; Setup ADC: enable, start conversion, prescaler
sts     ADCSRA, temp_reg
wait_for_conversion:
lds     temp_reg, ADCSRA
sbrs    temp_reg, ADIF                   		; Wait for ADC interrupt flag
rjmp    wait_for_conversion
lds     temp_reg, ADCH                   		; Read high byte of ADC result
cpi     temp_reg, MOTOR_STALL_THRESHOLD
brlo    handle_adc_stall                 		; If below threshold, handle potential stall
ldi     timer4_tick_count, TICKS_PER_SECOND  	; Reset timer
ldi     seconds_count_timer4, ADC_RESET_DELAY
ldi     temp_reg, 0x13                   		; Turn off ADC and clear ADIF
sts     ADCSRA, temp_reg
sei
ret

handle_adc_stall:
ldi     temp_reg, 0x13                   ; Turn off ADC and clear ADIF during stall handling
sts     ADCSRA, temp_reg
rjmp    signal_error_condition           ; Signal error condition due to stall
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
; SPI write subroutine used for setting up registers or controlling peripherals
spi_write_to_register:
sbi     PORTF, 1                         ; Set the SS pin to start SPI transaction
out     SPDR, temp_reg                   ; Start SPI transmission with byte1
spi_write_byte1:
lds     temp_reg, SPSR
sbrs    temp_reg, SPIF                   ; Wait for SPI transfer to complete
rjmp    spi_write_byte1
out     SPDR, temp_reg2                  ; Start SPI transmission with byte2
spi_write_byte2:
lds     temp_reg2, SPSR
sbrs    temp_reg2, SPIF                  ; Wait for SPI transfer to complete
rjmp    spi_write_byte2
cbi     PORTF, 1                         ; Clear the SS pin to end SPI transaction
ret
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
; Turn on motor by enabling PWM and configuring necessary registers
motor_turn_on:
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

; Turn off motor by disabling PWM
motor_turn_off:
ldi     temp_reg, 0x00
sts     TCCR1A, temp_reg                 ; Disable PWM, motor off
ldi     temp_reg, 0x0C
ldi     temp_reg2, 0x1A
rcall   spi_write_to_register            ; Disable motor through SPI
ret

; Signal an error condition, typically involves setting LEDs or sending alerts
signal_error_condition:
sbi     PORTC, 7                         ; Turn on yellow LED to indicate error
sbi     PORTD, 5                         ; Ensure green LED is off
cbi     PORTB, 0                         ; Turn on red LED to signal critical status
rcall   motor_turn_off                   ; Turn off motor immediately
rcall   enable_usart_receiver            ; Re-enable USART for further instructions
wait_for_error_resolution:
lds     temp_reg2, UCSR1A
sbrs    temp_reg2, RXC1
rjmp    wait_for_error_resolution
rcall   usart_receive_data               ; Check for error resolution command
cpi     temp_reg, 0x08
breq    reset_to_initial_state           ; If reset command received, go to initial state
cpi     temp_reg, 0x09
brne    wait_for_error_resolution
rcall   delay_long                       ; Long delay during error condition
rcall   enable_usart_transmitter
ldi     temp_reg, ERROR_CODE             ; Specific error code
rcall   usart_transmit_data
rcall   enable_usart_receiver
sbi     PORTB, 0                         ; Re-activate red LED after error handling
rcall   delay_long                       ; Final delay before looping back
ret

; Reset to initial state, typically involves reinitializing certain modules
reset_to_initial_state:
cli                                      ; Disable all interrupts
jmp     0x00                             ; Reset microcontroller
; ------------------------------------------------------------------------------

