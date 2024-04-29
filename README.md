## Multi-Microcontroller-Networking-System

This repository contains the AVR Assembly code for a networking system that involves multiple microcontrollers (MCUs), specifically designed around the ATmega32U4 chip. This system is structured to handle communications in a half-duplex manner across different nodes using USART, SPI, and TWI protocols.

## System Overview

The `Multi-Microcontroller-Networking-System` is designed to allow a master controller to communicate with multiple slave devices in a networked environment. The master controller manipulates the operations of various slave devices, which in turn manage peripheral devices such as motors or sensors.

### Chip Information

- **Microcontroller**: ATmega32U4
- **IDE**: Any IDE that supports AVR programming (e.g., Atmel Studio)
- **Language**: AVR Assembly
- **Reference**: Refer to the [Atmel(ATmega32U4) Documentation](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf) for detailed chip specifications and programming information.

### Communication Protocols

- **USART**: Utilized for half-duplex communication between the master and slave devices, allowing for message passing in both directions, but only one direction at a time, which simplifies the design and reduces potential data collision.
- **SPI**: Employed by slaves to control high-speed peripherals.
- **TWI (Two Wire Interface, I²C)**: Used for interfacing with sensors and low-speed peripherals that require sequential data transfer.

## Half-Duplex Communication

In half-duplex communication, data transmission can occur in both directions between devices, but only one direction can be active at any one time. This setup is ideal for our application where the master controller coordinates multiple slaves, ensuring clear and collision-free communication sessions initiated either by the master or by a slave responding to a request.

### Master Device

The master controller initializes and manages the communication protocol, sending commands to slaves and processing incoming data. This central management helps in maintaining an organized network flow and in processing asynchronous responses from multiple slaves.

### Slave Devices

Each slave listens for commands from the master, executes the received commands, controls peripherals as needed, and sends back the status or data requested by the master. Slaves use SPI or TWI to manage specific tasks assigned by the master.

### Achieving Half-Duplex Timing

In a half-duplex system, the communication channel (data bus) is shared, meaning the master and slave devices cannot transmit simultaneously — they must take turns. 
1. **Master Initiated Communication:** The master begins the communication. We are using a 8 bit microcontroller, but the USART we use can send up to 10 bits so we can send up to 10 bits for desired data.

2. **Data Transmission:** The data we send can be either an address to a specific microcontroller like a ping or specific command we wish the slave to perform. The slave should be able to intrepret the command from the master device. We have to make sure the data bus is cleared. This is important for half-duplex communication.

3. **Slave Response:**  Once the slave has processed the request, it sends a response back to the master. To avoid collision, the slave only transmits after it has received and processed the message from the master.


## Interrupt

Interrupts allow the microcontroller to pause its current tasks to respond to important events like receiving data. The program will perform regular checks such as error checking, ADC checking, and over-torque checking until an interrupt hits.

### Receiving Data: 
When data arrives at the USART interface, the Receive Complete Interrupt `(RXCIE1)` is triggered. This interrupt ensures that the microcontroller can immediately process incoming data, thereby minimizing latency.

### Data Transmission Ready: 
The Data Register Empty Interrupt `(UDRIE1)` can be employed to indicate that the USART data register is ready to accept new data for transmission. This is useful for efficiently managing transmission queues and ensuring the USART transmitter is utilized without delay.

## Design Overview

### Master to Slave Communication

1. **Initialization**:
   ```asm
   USART_Init:
   ldi r16, USART_BAUD_PRESCALE   
   sts UBRR1H, r16                
   ldi r16, 0x06                  
   sts UCSR1B, r16                
   ret
   ```

2. **Sending Commands:**
    ```asm
    USART_Tx:
    ldi r16, <COMMAND_CODE>         
    rcall USART_Tx                  
    ret
    ```

### Slave Response and Peripheral Control
1. **Receive Command:**
    ```asm
    USART_Rx:
    lds r16, UDR1                  
    cpi r16, PERIPHERAL_COMMAND
    breq EXECUTE_PERIPHERAL
    ret
    ```
2. **Using Polling Loop:**
    ```asm
    POLLING:
    lds	temp, UCSR1A			
	sbrs    temp, RXC1
	rjmp    POLLING
	rcall   USART_Rx
    
    ```

3. **Using Interrupt:**
    ```asm
    .org 0x0032
    rjmp USART_RX_Complete
    
    
    USART_RX_Complete:
    push    r16                   
    in      r16, SREG               
    push    r16
    lds     r16, UDR1               

    ; Process r16 Here
    
    pop     r16                    
    out     SREG, r16              
    pop     r16
    reti                       
    
    USART_Init:
    ldi r16, (1<<RXEN1)         
    ; Enable the receive complete interrupt
    ldi r17, (1<<RXCIE1)        
    sts UCSR1B, r16
    sts UCSR1B, r17             
    ; set up other stuff here like baud rate...
    

    ```

