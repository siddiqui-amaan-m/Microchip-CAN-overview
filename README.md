# Software Architecture Overview
The CAN communication software architecture for this project is structured around managing CAN bus communication between a microcontroller and external devices. The architecture involves several key components and functions that work together to initialize the CAN module, transmit data (such as voltage and current), and receive data. The software is designed to be modular, with each component having a distinct role in the communication process.

1. Core Components of the Software Architecture
1.1 CAN Module
The CAN module is the central part of the communication system. It is responsible for:

Setting up communication between the microcontroller and the CAN bus.
Handling message transmission and reception based on CAN protocol standards.
The architecture uses CAN0 of the SAMC21 microcontroller, leveraging Atmel Software Framework (ASF) functions for initialization, transmission, and reception. The CAN instance is defined as a global structure and is used across the software to access the CAN functionalities.

2. Key Functional Blocks
2.1 CAN Pin Configuration
Before enabling CAN communication, the system must properly configure the microcontroller’s physical pins for CAN transmission and reception. This is achieved by configuring:

The TX pin (for sending CAN messages).
The RX pin (for receiving CAN messages).
This ensures that the CAN signals are routed through the correct pins and multiplexed for CAN0. The pin configuration occurs at the hardware abstraction layer, where specific pins are assigned for communication.

2.2 CAN Initialization
The CAN initialization block is responsible for:

Configuring the CAN module with the appropriate settings (baud rate, CAN mode, etc.).
Switching the CAN module to normal operation mode so it can actively send and receive messages.
Enabling system interrupts to allow the microcontroller to respond when new CAN messages arrive.
This block initializes the CAN0 module to start communication, ensuring it adheres to the standard CAN protocol and that the CAN bus can be used for real-time data transmission.

2.3 CAN Message Transmission
The transmission block handles the process of sending data over the CAN bus. In this system, the architecture splits two 16-bit values (voltage and current) into low byte and high byte components. The splitting of these values is critical for transmitting larger values using CAN’s 8-bit data fields.

Data Formatting: The 16-bit data is formatted by extracting the low byte and high byte and positioning them into specific bit ranges (bits 11-18 and bits 0-7, respectively) within the CAN message.
ID and Control Information: The 29-bit extended CAN ID is constructed, which includes fields such as ID, Message Type, Priority, and To/From flags.
Multiple Transfers: Two separate transfer commands are issued: one for the voltage and one for the current. Each command transmits its respective value after properly formatting it.
2.4 CAN Message Reception
The reception block is triggered by an interrupt when a new CAN message arrives. This block:

Reads incoming data from the CAN message and extracts specific bytes (e.g., low and high byte values) from predefined bit positions.
Combines these bytes into a 16-bit value.
Processes the data: The received data (voltage or current) is divided by 10 to restore its original format (as it was multiplied by 10 before transmission for precision).
This block ensures that the system can receive and correctly interpret CAN messages, responding appropriately to incoming data.

2.5 CAN Filtering
CAN networks can have many nodes sending various messages. The CAN filtering block helps ensure that the system only processes the messages intended for it. This is done by:

Setting up filters based on the ID and other fields (e.g., To/From bit).
Enabling interrupts when the system detects a message that passes through the filter.
This filtering prevents unnecessary messages from being processed, reducing overhead and improving system efficiency.

3. Interrupt-Driven Communication
The system uses interrupts to handle real-time communication without requiring the main program to continuously check for messages (polling). When a new message is received, an interrupt handler is invoked:

CAN0_Handler1 is the function that processes the incoming messages.
Data extraction occurs in the handler, combining the low and high bytes into a complete 16-bit value.
This interrupt-driven architecture ensures that the system remains responsive and efficient, only processing messages when necessary.

4. Modular Design
The software architecture is modular, with each function encapsulating a specific responsibility. This allows for easy debugging, maintenance, and extension of the code. The primary modules include:

CAN pin configuration.
CAN initialization.
CAN message transmission.
CAN message reception.
CAN filtering.
The modularity allows for future expansion, such as adding additional CAN messages or adjusting how messages are filtered and processed.

5. System Flow
The overall flow of the system is as follows:

System Initialization: Initializes the CAN pins, configures the CAN0 module, and enables interrupts.
Message Transmission: Transmits voltage and current values over the CAN bus.
Message Reception: Interrupts the system when a message is received, processes the data, and updates the received values for use within the system.
Main Loop: Continuously monitors for new messages while performing any additional system functions.
6. Error Handling and Debugging
The system is designed to be robust, but certain elements could be added to handle potential errors:

Transmission errors: Can occur during message transmission; error handling routines can check the status of the CAN bus after each transmission.
Timeouts: If no message is received within a specific period, the system could trigger a timeout routine.
In the current system, the architecture focuses on sending and receiving data, assuming that the CAN bus is functional and messages are being transmitted successfully. Future improvements could include better error reporting and fault tolerance mechanisms.

7. Conclusion
The software architecture of this CAN communication system provides a flexible, modular, and interrupt-driven approach to handling real-time data transmission and reception. It efficiently formats and sends 16-bit voltage and current values, manages message filtering, and processes received messages for further use. This architecture ensures that the system can operate reliably in a CAN-based network, exchanging data with other nodes on the bus.
