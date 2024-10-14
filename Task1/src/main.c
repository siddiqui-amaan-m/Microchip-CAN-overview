#include <asf.h>  // Include the Atmel Software Framework

#define CAN_MODULE CAN0  // Define the CAN module to use CAN0

// Define TX and RX Pins (Pin 23 = PA24, Pin 24 = PA25)
#define CAN_TX_PIN   PIN_PA24G_CAN0_TX
#define CAN_RX_PIN   PIN_PA25G_CAN0_RX

#define CAN_TX_MUX_SETTING MUX_PA24G_CAN0_TX
#define CAN_RX_MUX_SETTING MUX_PA25G_CAN0_RX



// Declare the CAN instance structure
struct can_module can_instance; // adding this line was never mentioned in the datasheet

//Define variables
uint16_t set_voltage = 2205;  // 220.5V * 10
uint16_t set_current = 200;   // 20.0A * 10
float received_value;
float rec_voltage = 0;  
float rec_current = 0;

// Function to configure CAN pins
void configure_can_pins(void) {
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);

	// Configure CAN TX (Pin 23 / PA24)
	pin_config.mux_position = CAN_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);

	// Configure CAN RX (Pin 24 / PA25)
	pin_config.mux_position = CAN_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);
}

// Function to initialize the CAN module
void configure_can(void) {
	// Step 1: Create a configuration structure
	struct can_config config_can;
	
	// Step 2: Get the default CAN configuration
	can_get_config_defaults(&config_can);
	
	// Step 3: Initialize the CAN module (CAN0 in this case) with the default configuration
	can_init(&can_instance, CAN_MODULE, &config_can);
	
	// Step 4: Switch the CAN module to normal operation mode
	can_start(&can_instance);   // Lot of trouble to find this line
	
	// Step 5: Enable system interrupts for the CAN0 module
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
}



void send_can_message(void) {
	struct can_tx_element tx_element;
	uint32_t extended_id;
	uint32_t message_value;  // This will hold the combined Data0 and Data1

	// 1. First CAN Transfer Command: Sending "set_voltage" (2205)
	message_value = 0;  // Clear previous data
	message_value |= ((set_voltage & 0xFF) << 11);  // Place Data0 (low byte) in bits 11-18
	message_value |= ((set_voltage >> 8) & 0xFF);   // Place Data1 (high byte) in bits 0-7

	// Set default CAN transmission buffer
	can_get_tx_buffer_element_defaults(&tx_element);

	// Construct the 29-bit extended ID for the first transfer command
	extended_id = 0;
	extended_id |= (1 << 24);  // Set ID (1) at bits 24-28
	extended_id |= (1 << 19);  // Set Message Type (1) at bits 19-23
	extended_id |= (1 << 9);   // Set Priority (1) at bits 9-10
	extended_id |= (1 << 8);   // Set To/From (1) at bit 8

	// Assign the extended ID and manually packed message_value to tx_element.T0
	// Combine extended_id into T0
	tx_element.T0.reg |= extended_id; // Assign the extended ID to T0
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_XTD; // Set the extended ID flag
	tx_element.T1.reg = message_value;  // Write Data0 and Data1 to the message part (T1 register)

	// Send the first CAN message (Voltage: 2205)
	can_set_tx_buffer_element(&can_instance, &tx_element, 0);
	can_tx_transfer_request(&can_instance, 1 << 0);

	// 2. Second CAN Transfer Command: Sending "set_current" (200)
	// Manually position the bits
	message_value = 0;  // Clear previous data
	message_value |= ((set_current & 0xFF) << 11);  // Place Data0 (low byte) in bits 11-18
	message_value |= ((set_current >> 8) & 0xFF);   // Place Data1 (high byte) in bits 0-7

	// Set default CAN transmission buffer again
	can_get_tx_buffer_element_defaults(&tx_element);

	tx_element.T0.reg |= extended_id; // Assign the extended ID to T0
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_XTD; // Set the extended ID flag
	tx_element.T1.reg = message_value;  // Write Data0 and Data1 to the message part (T1 register)

	// Send the second CAN message (Current: 200)
	can_set_tx_buffer_element(&can_instance, &tx_element, 0);
	can_tx_transfer_request(&can_instance, 1 << 0);
}

void configure_can_filter(void) {
	struct can_extended_message_filter_element et_filter;
	
	can_get_extended_message_filter_element_default(&et_filter);

	// Set the ID = 2 at bit positions 24 to 28
	et_filter.F0.bit.EFID1 = (2 << 24);  
	
	// Set the To/From field to 0 at bit position 8
	et_filter.F0.bit.EFID1 |= (0 << 8);  
	
	// Set filter to accept new messages in RX FIFO 0
	et_filter.F0.bit.EFEC = CAN_RX_FIFO_0_NEW_MESSAGE;

	// Mask all bits except the ones at positions 24 to 28 for the ID and 8 for To/From
	et_filter.F1.bit.EFID2 = 0x1F010000; 

	// Apply the extended message filter to the CAN instance
	can_set_rx_extended_filter(&can_instance, &et_filter, 0);
	
	// Enable interrupt for receiving new messages in FIFO 0
	can_enable_interrupt(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
}

volatile uint16_t combined_value = 0;  

float  CAN0_Handler1(void) {
	uint32_t status = can_read_interrupt_status(&can_instance);

	// Check if there is a new message in FIFO 0
	if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
		can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);

		// Read the received message from FIFO 0
		struct can_rx_element_fifo_0 rx_element;
		can_get_rx_fifo_0_element(&can_instance, &rx_element, 0);
		can_rx_fifo_acknowledge(&can_instance, 0, 0);

		// Extract Data0 (low byte) from bit positions 11 to 18
		uint8_t low_byte = (rx_element.R1.reg >> 11) & 0xFF;  // Shift right by 11 and mask to get the low byte
		
		// Extract Data1 (high byte) from bit positions 0 to 7
		uint8_t high_byte = rx_element.R1.reg & 0xFF;  
		// Combine low and high bytes into a 16-bit variable
		received_value = (high_byte << 8) | low_byte;  // High byte in position 8-15, low byte in position 0-7
		
		// Divide the combined value by 10
		received_value /= 10;
	}
	return received_value;
}


int main(void) {
	// Initialize system
	system_init();

	// Configure CAN pins
	configure_can_pins();

	// Initialize CAN
	configure_can();
	
	// Send voltage and current
	send_can_message();

	rec_voltage=CAN0_Handler1();
	rec_current=CAN0_Handler1();

	while (1) {
		// Main loop
	}
}
