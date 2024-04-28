#include "serial.h"

#include "stm32f303xc.h"

// NOTE: these are stored as pointers because they
//       are const values so we can't store them directly
//       in the struct
struct _SerialPort {
	volatile uint32_t *BaudRate;
	volatile uint32_t *ControlRegister1;
	volatile uint32_t *FlagClearRegister;
	volatile uint32_t *StatusRegister;
	volatile uint16_t *DataOutputRegister;
	volatile uint16_t *DataInputRegister;
	volatile uint32_t *TimerEnableRegister;
	volatile uint32_t TimerEnableMask;
	volatile uint32_t SerialPortGPIO;
	volatile uint32_t *SerialPinModeRegister;
	volatile uint32_t SerialPinModeValue;
	volatile uint32_t *SerialPinSpeedRegister;
	volatile uint32_t SerialPinSpeedValue;
	volatile uint8_t *SerialPinAlternatePinRegister;
	volatile uint8_t SerialPinAlternatePinValue;
	void (*completion_function)(uint32_t);
};


// The different serial ports require different GPIO ports
enum {
	SERIAL_GPIO_A,
	SERIAL_GPIO_B,
	SERIAL_GPIO_C,
	SERIAL_GPIO_D,
	SERIAL_GPIO_E
};



// instantiate the serial port parameters
//   note: the complexity is hidden in the c file
SerialPort USART1_PORT = {&(USART1->BRR),
		&(USART1->CR1),
		&(USART1->ICR),
		&(USART1->ISR),
		&(USART1->TDR),
		&(USART1->RDR),
		&(RCC->APB2ENR),
		RCC_APB2ENR_USART1EN,
		SERIAL_GPIO_C,
		&(GPIOC->MODER),
		0xA00,
		&(GPIOC->OSPEEDR),
		0xF00,
		((uint8_t*)&(GPIOC->AFR[0])) + 2,
		0x77};


// InitialiseSerial - Initialise the serial port
// Input: baudRate is from an enumerated set
void SerialInitialise(uint32_t baudRate, SerialPort *serial_port, void (*completion_function)(uint32_t)) {

	serial_port->completion_function = completion_function;

	// enable clock power, system configuration clock and GPIOC
	// common to all UARTs
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	switch(serial_port->SerialPortGPIO) {
	case SERIAL_GPIO_C:
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		break;
	default:
		break;
	}

	// set pin mode
	*(serial_port->SerialPinModeRegister) = serial_port->SerialPinModeValue;

	// enable high speed clock for GPIOC
	*(serial_port->SerialPinSpeedRegister) = serial_port->SerialPinSpeedValue;

	// set alternate function to enable USART to an external pin
	*(serial_port->SerialPinAlternatePinRegister) = serial_port->SerialPinAlternatePinValue;

	*(serial_port->TimerEnableRegister) |= serial_port->TimerEnableMask;

	uint16_t *baud_rate_config = ((uint16_t*)serial_port->BaudRate); // only 16 bits used!

	// Baud rate calculation from datasheet
	switch(baudRate){
	case BAUD_9600:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_19200:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_38400:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_57600:
		// NEED TO FIX THIS !
		*baud_rate_config = 0x46;  // 115200 at 8MHz
		break;
	case BAUD_115200:
		*baud_rate_config = 0x46 * 0x06;  // 115200 at 8MHz
		break;
	}


	// enable serial port for tx and rx
	*(serial_port->ControlRegister1) |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}


void SerialOutputChar(uint8_t data, SerialPort *serial_port) {
	while((*(serial_port->StatusRegister) & USART_ISR_TXE) == 0){
	}

	*(serial_port->DataOutputRegister) = data;
}



void SerialOutputString(uint8_t *pt, SerialPort *serial_port) {

	uint32_t counter = 0;
	while(*pt) {
		SerialOutputChar(*pt, serial_port);
		counter++;
		pt++;
	}

	if (serial_port->completion_function != 0x00)
		serial_port->completion_function(counter);
}


// returns 1 if valid char, 0 if timeout
uint8_t SerialReceiveChar(SerialPort *serial_port, uint8_t *received_char)
{
	uint8_t latest_character = 0;

	uint16_t timeout = 0xffff;
  while (1) {
	  timeout--;
	  if (timeout == 0)
		  return 0;

	if (*(serial_port->StatusRegister) & USART_ISR_ORE || *(serial_port->StatusRegister) & USART_ISR_FE) {
		*(serial_port->FlagClearRegister) |= USART_ICR_ORECF | USART_ICR_FECF;
	}

	if (*(serial_port->StatusRegister) & USART_ISR_RXNE) { // Wait for RXNE flag to be set
		break;
	}
  }
  *received_char = *(serial_port->DataInputRegister);
  return 1;
}


void SerialOutputBuffer(uint8_t *buffer, uint16_t buffer_length, SerialPort *serial_port) {
	uint32_t counter = 0;

	while(counter <= buffer_length) {
		SerialOutputChar(*buffer, serial_port);
		counter++;
		buffer++;
	}

	// This code was added as the python reading program was not picking
	//  up the next sentinel properly. This should be removed when the
	//  python code is fixed.
	uint8_t complete_buffer = '\0';
	SerialOutputChar(&complete_buffer, serial_port);

//	if (serial_port->completion_function != 0x00)
//		serial_port->completion_function(counter);
}

// Constants
//#define SENTINEL_1 0xAA
//#define SENTINEL_2 0x55

#include "serialise.h"

uint16_t SerialInputPacketHeader(char *buffer, SerialPort *serial_port)
{
  //uint8_t latest_characters[2] = {0};
  // wait for sentinels
   while (1) {
	   buffer[0] = buffer[1];
	   uint8_t success = SerialReceiveChar(serial_port, &buffer[1]);

	   if (success == 0)
		   return 0;

	  if (buffer[0] == SENTINEL_1 && buffer[1] == SENTINEL_2) {
			  break;
	  }
  }

   uint16_t index = 2;

  while (index < sizeof(Header))
  {
	  uint8_t receivedChar;
	  uint8_t success = SerialReceiveChar(serial_port, &receivedChar);
	  if (success == 0)
		  return 0;

      buffer[index++] = receivedChar;
  }

  return index;
}


uint16_t SerialInputDataPacket(char *buffer, int length, SerialPort *serial_port) {
	uint16_t index = 0;

  while (index < length)
  {
	  uint8_t receivedChar;
	  uint8_t success = SerialReceiveChar(serial_port, &receivedChar);
	  if (success == 0)
		  return 0;

	  buffer[index++] = receivedChar;
  }

  return index;

}

