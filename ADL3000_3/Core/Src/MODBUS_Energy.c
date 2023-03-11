#include "MODBUS_Energy.h"
#include "main.h"
#include "stdint.h"

// UART Tx/Rx Buffers / vars

unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
uint16_t checksum = 0;
uint16_t crc_tx = 0;
uint16_t crc_rx = 0;

uint16_t ascii_tx_high = 0;
uint16_t ascii_tx_low = 0;

uint16_t result[10] = {};

int tx_hi = 0;
int tx_lo = 0;

extern UART_HandleTypeDef huart1;

// Instantiating the structs

//struct modbus_adu adu_t = {};


// Assemble ADU packet

void ADU_read(struct modbus_adu *adu, uint16_t instruction, uint16_t byte_length)
{
	adu->slave_addr = ADL_SLAVE_ID;
	adu->function_code = READ_HOLDING_REGISTERS;
	adu->start_addr = instruction;
	adu->num_regs = byte_length;

    adu->frame[0] = adu->slave_addr; // add slave_addr to adu frame[0] from pointer
    adu->frame[1] = adu->function_code; // add function_code to adu frame[0] from pointer
    adu->frame[2] = adu->start_addr >> 8 & 0xFF;
    adu->frame[3] = adu->start_addr & 0xFF;
    adu->frame[4] = adu->num_regs >> 8 & 0xFF;
	adu->frame[5] = adu->num_regs & 0xFF;

    uint16_t crc = crc16_1(adu->frame, 6); // calculate CRC for PDU only
    adu->frame[6] = crc & 0xff; // store LSB
    adu->frame[7] = crc >> 8; // store MSB

    adu->read_length = 8; // calculate and set ADU frame length

    if (byte_length != BYTE_1)
    {
    	adu->window = 3 + byte_length*2;
    }else{
    	adu->window = 3 + byte_length;
    }
}

void sendData(uint8_t *data)
{
	//ADU_read(&adu_t, VOLTAGE_OF_A_PHASE, BYTE_2);
	rx_head = 0;
	rx_tail = 0;
    HAL_Delay(100);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, data, 8, 1000);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);
	//rx_pckt_verify(&adu_t, rx_buffer);
}

uint8_t ascii_to_hex(unsigned char c) {
    uint8_t val;

    if (c >= '0' && c <= '9') {
        val = c - '0';
    } else if (c >= 'a' && c <= 'f') {
        val = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        val = c - 'A' + 10;
    } else {
        val = c; // default value for non-hex characters
    }
    return val;
}

bool rx_pckt_verify(struct modbus_adu *adu, unsigned char *rx_buffer)
{
	HAL_Delay(100); // wait before reading rx_buffer after callback

	tx_hi = rx_buffer[(adu->read_length)+2];
	tx_lo = (adu->window);

    ascii_tx_high = ascii_to_hex(rx_buffer[(adu->read_length) + (adu->window) + 1])<<8;
    ascii_tx_low = ascii_to_hex(rx_buffer[(adu->read_length) + (adu->window)]);

	crc_rx = crc16_2(rx_buffer, adu->read_length, adu->window);
	crc_tx = ascii_tx_high|ascii_tx_low;

	return crc_rx == crc_tx;
}

void rx_decode(struct modbus_adu *adu, unsigned char *rx_buffer)
{
	int num_bytes = ((adu->read_length)+2);

	int check = (adu->num_regs != BYTE_1)?(rx_buffer[num_bytes]/2):1;

	for (int i = 0; i < check; i++)
	{
		result[i] = (ascii_to_hex(rx_buffer[num_bytes + i + 1])<<8)|(ascii_to_hex(rx_buffer[num_bytes + i + 2]));
	}
}







