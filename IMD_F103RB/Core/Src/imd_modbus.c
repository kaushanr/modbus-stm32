#include "imd_modbus.h"
#include "main.h"
#include "stdint.h"

// UART Tx/Rx Buffers / vars

unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

uint16_t checksum = 0;
uint16_t crc_calc = 0;
uint16_t crc_rx = 0;

uint8_t TxData[8];

uint16_t result[10] = {};
struct byte_pckt_imd io_stat_reg = {0};

int rx_hi = 0;
int rx_lo = 0;

// Assemble ADU packet

void ADU_read(struct modbus_adu_imd *adu, uint16_t instruction, uint16_t byte_length)
{
	//adu->slave_addr = IMD_SLAVE_ID;
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

void ADU_write(struct modbus_adu_imd *adu, uint16_t instruction, uint16_t byte_length, uint16_t data)
{
	adu->slave_addr = IMD_SLAVE_ID;
	adu->function_code = SINGLE_REGISTER_WRITE;
	adu->start_addr = instruction;
	adu->write_data = data;

    adu->frame[0] = adu->slave_addr; // add slave_addr to adu frame[0] from pointer
    adu->frame[1] = adu->function_code; // add function_code to adu frame[0] from pointer
    adu->frame[2] = adu->start_addr >> 8 & 0xFF;
    adu->frame[3] = adu->start_addr & 0xFF;
    adu->frame[4] = adu->write_data >> 8 & 0xFF;
	adu->frame[5] = adu->write_data & 0xFF;

    uint16_t crc = crc16_1(adu->frame, 6); // calculate CRC for PDU only
    adu->frame[6] = crc & 0xff; // store LSB
    adu->frame[7] = crc >> 8; // store MSB

    adu->write_length = 8; // calculate and set ADU frame length

    if (byte_length != BYTE_1)
    {
    	adu->window = 3 + byte_length*2;
    }else{
    	adu->window = 3 + byte_length;
    }
}

void sendData(UART_HandleTypeDef *huart, uint8_t *data)
{
	rx_head = 0;
	rx_tail = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // tx enable
	HAL_UART_Transmit(huart, data, 8, 1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // rx enable
}

bool rx_pckt_verify(struct modbus_adu_imd *adu, unsigned char *rx_buffer)
{
	HAL_Delay(100); // wait before reading rx_buffer after callback

	rx_hi = rx_buffer[((adu->start_addr == INSULATION_MONITORING_CONTROL) ? (adu->write_length):(adu->read_length)) + (adu->window) + 1];
	rx_lo = rx_buffer[((adu->start_addr == INSULATION_MONITORING_CONTROL) ? (adu->write_length):(adu->read_length)) + (adu->window)];
	crc_calc = crc16_2(rx_buffer, ((adu->start_addr == INSULATION_MONITORING_CONTROL) ? (adu->write_length):(adu->read_length)), adu->window);
	crc_rx = rx_hi<<8|rx_lo;

	return crc_rx == crc_calc;
}

void rx_decode(struct modbus_adu_imd *adu, unsigned char *rx_buffer)
{
	int num_bytes = (((adu->start_addr == INSULATION_MONITORING_CONTROL) ? (adu->write_length):(adu->read_length))+2);
	int check = (adu->num_regs != BYTE_1)?(rx_buffer[num_bytes]/2):1;

	if (adu->start_addr == IO_STATUS)
		{
			result[0] = rx_buffer[num_bytes + 2];
			io_stat_reg.val = result[0];
			io_stat_reg.bit0 = (result[0] >> 0) & 1; // set bit0 of io_stat_reg to the least significant bit of result[0]
			io_stat_reg.bit1 = (result[0] >> 1) & 1; // set bit1 of io_stat_reg to the second least significant bit of result[0]
			io_stat_reg.bit2 = (result[0] >> 2) & 1;
			io_stat_reg.bit3 = (result[0] >> 3) & 1;
			io_stat_reg.bit4 = (result[0] >> 4) & 1;
			io_stat_reg.bit5 = (result[0] >> 5) & 1;
			io_stat_reg.bit6 = (result[0] >> 6) & 1;
			io_stat_reg.bit7 = (result[0] >> 7) & 1;
	}else{
			for (int i = 0; i < check; i++)
				{
					result[i] = ((rx_buffer[num_bytes + i + 1])<<8)|((rx_buffer[num_bytes + i + 2]));
				}
		}
}

void imd_read(UART_HandleTypeDef *huart, struct modbus_adu_imd *adu,uint16_t instruction, uint16_t byte_length)
{
	ADU_read(adu, instruction, byte_length);
	sendData(huart,adu->frame);
	memcpy(TxData, adu->frame, 8);
	if (rx_pckt_verify(adu, rx_buffer))
	{
		rx_decode(adu, rx_buffer);
	}else{
		return;
	}
}

void imd_write(UART_HandleTypeDef *huart, struct modbus_adu_imd *adu,uint16_t instruction, uint16_t byte_length, uint16_t data)
{
	ADU_write(adu, instruction, byte_length, data);
	sendData(huart,adu->frame);
	memcpy(TxData, adu->frame, 8);
	if (rx_pckt_verify(adu, rx_buffer))
	{
		rx_decode(adu, rx_buffer);
	}else{
		return;
	}
}





