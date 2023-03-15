/*

    ADL3000-E 3-Phase Energy Meter
    chargeNET DC EVSE - MAR 2023
    Kaushan Ranasinghe

*/ 

#ifndef IMD_MODBUS_H_
#define IMD_MODBUS_H_

#include "imd_crc.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <main.h>


// Modbus ADU Params 

#define RX_BUFFER_SIZE 30 // UART Rx buffer size

#define IMD_SLAVE_ID 0x01
#define MAX_FRAME_SIZE 256 // max packet size standard
#define MAX_DATA_SIZE 252

// Modbus PDU Params

//#define HOLD_REG_OFFSET_ADDR 0x9C41 // Holding Register offset - 40000
#define READ_HOLDING_REGISTERS 0x03
#define SINGLE_REGISTER_WRITE 0x06

// Read Data Registers

#define BUS_VOLTAGE 0x0010
#define POSITIVE_TO_GROUND_OHMS 0x0012
#define NEGATIVE_TO_GROUND_OHMS 0x0013
#define EV_SIDE_VOLTAGE 0x0014
#define IO_STATUS 0x001B

// Write Data Registers

#define INSULATION_MONITORING_CONTROL 0x0102

// Write Commands

#define TURN_ON 0x11
#define TURN_OFF 0x00

// Modbus RTU = Application Data Unit + Protocol Data Unit (ADU[PDU]) 

// Packet Sizes

#define BYTE_1 0x00
#define BYTE_2 0x01
#define BYTE_4 0x02
#define BYTE_6 0x03
#define BYTE_8 0x04

// UART Rx buffer params

extern unsigned char rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t rx_head;
extern volatile uint8_t rx_tail;
extern uint16_t checksum;
extern uint16_t crc_calc;
extern uint16_t crc_rx;
extern uint16_t ascii_tx_high;
extern uint16_t ascii_tx_low;
extern int rx_hi;
extern int rx_lo;
extern uint16_t result[10];
extern uint8_t TxData[8];

// Modbus-RTU ADU - include PDU + CRC in a frame

struct modbus_adu_imd{
	uint8_t slave_addr;
	uint8_t function_code;
	uint16_t start_addr; // provide MSB first
	uint16_t num_regs;
	uint8_t window;
    uint8_t read_length; // CRC provide LSB first
    uint16_t write_data;
    uint8_t write_length;
    uint8_t frame[];
};

struct byte_pckt_imd{
	unsigned int val;
	unsigned int bit0 : 1;
	unsigned int bit1 : 1;
	unsigned int bit2 : 1;
	unsigned int bit3 : 1;
	unsigned int bit4 : 1;
	unsigned int bit5 : 1;
	unsigned int bit6 : 1;
	unsigned int bit7 : 1;
};

// Functions

void ADU_read(struct modbus_adu_imd *adu,uint16_t val1,uint16_t val2);
void ADU_write(struct modbus_adu_imd *adu, uint16_t instruction, uint16_t byte_length, uint16_t data);
bool rx_pckt_verify(struct modbus_adu_imd *adu, unsigned char *rx_buffer);
void sendData(UART_HandleTypeDef *huart, uint8_t *data);
void rx_decode(struct modbus_adu_imd *adu, unsigned char *rx_buffer);
void imd_read(UART_HandleTypeDef *huart, struct modbus_adu_imd *adu,uint16_t instruction, uint16_t byte_length);
void imd_write(UART_HandleTypeDef *huart, struct modbus_adu_imd *adu,uint16_t instruction, uint16_t byte_length, uint16_t data);


#endif
