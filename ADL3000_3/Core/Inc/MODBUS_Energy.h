/*

    ADL3000-E 3-Phase Energy Meter
    chargeNET DC EVSE - MAR 2023
    Kaushan Ranasinghe

*/ 

#ifndef MODBUS_ENERGY_H_
#define MODBUS_ENERGY_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "crc_adl3000.h"

// Modbus ADU Params 

#define RX_BUFFER_SIZE 30 // UART Rx buffer size

#define ADL_SLAVE_ID 0x02
#define MAX_FRAME_SIZE 256 // max packet size standard
#define MAX_DATA_SIZE 252

// Modbus PDU Params

//#define HOLD_REG_OFFSET_ADDR 0x9C41 // Holding Register offset - 40000
#define READ_HOLDING_REGISTERS 0x03
#define MULTIPLE_REGISTER_WRITE 0x10

// Read Data Registers

#define CURRENT_TOTAL_ACTIVE_ENERGY 0x0000
#define CURRENT_SPIKE_TOTAL_ACTIVE_ENERGY 0x0002
#define CURRENT_PEAK_TOTAL_ACTIVE_ENERGY 0x0004
#define CURRENT_FLAT_TOTAL_ACTIVE_ENERGY 0x0006
#define CURRENT_VALLEY_TOTAL_ACTIVE_ENERGY 0x0008
#define CURRENT_FORWARD_ACTIVE_TOTAL_ENERGY 0x000A
#define CURRENT_FORWARD_ACTIVE_SPIKE_ENERGY 0x000C
#define CURRENT_FORWARD_ACTIVE_PEAK_ENERGY 0x000E
#define CURRENT_FORWARD_ACTIVE_FLAT_ENERGY 0x0010
#define CURRENT_FORWARD_ACTIVE_VALLEY_ENERGY 0x0012
#define CURRENT_REVERSING_ACTIVE_TOTAL_ENERGY 0x0014
#define CURRENT_REVERSING_ACTIVE_SPIKE_ENERGY 0x0016
#define CURRENT_REVERSING_ACTIVE_ENERGY 0x0018
#define CURRENT_REVERSING_ACTIVE_FLAT_ENERGY 0x001A
#define CURRENT_REVERSING_ACTIVE_VALLEY_ENERGY 0x001C
#define CURRENT_TOTAL_REACTIVE_ENERGY 0x001E
#define CURRENT_TOTAL_REACTIVE_SPIKE_ENERGY 0x0020
#define CURRENT_TOTAL_REACTIVE_PEAK_ENERGY 0x0022
#define CURRENT_TOTAL_REACTIVE_FLAT_ENERGY 0x0024
#define CURRENT_TOTAL_REACTIVE_VALLEY_ENERGY 0x0026
#define CURRENT_FORWARD_REACTIVE_TOTAL_ENERGY 0x0028
#define CURRENT_FORWARD_REACTIVE_SPIKE_ENERGY 0x002A
#define CURRENT_FORWARD_REACTIVE_PEAK_ENERGY 0x002C
#define CURRENT_FORWARD_REACTIVE_FLAT_ENERGY 0x002E
#define CURRENT_FORWARD_REACTIVE_VALLEY_ENERGY 0x0030
#define CURRENT_REVERSING_REACTIVE_TOTAL_ENERGY 0x0032
#define CURRENT_REVERSING_REACTIVE_SPIKE_ENERGY 0x0034
#define CURRENT_REVERSING_REACTIVE_PEAK_ENERGY 0x0036
#define CURRENT_REVERSING_REACTIVE_FLAT_ENERGY 0x0038
#define CURRENT_REVERSING_REACTIVE_VALLEY_ENERGY 0x003A

#define DATE_TIME_ss_mm 0x003C
#define DATE_TIME_hh_dd 0x003D
#define DATE_TIME_MM_yy 0x003E

#define FIRST_COMMUNICATION_PATH_ADDRESS 0x003F
#define FIRST_COMMUNICATION_PATH_BAUD_RATE 0x003F
#define PULSE_CONSTANT 0x0040

//#define 4_TIME_ZONES
//#define 1_8PERIOD_OF_TIME_PARAMETERS_SETTING_INFORMATION
//#define 1_9PERIOD_OF_TIME_PARAMETERS_SETTING_INFORMATION

#define VOLTAGE_OF_A_PHASE 0x0061
#define VOLTAGE_OF_B_PHASE 0x0062
#define VOLTAGE_OF_C_PHASE 0x0063
#define CURRENT_OF_A_PHASE 0x0064
#define CURRENT_OF_B_PHASE 0x0065
#define CURRENT_OF_C_PHASE 0x0066

#define FREQUENCY 0x0077
#define VOLTAGE_BETWEEN_A_B 0x0078
#define VOLTAGE_BETWEEN_C_B 0x0079
#define VOLTAGE_BETWEEN_A_C 0x007A
#define FORWARD_ACTIVE_MAXIMUM_DEMAND 0x007B
#define TIME_OF_OCCURRENCE_FOR_THE_FORWARD_ACTIVE_MAXIMUM_AMOUNT_mm_hh 0x007C
#define TIME_OF_OCCURRENCE_FOR_THE_FORWARD_ACTIVE_MAXIMUM_AMOUNT_dd_MM 0x007D
#define REVERSING_ACTIVE_MAXIMUM_DEMAND 0x007E
#define TIME_OF_OCCURRENCE_FOR_THE_REVERSING_ACTIVE_MAXIMUM_AMOUNT_mm_hh 0x007F
#define TIME_OF_OCCURRENCE_FOR_THE_REVERSING_ACTIVE_MAXIMUM_AMOUNT_dd_MM 0x0080
#define MAXIMUM_FORWARD_DEMAND_FOR_REACTIVE_POWER 0x0081
#define TIME_OF_OCCURRENCE_FOR_THE_FORWARD_REACTIVE_MAXIMUM_AMOUNT_mm_hh 0x0082
#define TIME_OF_OCCURRENCE_FOR_THE_FORWARD_REACTIVE_MAXIMUM_AMOUNT_dd_MM 0x0083
#define MAXIMUM_REVERSING_DEMAND_FOR_REACTIVE_POWER 0x0084
#define TIME_OF_OCCURRENCE_FOR_THE_REVERSING_REACTIVE_MAXIMUM_AMOUNT_mm_hh 0x0085
#define TIME_OF_OCCURRENCE_FOR_THE_REVERSING_REACTIVE_MAXIMUM_AMOUNT_dd_MM 0x0086
#define FORWARD_ACTIVE_ENERGY_OF_A_PHASE 0x0087
#define FORWARD_ACTIVE_ENERGY_OF_B_PHASE 0x0089
#define FORWARD_ACTIVE_ENERGY_OF_C_PHASE 0x008B
#define VOLTAGE_TRANSFER 0x008D
#define CURRENT_TRANSFER 0x008E
#define STATE_OF_DIDO_OVER_VOLTAGE 0x008F

#define RUNNING_STATE_1 0x0091
#define RUNNING_STATE_2 0x0091
#define ZERO_SEQUENCE_CURRENT 0x0092
#define VOLTAGE_IMBALANCE 0x0093
#define CURRENT_IMBALANCE 0x0094

/*
#define FIRST_COMMUNICATION_PATH_TESTING_BYTE
#define SECOND_COMMUNICATION_PATH_ADDRESS_HIGH_8_BYTES_BAUD_RATE_LOW_8_BYTES
#define SECOND_COMMUNICATION_PATH:TESTING_BYTE_HIGH_8_BYTES
#define STOP_BYTE_LOW_8_BYTES
#define RESERVED

//#define 9_14_PERIOD_OF_TIME_PARAMETERS_SETTING_INFORMATION
//#define 9_14_PERIOD_OF_TIME_PARAMETERS_SETTING_INFORMATION

#define RESERVED
#define ACTIVE_POWER_OF_A_PHASE
#define ACTIVE_POWER_OF_B_PHASE
#define ACTIVE_POWER_OF_C_PHASE
#define TOTAL_ACTIVE_POWER
#define REACTIVE_POWER_OF_A_PHASE Reactive_Power_of_A_Phase
#define REACTIVE_POWER_OF_B_PHASE Reactive_Power_of_B_Phase
#define REACTIVE_POWER_OF_C_PHASE Reactive_Power_of_C_Phase
#define TOTAL_REACTIVE_POWER Total_Reactive_Power

#define APPARENT_POWER_OF_A_PHASE Apparent_Power_of_A_Phase
#define APPARENT_POWER_OF_B_PHASE Apparent_Power_of_B_Phase
#define APPARENT_POWER_OF_C_PHASE Apparent_Power_of_C_Phase
#define TOTAL_APPARENT_POWER Total_Apparent_Power

#define POWER_FACTOR_OF_A_PHASE Power_Factor_of_A_Phase
#define POWER_FACTOR_OF_B_PHASE Power_Factor_of_B_Phase
#define POWER_FACTOR_OF_C_PHASE Power_Factor_of_C_Phase
#define TOTAL_POWER_FACTOR Total_Power_Factor

#define MAXIMUM_FORWARD_ACTIVE_DEMAND_A_DAY Maximum_Forward_Active_Demand_A_Day
#define OCCUR_TIME_1 Occur_Time_1
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_A_DAY Maximum_Reversing_Active_Demand_A_Day
#define OCCUR_TIME_2 Occur_Time_2
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_A_DAY Maximum_Forward_Reactive_Demand_A_Day
#define OCCUR_TIME_3 Occur_Time_3
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_A_DAY Maximum_Reversing_Reactive_Demand_A_Day
#define OCCUR_TIME_4 Occur_Time_4
#define MAXIMUM_FORWARD_ACTIVE_DEMAND_LAST_DAY Maximum_Forward_Active_Demand_Last_Day
#define OCCUR_TIME_5 Occur_Time_5
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_LAST_DAY Maximum_Reversing_Active_Demand_Last_Day
#define OCCUR_TIME_6 Occur_Time_6
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_LAST_DAY Maximum_Forward_Reactive_Demand_Last_Day
#define OCCUR_TIME_7 Occur_Time_7
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_DAY Maximum_Reversing_Reactive_Demand_Last_Day
#define OCCUR_TIME_8 Occur_Time_8

#define MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_DAY_2 Occur_Time_9
#define MAXIMUM_FORWARD_ACTIVE_DEMAND_LAST_2_DAYS Maximum_Forward_Active_Demand_Last_2_Days
#define OCCUR_TIME_10 Occur_Time_10
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_LAST_2_DAYS Maximum_Reversing_Active_Demand_Last_2_Days
#define OCCUR_TIME_11 Occur_Time_11
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_LAST_2_DAYS Maximum_Forward_Reactive_Demand_Last_2_Days
#define OCCUR_TIME_12 Occur_Time_12
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_2_DAYS Maximum_Reversing_Reactive_Demand_Last_2_Days
#define OCCUR_TIME_13 Occur_Time_13

#define CURRENT_FORWARD_ACTIVE_DEMAND Current_Forward_Active_Demand
#define CURRENT_REVERSING_ACTIVE_DEMAND Current_Reversing_Active_Demand
#define CURRENT_FORWARD_REACTIVE_DEMAND Current_Forward_Reactive_Demand
#define CURRENT_REVERSING_REACTIVE_DEMAND Current_Reversing_Reactive_Demand

#define MAXIMUM_VOLTAGE_ON_A_PHASE Maximum_Voltage_on_A_Phase
#define OCCUR_DATE_1 Occur_Date_1
#define OCCUR_TIME_14 Occur_Time_14
#define MAXIMUM_VOLTAGE_ON_B_PHASE_AND_OCCUR_TIME Maximum_Voltage_on_B_Phase_and_Occur_Time
#define MAXIMUM_VOLTAGE_ON_C
*/
// Modbus RTU = Application Data Unit + Protocol Data Unit (ADU[PDU]) 

// Modbus PDU - slave_addr + function code + data[start_addr + num regs] 

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


// Modbus-RTU ADU - include PDU + CRC in a frame

struct modbus_adu{
	uint8_t slave_addr;
	uint8_t function_code;
	uint16_t start_addr; // provide MSB first
	uint16_t num_regs;
	uint8_t window;
    uint8_t read_length; // CRC provide LSB first
    uint8_t write_length;
    uint8_t frame[];

};

// Instantiating the structs

//extern struct modbus_adu adu_t;


// Functions

void ADU_read(struct modbus_adu *adu,uint16_t val1,uint16_t val2);
bool rx_pckt_verify(struct modbus_adu *adu, unsigned char *rx_buffer);
void sendData(uint8_t *data);

void rx_decode(struct modbus_adu *adu, unsigned char *rx_buffer);


#endif
