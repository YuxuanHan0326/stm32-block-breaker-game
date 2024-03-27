/*
 * screen.c
 *
 *  Created on: Mar 12, 2024
 *      Author: Yuxuan Han
 */
#include "screen.h"

#define TIMEOUT 100

uint8_t rows[11] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x01, 0x03, 0x05, 0x07, 0x09};

uint8_t transfer_buffer[2];

void ISSI_write_reg(I2C_HandleTypeDef* handler, uint8_t reg, uint8_t data) {
	transfer_buffer[0] = reg;
	transfer_buffer[1] = data;

	HAL_I2C_Master_Transmit(handler, (ADDR << 1), transfer_buffer, 2, TIMEOUT);
}

void ISSI_write_PWM_reg(I2C_HandleTypeDef* handler, uint8_t start_reg, uint8_t data_buffer[8]){
	HAL_I2C_Mem_Write(handler, (ADDR << 1), start_reg, I2C_MEMADD_SIZE_8BIT, data_buffer, 8, 1000);
}

void ISSI_init(I2C_HandleTypeDef* handler){
	// Select Function Reg
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_FUNCTIONREG);

	// Shutdown Screen
	ISSI_write_reg(handler, SHUTDOWN, 0x00);

	// Picture Mode
	ISSI_write_reg(handler, MODE_SELECT, 0x00);

	// Set Fade in / out times
	ISSI_write_reg(handler, BREATH_CONTROL_1, (FADE_OUT_TIME << 4) + FADE_IN_TIME);

	// Enable Breath and set Extinguish time
	ISSI_write_reg(handler, BREATH_CONTROL_2, (1 << 4) + EXTINGUISH_TIME);

	// Disp frame 0
	ISSI_write_reg(handler, FRAME_SELECT, 0x00);

	// Shut down audio sync
	ISSI_write_reg(handler, AUDIO_SYNC, 0x00);

	// Enable Blink
	ISSI_write_reg(handler, DISPLAY_OPTION, BLINK_PERIOD + 0x08); // + 0x08 to turn on

	// Select bank 1
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_1);

	// Turn off all LEDs
	for (uint8_t i = 0; i <= 10; i++) {
		ISSI_write_reg(handler, rows[i], 0x00);
	}

	// Turn off all Blinking
	for (uint8_t i = 0; i <= 10; i++) {
		ISSI_write_reg(handler, rows[i] + 0x12, 0x00);
	}

	// Set all PWM to INIT_PWM
	for (uint8_t i = 0; i <= 10; i++) {
		uint8_t data_buffer[8] = {INIT_PWM, INIT_PWM, INIT_PWM, INIT_PWM, INIT_PWM, INIT_PWM, INIT_PWM, INIT_PWM};
		ISSI_write_PWM_reg(handler, 0x24 + rows[i] * 0x08, data_buffer);
	}

	// Select Function Reg
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_FUNCTIONREG);

	// Disable Shutdown
	ISSI_write_reg(handler, SHUTDOWN, 0x01);
}

void ISSI_send_buffer(I2C_HandleTypeDef* handler, uint8_t frameBuffer[11]) {
	// Select bank 1
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_1);

	// Send buffer
	for (uint8_t i = 0; i <= 10; i++) {
		ISSI_write_reg(handler, rows[i], frameBuffer[i]);
	}
}

void ISSI_send_buffer_PWM(I2C_HandleTypeDef* handler, uint8_t PWMBuffer[11][8]){
	// Select bank 1
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_1);

	// Send buffer for a whole row
	for (uint8_t i = 0; i <= 10; i++) {
		ISSI_write_PWM_reg(handler, 0x24 + rows[i] * 0x08, PWMBuffer[i]);
	}
}

void ISSI_send_buffer_BLINK(I2C_HandleTypeDef* handler, uint8_t blinkBuffer[11]){
	// Select bank 1
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_1);

	// Send buffer
	for (uint8_t i = 0; i <= 10; i++) {
		ISSI_write_reg(handler, rows[i] + 0x12, blinkBuffer[i]);
	}
}

void ISSI_clear_screen(I2C_HandleTypeDef* handler) {
	// Select bank 1
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_1);

	// Turn off all LEDs
	for (uint8_t i = 0; i <= 10; i++) {
		ISSI_write_reg(handler, rows[i], 0x00);
	}
}

void ISSI_breath_control(I2C_HandleTypeDef* handler, uint8_t enable){
	// Select Function Reg
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_FUNCTIONREG);

	ISSI_write_reg(handler, BREATH_CONTROL_2, (enable << 4) + EXTINGUISH_TIME);
}

void ISSI_set_frame(I2C_HandleTypeDef* handler, uint8_t frame){
	// Select Function Reg
	ISSI_write_reg(handler, COMMAND_REGISTER, BANK_FUNCTIONREG);

	ISSI_write_reg(handler, FRAME_SELECT, frame);
}
