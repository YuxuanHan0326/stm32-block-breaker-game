/*
 * screen.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Yuxuan Han
 */

#ifndef SRC_SCREEN_H_
#define SRC_SCREEN_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define ADDR 0x75

#define COMMAND_REGISTER 0xFD

#define BANK_1 0x00
#define BANK_2 0x01
#define BANK_3 0x02
#define BANK_4 0x03
#define BANK_5 0x04
#define BANK_6 0x05
#define BANK_7 0x06
#define BANK_8 0x07
#define BANK_FUNCTIONREG 0x0B

#define MODE_SELECT 0x00
#define FRAME_SELECT 0x01
#define AUTO_PLAY_CONTROL_1 0x02
#define AUTO_PLAY_CONTROL_2 0x03
#define DISPLAY_OPTION 0x05
#define AUDIO_SYNC 0x06
#define BREATH_CONTROL_1 0x08
#define BREATH_CONTROL_2 0x09
#define SHUTDOWN 0x0A
#define AGC_CONTROL 0x0B
#define ADC_RATE 0x0C

#define INIT_PWM 0x04
#define BLINK_PERIOD 0x01
#define FADE_OUT_TIME 2
#define FADE_IN_TIME 2
#define EXTINGUISH_TIME 1

void ISSI_write_reg(I2C_HandleTypeDef* handler, uint8_t reg, uint8_t data);

void ISSI_init(I2C_HandleTypeDef* handler);

void ISSI_send_buffer(I2C_HandleTypeDef* handler, uint8_t frameBuffer[11]);

void ISSI_send_buffer_PWM(I2C_HandleTypeDef* handler, uint8_t frameBuffer[11][8]);

void ISSI_send_buffer_BLINK(I2C_HandleTypeDef* handler, uint8_t blinkBuffer[11]);

void ISSI_write_PWM_reg(I2C_HandleTypeDef* handler, uint8_t start_reg, uint8_t data_buffer[8]);

void ISSI_clear_screen(I2C_HandleTypeDef* handler);

void ISSI_breath_control(I2C_HandleTypeDef* handler, uint8_t enable);

void ISSI_set_frame(I2C_HandleTypeDef* handler, uint8_t frame);

#endif /* SRC_SCREEN_H_ */


