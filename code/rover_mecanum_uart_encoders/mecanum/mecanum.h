/*
 * mecanum.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Tommaso
 */

#ifndef INC_MECANUM_H_
#define INC_MECANUM_H_

#include <stm32f4xx_hal.h>

typedef struct {
	GPIO_TypeDef *dir_pin_1_port;
	uint16_t dir_pin_1;
	GPIO_TypeDef *dir_pin_2_port;
	uint16_t dir_pin_2;
	TIM_HandleTypeDef *timer;
	uint16_t channel;
	uint32_t timer_reload;
} motor_t;

/**
 * @brief Update the timer_reload value to the current one
 * @retval None
 */
void motor_init(motor_t* motor);

/**
 * @brief Stops the motor
 * @retval None
 */
void motor_stop(motor_t* motor);

/**
 * @brief Runs the motor at a power between -1.0 and 1.0
 * @param power: power to run the motor at
 * @retval None
 */
void motor_run(motor_t* motor, float power);


typedef float encoder_speed_t;

typedef struct {
	TIM_HandleTypeDef *timer;
	int32_t half_auto_reload;
	uint32_t last_time;
	encoder_speed_t speed;
} encoder_t;

void encoder_init(encoder_t* encoder, uint32_t time);

void encoder_callback(encoder_t* encoder, uint32_t time);

encoder_speed_t encoder_get_speed(encoder_t* encoder);


typedef struct {
	motor_t* fl_motor;
	motor_t* fr_motor;
	motor_t* bl_motor;
	motor_t* br_motor;

	encoder_t* fl_encoder;
	encoder_t* fr_encoder;
	encoder_t* bl_encoder;
	encoder_t* br_encoder;
} four_wheeled_robot_t;

typedef struct {
	encoder_speed_t fl_speed;
	encoder_speed_t fr_speed;
	encoder_speed_t bl_speed;
	encoder_speed_t br_speed;
} four_wheeled_robot_encoders_speeds_t;

void mecanum_robot_init(four_wheeled_robot_t *mecanum_robot, uint32_t time);

void mecanum_robot_stop(four_wheeled_robot_t *mecanum_robot);

/**
 * @brief Move the mecanum robot at given parameters
 */
void mecanum_robot_move(four_wheeled_robot_t *mecanum_robot, float power, float angle, float angular_speed);

void mecanum_robot_encoders_callback(four_wheeled_robot_t *mecanum_robot, uint32_t time);

void mecanum_robot_get_encoder_speeds(four_wheeled_robot_t *mecanum_robot, four_wheeled_robot_encoders_speeds_t* encoders_speeds);

#endif /* INC_MECANUM_H_ */
