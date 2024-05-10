/*
 * mecanum.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Tommaso
 */

#include <mecanum.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

/**
 * @brief Update the timer_reload value to the current one
 * @retval None
 */
void motor_init(motor_t* motor){
	motor->timer_reload = __HAL_TIM_GET_AUTORELOAD(motor->timer);
}

/**
 * @brief Stops the motor
 * @retval None
 */
void motor_stop(motor_t* motor){
	HAL_GPIO_WritePin(motor->dir_pin_1_port, motor->dir_pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->dir_pin_2_port, motor->dir_pin_2, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(motor->timer, motor->channel);
}

/**
 * @brief Runs the motor at a power between -1.0 and 1.0
 * @param power: power to run the motor at
 * @retval None
 */
void motor_run(motor_t* motor, float power){
	if (power){
		power = fminf(fmaxf(power, -1.0f), 1.0f);
		bool direction = power > 0.0;
		HAL_GPIO_WritePin(motor->dir_pin_1_port, motor->dir_pin_1, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->dir_pin_2_port, motor->dir_pin_2, !direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->channel, fabsf(power) * (float)motor->timer_reload);
		HAL_TIM_PWM_Start(motor->timer, motor->channel);
	} else {
		motor_stop(motor);
	}
}


void encoder_init(encoder_t* encoder, uint32_t time){
	encoder->last_time = time;
	encoder->half_auto_reload = (int32_t)__HAL_TIM_GET_AUTORELOAD(encoder->timer) / 2;
	HAL_TIM_Encoder_Start(encoder->timer, TIM_CHANNEL_ALL);
}

void encoder_callback(encoder_t* encoder, uint32_t time){
	uint32_t d_t = time - encoder->last_time;
	int32_t d_p = (int32_t)__HAL_TIM_GET_COUNTER(encoder->timer) - encoder->half_auto_reload;
	encoder->speed = (encoder_speed_t)d_p / (encoder_speed_t)d_t;
	encoder->last_time = time;
	__HAL_TIM_SET_COUNTER(encoder->timer, (uint32_t)encoder->half_auto_reload);

	//printf("d_p: %li, d_t: %lu\r\n", d_p, d_t);
}

encoder_speed_t encoder_get_speed(encoder_t* encoder){
	return encoder->speed;
}


void mecanum_robot_init(four_wheeled_robot_t *mecanum_robot, uint32_t time){
	motor_init(mecanum_robot->fl_motor);
	motor_init(mecanum_robot->fr_motor);
	motor_init(mecanum_robot->bl_motor);
	motor_init(mecanum_robot->br_motor);

	encoder_init(mecanum_robot->fl_encoder, time);
	encoder_init(mecanum_robot->fr_encoder, time);
	encoder_init(mecanum_robot->bl_encoder, time);
	encoder_init(mecanum_robot->br_encoder, time);
}

void mecanum_robot_stop(four_wheeled_robot_t *mecanum_robot){
	motor_stop(mecanum_robot->fl_motor);
	motor_stop(mecanum_robot->fr_motor);
	motor_stop(mecanum_robot->bl_motor);
	motor_stop(mecanum_robot->br_motor);
}
/**
 * @brief Move the mecanum robot at given parameters
 */
void mecanum_robot_move(four_wheeled_robot_t *mecanum_robot, float power, float angle, float angular_speed){
	if (power == 0.0 && angular_speed == 0.0){
		mecanum_robot_stop(mecanum_robot);
		return;
	}

	power = fminf(fmaxf(power, 0.0f), 1.0f);


	float angle_offset = angle - M_PI_4;
	float sine = sin(angle_offset);
	float cosine = cos(angle_offset);
	float maximum = fmaxf(fabsf(sine), fabsf(cosine));

	float fl = power * cosine/maximum + angular_speed;
	float fr = power * sine/maximum - angular_speed;
	float bl = power * sine/maximum + angular_speed;
	float br = power * cosine/maximum - angular_speed;

	if ((power + fabsf(angular_speed)) > 1) {
		float k = power + angular_speed;
		fl /= k;
		fr /= k;
		bl /= k;
		br /= k;
	}

	motor_run(mecanum_robot->fl_motor, fl);
	motor_run(mecanum_robot->fr_motor, fr);
	motor_run(mecanum_robot->bl_motor, bl);
	motor_run(mecanum_robot->br_motor, br);
}

void mecanum_robot_encoders_callback(four_wheeled_robot_t *mecanum_robot, uint32_t time){
	encoder_callback(mecanum_robot->fl_encoder, time);
	encoder_callback(mecanum_robot->fr_encoder, time);
	encoder_callback(mecanum_robot->bl_encoder, time);
	encoder_callback(mecanum_robot->br_encoder, time);
}

void mecanum_robot_get_encoder_speeds(four_wheeled_robot_t *mecanum_robot, four_wheeled_robot_encoders_speeds_t* encoders_speeds){
	encoders_speeds->fl_speed = encoder_get_speed(mecanum_robot->fl_encoder);
	encoders_speeds->fr_speed = encoder_get_speed(mecanum_robot->fr_encoder);
	encoders_speeds->bl_speed = encoder_get_speed(mecanum_robot->bl_encoder);
	encoders_speeds->br_speed = encoder_get_speed(mecanum_robot->br_encoder);
}
