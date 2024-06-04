/*
 * robot.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Tommaso
 */

#include <robot.h>

#include <stdbool.h>
#include <stdio.h>

#include <string.h>

#include <math.h>

#define MOTOR_PARAM_CHECK(p) \
	if (p == NULL)           \
	return MOTOR_ERR_INVALID_ARG

motor_err_t motor_init(motor_t *motor, motor_config_t *config)
{
	MOTOR_PARAM_CHECK(motor);
	MOTOR_PARAM_CHECK(config);

	if (motor->state_.initialized)
		return MOTOR_ERR_INVALID_STATE;

	memcpy(&motor->config_, config, sizeof(motor->config_));

	motor->state_.initialized = true;

	motor_update_autoreload(motor);

	return MOTOR_ERR_OK;
}

motor_err_t motor_deinit(motor_t *motor)
{
	MOTOR_PARAM_CHECK(motor);

	if (!motor->state_.initialized || motor->state_.running)
		return MOTOR_ERR_INVALID_STATE;

	motor->state_.initialized = false;

	return MOTOR_ERR_OK;
}

motor_err_t motor_update_autoreload(motor_t *motor)
{
	MOTOR_PARAM_CHECK(motor);

	if (!motor->state_.initialized)
		return MOTOR_ERR_INVALID_STATE;

	motor->autoreload_ = (float)__HAL_TIM_GET_AUTORELOAD(motor->config_.timer);

	return MOTOR_ERR_OK;
}

motor_err_t motor_stop(motor_t *motor)
{
	MOTOR_PARAM_CHECK(motor);

	if (!motor->state_.initialized)
		return MOTOR_ERR_INVALID_STATE;

	HAL_GPIO_WritePin(motor->config_.dir_pin_1_port, motor->config_.dir_pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->config_.dir_pin_2_port, motor->config_.dir_pin_2, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(motor->config_.timer, motor->config_.channel);

	if (motor->state_.running)
		motor->state_.running = false;

	return MOTOR_ERR_OK;
}

motor_err_t motor_run(motor_t *motor, float power)
{
	MOTOR_PARAM_CHECK(motor);

	if (!motor->state_.initialized)
		return MOTOR_ERR_INVALID_STATE;

	if (power)
	{
		power = fminf(fmaxf(power, -1.0f), 1.0f);
		bool direction = power > 0.0;
		HAL_GPIO_WritePin(motor->config_.dir_pin_1_port, motor->config_.dir_pin_1, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->config_.dir_pin_2_port, motor->config_.dir_pin_2, !direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(motor->config_.timer, motor->config_.channel, fabsf(power) * motor->autoreload_);
		HAL_TIM_PWM_Start(motor->config_.timer, motor->config_.channel);
	}
	else
	{
		motor_stop(motor);
	}

	if (!motor->state_.running)
		motor->state_.running = true;

	return MOTOR_ERR_OK;
}

motor_err_t motor_get_state(motor_t *motor, motor_state_t *state)
{
	MOTOR_PARAM_CHECK(motor);
	MOTOR_PARAM_CHECK(state);

	memcpy(state, &motor->state_, sizeof(*state));

	return MOTOR_ERR_OK;
}


#define ROBOT_PARAM_CHECK(p) if (p == NULL) return ROBOT_ERR_INVALID_ARG

#define ROBOT_EQUAL_CHECK(a, b) if (a != b) return ROBOT_ERR_FAIL

frwd_robot_err_t frwd_robot_init(frwd_robot_t *robot, motor_t *fl_motor, motor_t *fr_motor, motor_t *bl_motor, motor_t *br_motor)
{
	ROBOT_PARAM_CHECK(robot);
	ROBOT_PARAM_CHECK(fl_motor);
	ROBOT_PARAM_CHECK(fr_motor);
	ROBOT_PARAM_CHECK(bl_motor);
	ROBOT_PARAM_CHECK(br_motor);

	robot->fl_motor_ = fl_motor;
	robot->fr_motor_ = fr_motor;
	robot->bl_motor_ = bl_motor;
	robot->br_motor_ = br_motor;

	return ROBOT_ERR_OK;
}

frwd_robot_err_t frwd_robot_deinit(frwd_robot_t *robot)
{
	return ROBOT_ERR_OK;
}

frwd_robot_err_t frwd_robot_stop(frwd_robot_t *robot)
{
	return ROBOT_ERR_OK;
}

frwd_robot_err_t frwd_robot_mecanum_move(frwd_robot_t *robot, float power, float theta, float angular_speed)
{
	return ROBOT_ERR_OK;
}
