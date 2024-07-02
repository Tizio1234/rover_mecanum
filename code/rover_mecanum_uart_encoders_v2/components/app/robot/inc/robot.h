/*
 * robot.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Tommaso
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <stdbool.h>

#include <tim.h>

typedef struct {
	GPIO_TypeDef *dir_pin_1_port;
	uint16_t dir_pin_1;
	GPIO_TypeDef *dir_pin_2_port;
	uint16_t dir_pin_2;
	TIM_HandleTypeDef *timer;
	uint16_t channel;
} motor_config_t;

typedef struct {
	bool initialized;
	bool running;
} motor_state_t;

typedef struct {
	motor_config_t config_;
	motor_state_t state_;
	float autoreload_;
} motor_t;

typedef enum _motor_err {
	MOTOR_ERR_OK = 0,
	MOTOR_ERR_FAIL,
	MOTOR_ERR_INVALID_STATE,
	MOTOR_ERR_INVALID_ARG
} motor_err_t;

/**
 * @brief Initializes the motor
 */
motor_err_t motor_init(motor_t *motor, const motor_config_t *config);

/**
 * @brief Deinitializes the motor
 */
motor_err_t motor_deinit(motor_t *motor);

motor_err_t motor_update_autoreload(motor_t *motor);

/**
 * @brief Stops the motor, if motor is already stopped, it will return ok anyway
 */
motor_err_t motor_stop(motor_t *motor);

/**
 * @brief Runs the motor at a power between -1.0 and 1.0
 * @param power: power to run the motor at, value must be between -1.0 and 1.0 included
 * @retval None
 */
motor_err_t motor_run(motor_t *motor, float power);

/**
 * @brief Get current motor state
 */
motor_err_t motor_get_state(motor_t *motor, motor_state_t *state);

typedef struct {
	bool initialized;
	bool running;
} robot_state_t;

typedef struct {
	motor_t *fl_motor_;
	motor_t *fr_motor_;
	motor_t *bl_motor_;
	motor_t *br_motor_;

	robot_state_t state_;
} frwd_robot_t;

typedef enum {
	ROBOT_ERR_OK = 0,
	ROBOT_ERR_FAIL,
	ROBOT_ERR_INVALID_STATE,
	ROBOT_ERR_INVALID_ARG,
	ROBOT_ERR_MOTOR
} robot_err_t;

/**
 * @brief Init a frwd robot, motors must be already initialized
 * @param mecanum_robot pointer to fwrd robot instance
 * @param fl_motor pointer to fl motor instance
 * @param fr_motor pointer to fr motor instance
 * @param bl_motor pointer to bl motor instance
 * @param br_motor pointer to br motor instance
 */
robot_err_t frwd_robot_init(frwd_robot_t *robot, motor_t *fl_motor,
			    motor_t *fr_motor, motor_t *bl_motor,
			    motor_t *br_motor);

/**
 * @brief Deinit a fwrd robot
 * @param mecanum_robot pointer to fwrd robot instance
 */
robot_err_t frwd_robot_deinit(frwd_robot_t *robot);

/**
 * @brief Stop the fwrd motor
 * @param mecanum_robot pointer to fwrd robot instance
 */
robot_err_t frwd_robot_stop(frwd_robot_t *robot);

/**
 * @brief Move a fwrd robot in a mecanum way
 * @paragraph This function takes a vector and an angular speed
 * @param mecanum_robot pointer to fwrd robot instance
 * @param power r of the vector, must be between 0.0 and 1.0
 * @param theta theta of the vector in radians
 * @param turn angular speed, a good value should be between -0.2 and +0.2
 */
robot_err_t frwd_robot_mecanum_move(frwd_robot_t *robot, float power,
				    float theta, float turn);

#endif /* ROBOT_H_ */
