#include <app_main.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <usart.h>
#include <gpio.h>
#include <cmsis_os.h>

#include <lwrb/lwrb.h>
#include <lwpkt/lwpkt.h>

#include <cJSON.h>

#include <uart_rb.h>

#include <robot.h>

#define HUART huart6

#define OS_MS_TO_TICKS(x) (x * osKernelGetTickFreq() / 1000)
#define OS_TICKS_TO_MS(x) (x / osKernelGetTickFreq() * 1000)

// motors timers
#define FL_MOTOR_TIM htim1
#define FR_MOTOR_TIM htim1
#define BL_MOTOR_TIM htim1
#define BR_MOTOR_TIM htim1

// motors timers channels
#define FL_MOTOR_TIM_CHAN TIM_CHANNEL_1
#define FR_MOTOR_TIM_CHAN TIM_CHANNEL_2
#define BL_MOTOR_TIM_CHAN TIM_CHANNEL_3
#define BR_MOTOR_TIM_CHAN TIM_CHANNEL_4

// motors gpio config
#define FL_MOTOR_A_GPIO_PORT FL_MOTOR_A_GPIO_Port
#define FL_MOTOR_A_GPIO_PIN FL_MOTOR_A_Pin
#define FL_MOTOR_B_GPIO_PORT FL_MOTOR_B_GPIO_Port
#define FL_MOTOR_B_GPIO_PIN FL_MOTOR_B_Pin

#define FR_MOTOR_A_GPIO_PORT FR_MOTOR_A_GPIO_Port
#define FR_MOTOR_A_GPIO_PIN FR_MOTOR_A_Pin
#define FR_MOTOR_B_GPIO_PORT FR_MOTOR_B_GPIO_Port
#define FR_MOTOR_B_GPIO_PIN FR_MOTOR_B_Pin

#define BL_MOTOR_A_GPIO_PORT BL_MOTOR_A_GPIO_Port
#define BL_MOTOR_A_GPIO_PIN BL_MOTOR_A_Pin
#define BL_MOTOR_B_GPIO_PORT BL_MOTOR_B_GPIO_Port
#define BL_MOTOR_B_GPIO_PIN BL_MOTOR_B_Pin

#define BR_MOTOR_A_GPIO_PORT BR_MOTOR_A_GPIO_Port
#define BR_MOTOR_A_GPIO_PIN BR_MOTOR_A_Pin
#define BR_MOTOR_B_GPIO_PORT BR_MOTOR_B_GPIO_Port
#define BR_MOTOR_B_GPIO_PIN BR_MOTOR_B_Pin

#define CJSON_NUMBER_OR_ZERO(x) ((cJSON_IsNumber(x) ? cJSON_GetNumberValue(x) : 0.0f))

#define ROBOT_SAFETY_STOP_TIMER_PERIOD_MS 350

static const motor_config_t fl_motor_config = {
    .timer = &FL_MOTOR_TIM,
    .channel = FL_MOTOR_TIM_CHAN,
    .dir_pin_1_port = FL_MOTOR_A_GPIO_PORT,
    .dir_pin_1 = FL_MOTOR_A_GPIO_PIN,
    .dir_pin_2_port = FL_MOTOR_B_GPIO_PORT,
    .dir_pin_2 = FL_MOTOR_B_GPIO_PORT};
static const motor_config_t fr_motor_config = {
    .timer = &FR_MOTOR_TIM,
    .channel = FR_MOTOR_TIM_CHAN,
    .dir_pin_1_port = FR_MOTOR_A_GPIO_PORT,
    .dir_pin_1 = FR_MOTOR_A_GPIO_PIN,
    .dir_pin_2_port = FR_MOTOR_B_GPIO_PORT,
    .dir_pin_2 = FR_MOTOR_B_GPIO_PORT};
static const motor_config_t bl_motor_config = {
    .timer = &BL_MOTOR_TIM,
    .channel = BL_MOTOR_TIM_CHAN,
    .dir_pin_1_port = BL_MOTOR_A_GPIO_PORT,
    .dir_pin_1 = BL_MOTOR_A_GPIO_PIN,
    .dir_pin_2_port = BL_MOTOR_B_GPIO_PORT,
    .dir_pin_2 = BL_MOTOR_B_GPIO_PORT};
static const motor_config_t br_motor_config = {
    .timer = &BR_MOTOR_TIM,
    .channel = BR_MOTOR_TIM_CHAN,
    .dir_pin_1_port = BR_MOTOR_A_GPIO_PORT,
    .dir_pin_1 = BR_MOTOR_A_GPIO_PIN,
    .dir_pin_2_port = BR_MOTOR_B_GPIO_PORT,
    .dir_pin_2 = BR_MOTOR_B_GPIO_PORT};

static const cJSON_Hooks hooks = {
    .free_fn = free,
    .malloc_fn = malloc};

static motor_t fl_motor;
static motor_t fr_motor;
static motor_t bl_motor;
static motor_t br_motor;
static frwd_robot_t robot;

static lwpkt_t rover_uart_pkt;

static uint8_t rover_uart_rb_tx_rb_data[256], rover_uart_rb_rx_rb_data[256];
static uart_rb_t rover_uart_rb;

static osSemaphoreId_t robot_sem_id;
static const osSemaphoreAttr_t robot_sem_attr = {
    .name = "robot sem"};

static osTimerId_t robot_safety_stop_timer_id;
static const osTimerAttr_t robot_safety_stop_timer_attr = {
    .name = "robot safety stop timer"};
void robot_safety_stop_timer_fun(void *arg)
{
    osStatus_t status = osSemaphoreAcquire(robot_sem_id, 0);
    if (status == osOK)
    {
        frwd_robot_stop(&robot);
        osSemaphoreRelease(robot_sem_id);
    }
}

static void uart_rx_evt_cb(UART_HandleTypeDef *huart, uint16_t Pos)
{
    if (huart->Instance == HUART.Instance)
    {
        uart_rb_rx_evt_cb(&rover_uart_rb, Pos);
    }
}

static void uart_tx_tc_cb(UART_HandleTypeDef *huart)
{
    if (huart->Instance == HUART.Instance)
    {
        uart_rb_tx_tc_cb(&rover_uart_rb);
    }
}

osThreadId_t app_main_thread_id = NULL;
const osThreadAttr_t app_main_thread_attr = {
    .name = "app main thread",
    .stack_size = 4 * 256,
    .priority = osPriorityNormal};

void app_main_thread_fun(void *param)
{
    UNUSED(param);

    cJSON_InitHooks(&hooks);

    robot_sem_id = osSemaphoreNew(1, 1, &robot_sem_attr);
    robot_safety_stop_timer_id = osTimerNew(robot_safety_stop_timer_fun, osTimerOnce, NULL, &robot_safety_stop_timer_attr);

    osSemaphoreAcquire(robot_sem_id, osWaitForever);
    motor_init(&fl_motor, &fl_motor_config);
    motor_init(&fr_motor, &fr_motor_config);
    motor_init(&bl_motor, &bl_motor_config);
    motor_init(&br_motor, &br_motor_config);
    frwd_robot_init(&robot, &fl_motor, &fr_motor, &bl_motor, &br_motor);
    osSemaphoreRelease(robot_sem_id);

    HAL_UART_RegisterRxEventCallback(&HUART, uart_rx_evt_cb);
    HAL_UART_RegisterCallback(&HUART, HAL_UART_TX_COMPLETE_CB_ID, uart_tx_tc_cb);

    uart_rb_init(&rover_uart_rb, &HUART, rover_uart_rb_rx_rb_data, sizeof(rover_uart_rb_rx_rb_data), rover_uart_rb_tx_rb_data, sizeof(rover_uart_rb_tx_rb_data));
    uart_rb_start(&rover_uart_rb);

    lwpkt_init(&rover_uart_pkt, &rover_uart_rb.tx_rb, &rover_uart_rb.rx_rb);

    while (1)
    {
        uint32_t ticks = osKernelGetTickCount();
        lwpktr_t res = lwpkt_process(&rover_uart_pkt, OS_TICKS_TO_MS(ticks));
        switch (res)
        {
        case lwpktVALID:
            size_t len = lwpkt_get_data_len(&rover_uart_pkt);
            char *data = (char *)lwpkt_get_data(&rover_uart_pkt);

            // printf("Got a valid packet(%.*s)\r\n", len, data);
            printf("Got a valid packet(%u)\r\n", len);

            cJSON *pkt_json = cJSON_ParseWithLength(data, len);
            if (cJSON_IsObject(pkt_json))
            {
                printf("json object\r\n");

                cJSON *power_json = cJSON_GetObjectItem(pkt_json, "p");
                cJSON *theta_json = cJSON_GetObjectItem(pkt_json, "th");
                cJSON *turn_json = cJSON_GetObjectItem(pkt_json, "tu");
                cJSON *stop_json = cJSON_GetObjectItem(pkt_json, "stop");

                if (cJSON_IsTrue(stop_json))
                {
                    osSemaphoreAcquire(robot_sem_id, osWaitForever);
                    frwd_robot_stop(&robot);
                    osSemaphoreRelease(robot_sem_id);
                    printf("stop\r\n");
                }
                else if (cJSON_IsNumber(power_json) || cJSON_IsNumber(theta_json) || cJSON_IsNumber(turn_json))
                {
                    float power = CJSON_NUMBER_OR_ZERO(power_json);
                    float turn = CJSON_NUMBER_OR_ZERO(turn_json);
                    float theta = CJSON_NUMBER_OR_ZERO(theta_json);

                    osSemaphoreAcquire(robot_sem_id, osWaitForever);
                    frwd_robot_mecanum_move(&robot, power, theta, turn);
                    osSemaphoreRelease(robot_sem_id);
                    printf("running\r\n");
                }
                else
                {
                    break;
                }
                osTimerStart(robot_safety_stop_timer_id, OS_MS_TO_TICKS(ROBOT_SAFETY_STOP_TIMER_PERIOD_MS));
            }

            break;
        }
        
        static GPIO_PinState last_state = GPIO_PIN_SET;
        GPIO_PinState current_state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

        if (current_state != last_state){
            osSemaphoreAcquire(robot_sem_id, osWaitForever);
            if (current_state == GPIO_PIN_RESET) frwd_robot_mecanum_move(&robot, 0.5f, M_PI_2, 0.0f);
            else frwd_robot_stop(&robot);
            osSemaphoreRelease(robot_sem_id);
        }

        last_state = current_state;

        osDelayUntil(ticks + OS_MS_TO_TICKS(20));
    }
}

void init_app_main(void)
{
    app_main_thread_id = osThreadNew(app_main_thread_fun, NULL, &app_main_thread_attr);

    printf("Initialized:\r\n");
    printf("app main.\r\n");
}
