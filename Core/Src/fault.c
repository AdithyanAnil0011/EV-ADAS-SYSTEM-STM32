/*
 * fault.c

 */
#include "fault.h"
#include "ultrasonic.h"

extern TIM_HandleTypeDef htim1;

void Fault_Init(Fault_HandleTypeDef *flt)
{
    flt->flags  = FAULT_NONE;
    flt->active = 0;
}

void Fault_Check(Fault_HandleTypeDef *flt,
                 EV_HandleTypeDef *ev,
                 ADAS_HandleTypeDef *adas)
{
    uint8_t new_flags = FAULT_NONE;   //NO fault detected initially

    if (ev->motor_temp >= EV_MAX_MOTOR_TEMP_C)
        new_flags |= FAULT_OT;           //adding motor temp fault if curr temp>Thres

    if (ev->soc <= EV_FAULT_SOC_PCT)
        new_flags |= FAULT_SOC; //adding battery fault if curr temp > thres

    if (adas->collision_warn == 2 && HCSR04_IsValid(HCSR04_FRONT))  // checks if the collision warn due to some malfunction
        new_flags |= FAULT_COL;

    flt->flags = new_flags;  //storing the fault detected
    // eg: if Motor Temp = 100°C, SOC = 3%, Collision = Critical
    //then new_flags-> FAULT_OT,FAULT_SOC, FAULT_COL


    if (flt->flags != FAULT_NONE) //if fault detected?
    {
        flt->active = 1;
        ev->state    = STATE_FAULT;   // ← add this

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);  //yellow led on
    }
    else
    {
        flt->active = 0;
        if (ev->state == STATE_FAULT)
            ev->state = STATE_PARKED;  // only exit fault, let EV_Update handle rest
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    }
}

void Fault_Clear(Fault_HandleTypeDef *flt, EV_HandleTypeDef *ev)
{
    flt->flags  = FAULT_NONE;
    flt->active = 0;

    ev->state = STATE_PARKED;

    /* Reset EV to safe state */
    ev->speed_kmh   = 0.0f;
    ev->motor_torque = 0.0f;
    ev->motor_temp  = 25.0f;
    ev->soc         = 80.0f;

    /* Turn off fault LED */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}
