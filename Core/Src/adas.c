/*
 * adas.c
 */

#include "adas.h"
#include "ultrasonic.h"

/*  Hysteresis helper */
/* Returns 1 if alarm is active, 0 if cleared after hysteresis
       */

// static ->indicates that this func is private to adas.c(ie No need to call this function)
static uint8_t hyst_check(uint8_t *cnt, uint8_t condition)
{  // *cnt -> its a pointer to the hysteresis counter
    if (condition)
    {
        *cnt = ADAS_HYSTERESIS_CNT;
        return 1;          // meaning obstacle is detected
    }

    if (*cnt > 0)
    {
        (*cnt)--;
        return 1;
    }

    return 0;
}
/* ADAS_Init  */
void ADAS_Init(ADAS_HandleTypeDef *adas)
{
    memset(adas, 0, sizeof(*adas));
    adas->front_cm = 400.0f;
    adas->left_cm  = 400.0f;
    adas->right_cm = 400.0f;
    adas->ttc_sec  = 99.9f;//This doesn't mean the collision is exactly 99.9 seconds away.
    //It means effectively no immediate collision
}

/*  ADAS_Update — call after HCSR04_ReadAll()  */
void ADAS_Update(ADAS_HandleTypeDef *adas, EV_HandleTypeDef *ev)
{
    /*S1: Get distances from cache */
    adas->front_cm = HCSR04_GetDistance(HCSR04_FRONT);  //values received from sensors are
    adas->left_cm  = HCSR04_GetDistance(HCSR04_LEFT);   // copied here
    adas->right_cm = HCSR04_GetDistance(HCSR04_RIGHT);

    /* Step 2: TTC = distance(m) / speed(m/s) */
    float speed_ms = ev->speed_kmh / 3.6f;
    float front_m  = adas->front_cm / 100.0f;   // in meter
    if (speed_ms > 0.5f && adas->front_cm < 200.0f) {
        adas->ttc_sec = front_m / speed_ms;
        adas->ttc_sec = CLAMP(adas->ttc_sec, 0.0f, 99.9f);
    } else {
        adas->ttc_sec = 99.9f;   /* no obstacle or stationary → safe */
    }

    /* Step 3: Forward Collision Warning */
    //critical
    uint8_t fcw_crit = (adas->front_cm < ADAS_FCW_CRIT_CM)
                     || (adas->ttc_sec  < ADAS_TTC_CRIT_S);
    //warning
    uint8_t fcw_warn = (adas->front_cm < ADAS_FCW_WARN_CM)
                     || (adas->ttc_sec  < ADAS_TTC_WARN_S);


    // with hysteresis the warning will stay stable,if the val is changing
    //around the threshold, then the led will start flickering
    // to avoid this we use hysteresis
    if (hyst_check(&adas->hyst_fcw_crit, fcw_crit)) {
        adas->hyst_fcw_warn = 0;        // reset warn counter when crit fires
        adas->collision_warn = 2;
    } else if (hyst_check(&adas->hyst_fcw_warn, fcw_warn)) {
        adas->collision_warn = 1;
    } else {
        adas->collision_warn = 0;
    }

    /* Step 4: Blind Spot Detection */
    uint8_t bsd_l = (adas->left_cm  < ADAS_BSD_DIST_CM)
                  && (ev->speed_kmh  > ADAS_BSD_SPEED_KMH);
    uint8_t bsd_r = (adas->right_cm < ADAS_BSD_DIST_CM)
                  && (ev->speed_kmh  > ADAS_BSD_SPEED_KMH);
// to prevent flickering
    adas->blindspot_left  = hyst_check(&adas->hyst_bsd_l, bsd_l);
    adas->blindspot_right = hyst_check(&adas->hyst_bsd_r, bsd_r);

    /* Step 5: Overspeed */
    adas->overspeed = hyst_check(&adas->hyst_over,
                       ev->speed_kmh > ADAS_OVERSPEED_KMH);    //if speed > 120, overspeed becomes true


    /* Step 6: Parking Assist — active below 10 km/h, scores closest obstacle */
    adas->parking_active = (ev->speed_kmh < ADAS_PARK_SPEED_KMH) ? 1 : 0;

    if (adas->parking_active) {
        float closest = adas->front_cm;
        if (adas->left_cm  < closest) closest = adas->left_cm;
        if (adas->right_cm < closest) closest = adas->right_cm;
        closest = CLAMP(closest, HCSR04_MIN_CM, HCSR04_MAX_CM);

        /* 0 = far (400cm), 100 = very close (2cm) */
        float score = 100.0f * (1.0f - (closest - HCSR04_MIN_CM)
                                      / (HCSR04_MAX_CM - HCSR04_MIN_CM));
        adas->parking_score = (uint8_t)CLAMP(score, 0.0f, 100.0f);
    } else {
        adas->parking_score = 0;
    }


    /* Step 7: Overall alarm priority */
    if      (adas->collision_warn == 2) adas->alarm_priority = ALARM_CRITICAL;
    else if (adas->collision_warn == 1) adas->alarm_priority = ALARM_WARNING;
    else if (adas->blindspot_left
          || adas->blindspot_right
          || adas->overspeed)
    	adas->alarm_priority = ALARM_ADVISORY;
    else
    	adas->alarm_priority = ALARM_NONE;

    /* Step 8: Drive LEDs */
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,   /* LED_COLLISION */
//        (adas->collision_warn > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (adas->collision_warn == 2)     //CRITICAL
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    else if (adas->collision_warn == 1)   //WARNING
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    else             //NO WARNING
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,   /* LED_BLINDSPOT_L */
        adas->blindspot_left  ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,  /* LED_BLINDSPOT_R */
        adas->blindspot_right ? GPIO_PIN_SET : GPIO_PIN_RESET);
}





