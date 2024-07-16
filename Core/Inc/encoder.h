#ifndef __ENCODER_H
#define __ENCODER_H

/* defines */
#define X_ENCODER_TIM_HANDLE htim1
#define Y_ENCODER_TIM_HANDLE htim3
#define Z_ENCODER_TIM_HANDLE htim4

/* external variables */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* MACROs */
#define ENCODER_PARAM_ARRAY_INIT {{&X_ENCODER_TIM_HANDLE, 0, 0, 0}, \
                                  {&Y_ENCODER_TIM_HANDLE, 0, 0, 0}, \
                                  {&Z_ENCODER_TIM_HANDLE, 0, 0, 0},}

/* type define */
typedef float encoder_degree_t[NUM_DIMENSIONS];
typedef float encoder_position_t[NUM_DIMENSIONS];

/* function prototypes */
void encoderInit();
void encoderReadPositionTask(void *pvParameters);
void encoderInterruptHandler();
void encoderResetCounter(axis_t axis);
void encoderReadDegree(encoder_degree_t *degree);
void encoderReadInstantDegree(encoder_degree_t *degree);

/**
 * TODO
 */
// float encoderReadRPM();
// uint32_t encoderGetCounter(axis_t axis);

#endif // __ENCODER_H