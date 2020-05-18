/*
 * AccelGyro.h
 *
 *  Created on: May 17, 2020
 *      Author: atmat
 */
#include "TJ_MPU6050.h"


static TIM_HandleTypeDef Timer;
int MiliSec;
MPU_ConfigTypeDef MpuConfig;
RawData_Def AccelData, GyroData;
short RawData[6];
short Acceleration[6];
short LastAcceleration[6];
long long Velocity[6];
long long LastVelocity[6];
long long Distance[6];
short Callibration[6];

void Init(I2C_HandleTypeDef *I2Chnd, TIM_HandleTypeDef *htim);
void Update(void);
void Callibrate(int count);
void GetAcceleration(char out[], int start);
void GetVelocity(char out[], int start);
void GetDistance(char out[], int start);


