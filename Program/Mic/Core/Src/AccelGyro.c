/*
 * AccelGyro.cpp
 *
 *  Created on: May 17, 2020
 *      Author: atmat
 */

#include "AccelGyro.h"

void Init(I2C_HandleTypeDef *I2Chnd, TIM_HandleTypeDef *htim) {
	memcpy(&Timer, htim, sizeof(*htim));
	MPU6050_Init(I2Chnd);
	MpuConfig.Accel_Full_Scale = AFS_SEL_2g;
	MpuConfig.ClockSource = Internal_8MHz;
	MpuConfig.CONFIG_DLPF = DLPF_260A_256G_Hz;
	MpuConfig.Gyro_Full_Scale = FS_SEL_500;
	MpuConfig.Sleep_Mode_Bit = 0;
	MPU6050_Config(&MpuConfig);

	for (int a = 0; a < 6; a++){
		LastAcceleration[a] = Callibration[a] = Velocity[a] = LastVelocity[a] = Distance[a] = 0;
	}
	HAL_TIM_Base_Start_IT(&Timer);
}

void Update(){
	// Pobranie danych
	MPU6050_Get_Accel_RawData(&AccelData);	// Najpier trzeba akcelerometr
	MPU6050_Get_Gyro_RawData(&GyroData);		// Potem zyroskop
	MiliSec  = Timer.Instance->CNT;
	Timer.Instance->CNT = 0;
	// Raw data
	RawData[0] = AccelData.x;
	RawData[1] = AccelData.y;
	RawData[2] = AccelData.z;
	RawData[3] = GyroData.x;
	RawData[4] = GyroData.y;
	RawData[5] = GyroData.z;
	for (int a = 0; a < 6; a++){
		// Acceleration
		Acceleration[a] = RawData[a] - Callibration[a];
		// Velocity
		Velocity[a] += (Acceleration[a] + (Acceleration[a] - LastAcceleration[a]) / 2) * MiliSec;
		// Distance
		Distance[a] += (Velocity[a] + (Velocity[a] - LastVelocity[a]) / 2) * MiliSec;
		// Zapamiętanie ostatniej probki
		LastAcceleration[a] = Acceleration[a];
		LastVelocity[a] = Velocity[a];
	}
}

void Callibrate(int count){
	long int TmpCall[6];
    int Count = count;
    for (int a = 0; a < 6; a++){
        TmpCall[a] = LastAcceleration[a] = Velocity[a] = LastVelocity[a] = Distance[a] = 0;
    }
    while (Count > 0) {
    	// Pobranie danych
    	MPU6050_Get_Accel_RawData(&AccelData);	// Najpier trzeba akcelerometr
    	MPU6050_Get_Gyro_RawData(&GyroData);		// Potem zyroskop
    	// Raw data
    	TmpCall[0] += AccelData.x;
    	TmpCall[1] += AccelData.y;
    	TmpCall[2] += AccelData.z;
    	TmpCall[3] += GyroData.x;
    	TmpCall[4] += GyroData.y;
    	TmpCall[5] += GyroData.z;
    	NRF24_DelayMicroSeconds(100);
		Count--;
    }
    for (int a = 0; a < 6; a++){
    	Callibration[a] = TmpCall[a]/count;
    }
    Timer.Instance->CNT = 0;
}

void GetAcceleration(char out[], int start){
	for (int a = 0; a < 6; a++){
		out[a * 2 + start] = Acceleration[a];
		out[a * 2 + 1 + start] = Acceleration[a] >> 8;
	}
}

void GetVelocity(char out[], int start){
	short vel;
	for (int a = 0; a < 6; a++){
		vel = (Velocity[a])/1000000;
		out[a * 2 + start] = vel;
		out[a * 2 + 1 + start] = vel >> 8;
	}
}

void GetDistance(char out[], int start){
	short vel;
	for (int a = 0; a < 6; a++){
		vel = (Distance[a])/1000000000000;
		out[a * 2 + start] = vel;
		out[a * 2 + 1 + start] = vel >> 8;
	}
}
