#include "sh2_hal.h"
#include "sh2_util.h"
#include "sh2_SensorValue.h"
#include "main.h"

// --- HAL instance ---
static sh2_Hal_t hal;
static IMU_Config imuC;
SensorData_t imuData = {0};

static void sensorHandler(void *cookie, sh2_SensorEvent_t *event);

void SH2_HAL_Setup(IMU_Config *c){
	imuC = *c;
}

// --- Time function (approximate, from HAL tick) ---
static uint32_t getTimeUs(sh2_Hal_t *self) {
    return HAL_GetTick() * 1000;  // crude ms→us conversion
}

// --- Open: init/reset the sensor ---
static int open(sh2_Hal_t *self) {
    // Reset BNO08X
    HAL_GPIO_WritePin(imuC.rstPort, imuC.rstPin, GPIO_PIN_RESET);
    HAL_Delay(10);  // hold reset
    HAL_GPIO_WritePin(imuC.rstPort, imuC.rstPin, GPIO_PIN_SET);
    HAL_Delay(50);  // allow time to boot

    return 0;
}

// --- Close: deinit/reset ---
static void close(sh2_Hal_t *self) {
    HAL_GPIO_WritePin(imuC.rstPort, imuC.rstPin, GPIO_PIN_RESET);
}

// --- Read: poll INTN and read I2C data if available ---
static int read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    if (HAL_GPIO_ReadPin(imuC.intnPort, imuC.intnPin) == GPIO_PIN_RESET) {
        if (HAL_I2C_Master_Receive(imuC.i2cHandle, imuC.i2cAddr, pBuffer, len, HAL_MAX_DELAY) == HAL_OK) {
            if (t_us) *t_us = getTimeUs(self);
            return len;
        }
    }
    return 0;
}

// --- Write: send data to BNO08X ---
static int write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (HAL_I2C_Master_Transmit(imuC.i2cHandle, imuC.i2cAddr, pBuffer, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    }
    return 0;
}

// --- Return pointer to HAL interface ---
sh2_Hal_t *sh2_hal_get(void) {
    hal.open = open;
    hal.close = close;
    hal.read = read;
    hal.write = write;
    hal.getTimeUs = getTimeUs;
    return &hal;
}

void IMU_Init(void){
	sh2_open(sh2_hal_get(), NULL, NULL);
	sh2_setSensorCallback(sensorHandler, NULL);
	sh2_SensorConfig_t config = {0};
	config.reportInterval_us = 50000;

	sh2_setSensorConfig(SH2_ACCELEROMETER, &config);
	sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);
	sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &config);
	sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);

}

void IMU_Service(void){
	sh2_service();
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *event){
	sh2_SensorValue_t val;
	sh2_decodeSensorEvent(&val, event);

	switch (event-> reportId){
		case SH2_ACCELEROMETER:
			imuData.ax = val.un.accelerometer.x;
			imuData.ay = val.un.accelerometer.y;
			imuData.az = val.un.accelerometer.z;
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			imuData.gx = val.un.gyroscope.x;
			imuData.gy = val.un.gyroscope.y;
			imuData.gz = val.un.gyroscope.z;
			break;
		case SH2_MAGNETIC_FIELD_CALIBRATED:
			imuData.mx = val.un.magneticField.x;
			imuData.my = val.un.magneticField.y;
			imuData.mz = val.un.magneticField.z;
			break;
		case SH2_ROTATION_VECTOR:
			imuData.i = val.un.rotationVector.i;
			imuData.j = val.un.rotationVector.j;
			imuData.k = val.un.rotationVector.k;
			imuData.r = val.un.rotationVector.real;
			break;
	}
}

void Tare(void){
	sh2_setTareNow(SH2_TARE_X|SH2_TARE_Y|SH2_TARE_Z, SH2_TARE_BASIS_ROTATION_VECTOR);
}


