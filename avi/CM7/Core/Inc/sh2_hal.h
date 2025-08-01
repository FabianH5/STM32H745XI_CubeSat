/*
 * Copyright 2018-2021 CEVA, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with CEVA, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * SH2 HAL Interface
 */

// Include guard
#ifndef SH2_HAL_H
#define SH2_HAL_H

#include <stdint.h>
#include <stm32h7xx.h>

// Maximum SHTP Transfer and Payload sizes
#define SH2_HAL_MAX_TRANSFER_OUT (128)
#define SH2_HAL_MAX_PAYLOAD_OUT  (128)

#define SH2_HAL_MAX_TRANSFER_IN  (1024)
#define SH2_HAL_MAX_PAYLOAD_IN   (1024)

typedef struct{
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
	float i, j, k, r;
}SensorData_t;

extern SensorData_t imuData;

typedef struct {
	I2C_HandleTypeDef *i2cHandle;
	uint8_t i2cAddr;
	GPIO_TypeDef *intnPort;
	uint16_t intnPin;
	GPIO_TypeDef *rstPort;
	uint16_t rstPin;
}IMU_Config;

typedef struct sh2_Hal_s sh2_Hal_t;

// The SH2 interface uses these functions to access the underlying
// communications device. The system designer will need to
// implement these.  At system intialization time, an sh2_Hal_t
// structure should be initialized with pointers to all the hardware
// access layer functions.  A pointer to this structure must then be
// passed to sh2_open() to initialize the SH2 interface.
//
// If the DFU (download firmware update) capability is needed, the
// example DFU code also uses this interface but each function has
// somewhat different requirements.  So a separate instance of an
// sh2_Hal_t structure, pointing to different functions, is
// necessary to support DFU.

struct sh2_Hal_s {
    // This function initializes communications with the device.  It
    // can initialize any GPIO pins and peripheral devices used to
    // interface with the sensor hub.
    // It should also perform a reset cycle on the sensor hub to
    // ensure communications start from a known state.
    int (*open)(sh2_Hal_t *self);

    // This function completes communications with the sensor hub.
    // It should put the device in reset then de-initialize any
    // peripherals or hardware resources that were used.
    void (*close)(sh2_Hal_t *self);

    // This function supports reading data from the sensor hub.
    // It will be called frequently to service the device.
    //
    // If the HAL has received a full SHTP transfer, this function
    // should load the data into pBuffer, set the timestamp to the
    // time the interrupt was detected, and return the non-zero length
    // of data in this transfer.
    //
    // If the HAL has not recevied a full SHTP transfer, this function
    // should return 0.
    //
    // Because this function is called regularly, it can be used to
    // perform other housekeeping operations.  (In the case of UART
    // interfacing, bytes transmitted are staggered in time and this
    // function can be used to keep the transmission flowing.)
    int (*read)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);

    // This function supports writing data to the sensor hub.
    // It is called each time the application has a block of data to
    // transfer to the device.
    //
    // If the device isn't ready to receive data, this function can
    // return 0 without performing the transmit function.
    //
    // If the transmission can be started, this function needs to
    // copy the data from pBuffer and return the number of bytes
    // accepted.  It need not block.  The actual transmission of
    // the data can continue after this function returns.
    int (*write)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

    // This function should return a 32-bit value representing a
    // microsecond counter.  The count may roll over after 2^32
    // microseconds.  
    uint32_t (*getTimeUs)(sh2_Hal_t *self);
};

void SH2_HAL_Setup(IMU_Config *c);

sh2_Hal_t *sh2_hal_get(void);

void IMU_Init(void);

void IMU_Service(void);

void Tare(void);

#endif
