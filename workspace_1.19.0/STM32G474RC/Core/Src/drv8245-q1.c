/*
* drv8245-q1.c
*
*  Created on: Jan 28, 2026
*      Author: caseyear
*/

#include "drv8245-q1.h"
#include <stdio.h>

static drv8245_handle_t *_drv8245_handle;

// Register cache for read-modify-write operations
static uint8_t reg_cache[6] = {0};  // COMMAND through CONFIG4

void drv8245_assign(SPI_HandleTypeDef *hspi_device) {
    if (_drv8245_handle == NULL) {
        printf("ERROR: drv8245_handle not initialized\r\n");
        return;
    }
    _drv8245_handle->hspi = hspi_device;
}

void drv8245_readData(uint8_t *data) {
    // Pull CS low
    HAL_GPIO_WritePin(_drv8245_handle->csPort, _drv8245_handle->csPin, GPIO_PIN_RESET);

    // SPI read: send address with read bit (bit 14 = 1)
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    tx_data[0] = 0x40 | (data[0] & 0x3F);  // Bit 14=1 (read), bits 13-8=address
    tx_data[1] = 0x00;                      // Dummy byte

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(_drv8245_handle->hspi,
                                                        tx_data, rx_data, 2, 100);

    // Pull CS high
    HAL_GPIO_WritePin(_drv8245_handle->csPort, _drv8245_handle->csPin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        printf("DRV8245 SPI Read Failed: %d\r\n", status);
        return;
    }

    data[1] = rx_data[1];  // Return data byte
}

void drv8245_writeData(uint8_t *data) {
    // Pull CS low
    HAL_GPIO_WritePin(_drv8245_handle->csPort, _drv8245_handle->csPin, GPIO_PIN_RESET);

    // SPI write: send address with write bit (bit 14 = 0)
    uint8_t tx_data[2];
    tx_data[0] = data[0] & 0x3F;  // Bit 14=0 (write), bits 13-8=address
    tx_data[1] = data[1];          // Data byte

    HAL_StatusTypeDef status = HAL_SPI_Transmit(_drv8245_handle->hspi, tx_data, 2, 100);

    // Pull CS high
    HAL_GPIO_WritePin(_drv8245_handle->csPort, _drv8245_handle->csPin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        printf("DRV8245 SPI Write Failed: %d\r\n", status);
    }
}

void drv8245_updateData() {
    // Helper function to update register cache
    uint8_t data[2];

    // Read COMMAND through CONFIG4 registers
    for (uint8_t i = 0; i < 6; i++) {
        data[0] = COMMAND + i;
        drv8245_readData(data);
        reg_cache[i] = data[1];
    }
}

void drv8245_init() {
    if (_drv8245_handle == NULL) {
        printf("ERROR: DRV8245 handle not assigned\r\n");
        return;
    }

    // Set CS high initially
    HAL_GPIO_WritePin(_drv8245_handle->csPort, _drv8245_handle->csPin, GPIO_PIN_SET);

    // SPI (P) variant: VDD supply must be enabled externally
    // Wait for VDD to stabilize and device to complete power-up sequence
    printf("Waiting for DRV8245 power-up (VDD must be > 3.8V)...\r\n");
    HAL_Delay(10);  // Allow VDD to stabilize

    // Wait for tCOM (400µs typ) - device ready for communication
    HAL_Delay(2);   // 2ms to be safe

    // Device should assert nFAULT low, then we send CLR_FLT command
    printf("Sending CLR_FLT to acknowledge power-up...\r\n");
    drv8245_clearFaults();

    // Wait for device to enter STANDBY state
    HAL_Delay(2);

    // Read device ID
    uint8_t data[2];
    data[0] = DEVICE_ID;
    drv8245_readData(data);
    printf("DRV8245 Device ID: 0x%02X\r\n", data[1]);

    // Default configuration
    // CONFIG1: Enable OLA detection, OCP retry
    data[0] = CONFIG1;
    data[1] = CONFIG1_EN_OLA | CONFIG1_OCP_RETRY;
    drv8245_writeData(data);

    // CONFIG2: Default PWM extend, ITRIP at 1.0A
    data[0] = CONFIG2;
    data[1] = CONFIG2_PWM_EXTEND | (0x03);  // S_ITRIP = 011 (1.0A)
    drv8245_writeData(data);

    // CONFIG3: Default slew rate, PH/EN mode
    data[0] = CONFIG3;
    data[1] = (0x02 << 2) | 0x01;  // Medium slew rate, PH/EN mode
    drv8245_writeData(data);

    // CONFIG4: Default OCP settings
    data[0] = CONFIG4;
    data[1] = 0x00;
    drv8245_writeData(data);

    // Update cache
    drv8245_updateData();

    printf("DRV8245 Initialized\r\n");
}

void drv8245_setSlewRate(uint8_t slew_rate) {
    // slew_rate: 0-7 (0=slowest, 7=fastest)
    uint8_t data[2];
    data[0] = CONFIG3;
    drv8245_readData(data);

    data[1] = (data[1] & ~CONFIG3_S_SR) | ((slew_rate & 0x07) << 2);
    drv8245_writeData(data);

    printf("DRV8245 Slew Rate Set: %d\r\n", slew_rate);
}

void drv8245_setOCR(uint8_t ocr_level) {
    // ocr_level: 0-3 (overcurrent threshold)
    uint8_t data[2];
    data[0] = CONFIG4;
    drv8245_readData(data);

    data[1] = (data[1] & ~CONFIG4_OCP_SEL) | ((ocr_level & 0x03) << 3);
    drv8245_writeData(data);

    printf("DRV8245 OCR Level Set: %d\r\n", ocr_level);
}

void drv8245_setBridgeMode(uint8_t mode) {
    // mode: 0=PH/EN, 1=PWM, 2=Independent
    uint8_t data[2];
    data[0] = CONFIG3;
    drv8245_readData(data);

    data[1] = (data[1] & ~CONFIG3_S_MODE) | (mode & 0x03);
    drv8245_writeData(data);

    printf("DRV8245 Bridge Mode Set: %d\r\n", mode);
}

void drv8245_reset() {
    // SPI (P) variant: No nSLEEP pin
    // Reset by cycling VDD power externally or sending software reset command
    printf("DRV8245 (P) variant: Reset via VDD power cycle required\r\n");
    printf("Software workaround: Clearing all faults and reconfiguring...\r\n");

    drv8245_clearFaults();
    HAL_Delay(5);

    // Re-initialize
    drv8245_init();
}

void drv8245_setSpiControl(uint8_t enable) {
    uint8_t data[2];

    // First unlock SPI_IN register
    data[0] = COMMAND;
    drv8245_readData(data);
    data[1] = (data[1] & 0xFC) | 0x02;  // SPI_IN_LOCK = 10b (unlock)
    drv8245_writeData(data);

    // Now write to SPI_IN
    data[0] = SPI_IN;
    drv8245_readData(data);

    if (enable) {
        data[1] |= SPI_IN_S_EN_IN1;
    } else {
        data[1] &= ~SPI_IN_S_EN_IN1;
    }

    drv8245_writeData(data);
}

void drv8245_setDriveOutput(uint8_t enable, uint8_t direction) {
    uint8_t data[2];

    // First unlock SPI_IN register
    data[0] = COMMAND;
    drv8245_readData(data);
    data[1] = (data[1] & 0xFC) | 0x02;  // SPI_IN_LOCK = 10b (unlock)
    drv8245_writeData(data);

    // Now write to SPI_IN
    data[0] = SPI_IN;
    drv8245_readData(data);

    // Clear relevant bits
    data[1] &= ~(SPI_IN_S_DRVOFF | SPI_IN_S_EN_IN1 | SPI_IN_S_PH_IN2);

    if (enable) {
        data[1] |= SPI_IN_S_EN_IN1;  // Enable output

        if (direction) {
            data[1] |= SPI_IN_S_PH_IN2;  // Forward
        }
        // else: direction bit stays 0 for reverse
    } else {
        data[1] |= SPI_IN_S_DRVOFF;  // Disable output
    }

    drv8245_writeData(data);
}

void drv8245_lockReg() {
    uint8_t data[2];
    data[0] = COMMAND;
    drv8245_readData(data);

    data[1] = (data[1] & 0xFC) | 0x03;  // REG_LOCK = 11b (lock)
    drv8245_writeData(data);

    printf("DRV8245 Registers Locked\r\n");
}

void drv8245_unlockReg() {
    uint8_t data[2];
    data[0] = COMMAND;
    drv8245_readData(data);

    data[1] = (data[1] & 0xFC) | 0x01;  // REG_LOCK = 01b (unlock)
    drv8245_writeData(data);

    printf("DRV8245 Registers Unlocked\r\n");
}

void drv8245_getFaultSummary() {
    uint8_t data[2];

    // Read FAULT_SUMMARY
    data[0] = FAULT_SUMMARY;
    drv8245_readData(data);

    printf("\r\nDRV8245 Fault Summary: 0x%02X\r\n", data[1]);
    if (data[1] & FAULT_SUMMARY_SPI_ERR) printf("  - SPI Error\r\n");
    if (data[1] & FAULT_SUMMARY_POR)     printf("  - Power-On Reset\r\n");
    if (data[1] & FAULT_SUMMARY_FAULT)   printf("  - Fault Condition\r\n");
    if (data[1] & FAULT_SUMMARY_VMOV)    printf("  - VM Overvoltage\r\n");
    if (data[1] & FAULT_SUMMARY_VMUV)    printf("  - VM Undervoltage\r\n");
    if (data[1] & FAULT_SUMMARY_OCP)     printf("  - Overcurrent\r\n");
    if (data[1] & FAULT_SUMMARY_TSD)     printf("  - Thermal Shutdown\r\n");
    if (data[1] & FAULT_SUMMARY_OLA)     printf("  - Open Load\r\n");

    // Read STATUS1
    data[0] = STATUS1;
    drv8245_readData(data);
    printf("STATUS1: 0x%02X\r\n", data[1]);

    // Read STATUS2
    data[0] = STATUS2;
    drv8245_readData(data);
    printf("STATUS2: 0x%02X\r\n", data[1]);
}

void drv8245_clearFaults() {
    uint8_t data[2];
    data[0] = COMMAND;
    data[1] = 0x80;  // CLR_FLT bit (bit 7)
    drv8245_writeData(data);

    HAL_Delay(1);

    printf("DRV8245 Faults Cleared\r\n");
}
