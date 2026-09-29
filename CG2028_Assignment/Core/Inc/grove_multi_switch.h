#ifndef GROVE_MULTI_SWITCH_H
#define GROVE_MULTI_SWITCH_H

#include "stm32l4xx_hal.h"

/* The module uses 7-bit address 0x03. STM32 HAL addresses are shifted left. */
#define GROVE_MULTI_SWITCH_DEFAULT_ADDRESS (0x03U << 1)

#define GROVE_MULTI_SWITCH_VENDOR_ID       0x2886U
#define GROVE_MULTI_SWITCH_PID_5_WAY       0x0002U
#define GROVE_MULTI_SWITCH_PID_6_POS_DIP   0x0003U

#define GROVE_MULTI_SWITCH_BUTTON_MAX      6U

/* Values returned in the 32-bit event field. */
#define GROVE_MULTI_SWITCH_EVENT_NONE      0x00000000UL
#define GROVE_MULTI_SWITCH_EVENT_PRESENT   0x80000000UL

/* Values returned in each button byte. */
#define GROVE_MULTI_SWITCH_RAW_STATUS      (1U << 0)
#define GROVE_MULTI_SWITCH_SINGLE_CLICK    (1U << 1)
#define GROVE_MULTI_SWITCH_DOUBLE_CLICK    (1U << 2)
#define GROVE_MULTI_SWITCH_LONG_PRESS      (1U << 3)
#define GROVE_MULTI_SWITCH_LEVEL_CHANGED   (1U << 4)

typedef struct
{
    uint32_t event;
    uint8_t button[GROVE_MULTI_SWITCH_BUTTON_MAX];
} GroveMultiSwitch_EventTypeDef;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    uint32_t device_id;
    uint8_t button_count;
    uint8_t firmware_version;
    char firmware_string[11];
    uint8_t previous_button[GROVE_MULTI_SWITCH_BUTTON_MAX];
    uint8_t previous_button_valid;
} GroveMultiSwitch_HandleTypeDef;

HAL_StatusTypeDef GroveMultiSwitch_Init(GroveMultiSwitch_HandleTypeDef *device,
                                        I2C_HandleTypeDef *hi2c,
                                        uint16_t address);

HAL_StatusTypeDef GroveMultiSwitch_ReadEvent(
    GroveMultiSwitch_HandleTypeDef *device,
    GroveMultiSwitch_EventTypeDef *event);

HAL_StatusTypeDef GroveMultiSwitch_SetEventMode(
    GroveMultiSwitch_HandleTypeDef *device,
    uint8_t enable);

HAL_StatusTypeDef GroveMultiSwitch_ReadDeviceID(
    GroveMultiSwitch_HandleTypeDef *device,
    uint32_t *device_id);

uint32_t GroveMultiSwitch_GetDeviceID(
    const GroveMultiSwitch_HandleTypeDef *device);

uint8_t GroveMultiSwitch_GetButtonCount(
    const GroveMultiSwitch_HandleTypeDef *device);

const char *GroveMultiSwitch_GetFirmwareString(
    const GroveMultiSwitch_HandleTypeDef *device);

#endif /* GROVE_MULTI_SWITCH_H */
