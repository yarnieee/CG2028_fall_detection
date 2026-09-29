#include "grove_multi_switch.h"

#include <string.h>

#define GROVE_MULTI_SWITCH_CMD_GET_DEVICE_ID  0x00U
#define GROVE_MULTI_SWITCH_CMD_GET_EVENT      0x01U
#define GROVE_MULTI_SWITCH_CMD_EVENT_MODE     0x02U
#define GROVE_MULTI_SWITCH_CMD_BLOCK_MODE     0x03U
#define GROVE_MULTI_SWITCH_CMD_GET_VERSION    0xE2U

#define GROVE_MULTI_SWITCH_TIMEOUT_MS         100U
#define GROVE_MULTI_SWITCH_VERSION_LENGTH     10U

static HAL_StatusTypeDef GroveMultiSwitch_Write(
    GroveMultiSwitch_HandleTypeDef *device,
    const uint8_t *data,
    uint16_t length)
{
    return HAL_I2C_Master_Transmit(device->hi2c,
                                   device->address,
                                   (uint8_t *)data,
                                   length,
                                   GROVE_MULTI_SWITCH_TIMEOUT_MS);
}

static HAL_StatusTypeDef GroveMultiSwitch_Read(
    GroveMultiSwitch_HandleTypeDef *device,
    uint8_t *data,
    uint16_t length)
{
    return HAL_I2C_Master_Receive(device->hi2c,
                                  device->address,
                                  data,
                                  length,
                                  GROVE_MULTI_SWITCH_TIMEOUT_MS);
}

static HAL_StatusTypeDef GroveMultiSwitch_ReadRegister(
    GroveMultiSwitch_HandleTypeDef *device,
    uint8_t command,
    uint8_t *data,
    uint16_t length)
{
    HAL_StatusTypeDef status = GroveMultiSwitch_Write(device, &command, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    return GroveMultiSwitch_Read(device, data, length);
}

static uint32_t GroveMultiSwitch_U32FromLittleEndian(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint16_t GroveMultiSwitch_GetProductID(uint32_t device_id)
{
    return (uint16_t)(device_id & 0xFFFFUL);
}

static uint16_t GroveMultiSwitch_GetVendorID(uint32_t device_id)
{
    return (uint16_t)(device_id >> 16);
}

HAL_StatusTypeDef GroveMultiSwitch_ReadDeviceID(
    GroveMultiSwitch_HandleTypeDef *device,
    uint32_t *device_id)
{
    uint8_t data[4];
    HAL_StatusTypeDef status;

    if (device == NULL || device_id == NULL)
    {
        return HAL_ERROR;
    }

    status = GroveMultiSwitch_ReadRegister(device,
                                           GROVE_MULTI_SWITCH_CMD_GET_DEVICE_ID,
                                           data,
                                           sizeof(data));
    if (status == HAL_OK)
    {
        *device_id = GroveMultiSwitch_U32FromLittleEndian(data);
    }

    return status;
}

static HAL_StatusTypeDef GroveMultiSwitch_ReadFirmwareVersion(
    GroveMultiSwitch_HandleTypeDef *device)
{
    uint8_t data[GROVE_MULTI_SWITCH_VERSION_LENGTH];
    HAL_StatusTypeDef status = GroveMultiSwitch_ReadRegister(
        device,
        GROVE_MULTI_SWITCH_CMD_GET_VERSION,
        data,
        sizeof(data));

    if (status != HAL_OK)
    {
        return status;
    }

    memcpy(device->firmware_string, data, sizeof(data));
    device->firmware_string[sizeof(data)] = '\0';

    /* The library encodes a version such as BN-5E-0.1 at indexes 6 and 8. */
    if (data[6] >= '0' && data[6] <= '9' &&
        data[8] >= '0' && data[8] <= '9')
    {
        device->firmware_version = (uint8_t)((data[6] - '0') * 10U +
                                             (data[8] - '0'));
    }
    else
    {
        device->firmware_version = 0U;
    }

    return HAL_OK;
}

HAL_StatusTypeDef GroveMultiSwitch_SetEventMode(
    GroveMultiSwitch_HandleTypeDef *device,
    uint8_t enable)
{
    uint8_t command = enable ? GROVE_MULTI_SWITCH_CMD_EVENT_MODE
                             : GROVE_MULTI_SWITCH_CMD_BLOCK_MODE;

    if (device == NULL)
    {
        return HAL_ERROR;
    }

    return GroveMultiSwitch_Write(device, &command, 1U);
}

HAL_StatusTypeDef GroveMultiSwitch_Init(GroveMultiSwitch_HandleTypeDef *device,
                                        I2C_HandleTypeDef *hi2c,
                                        uint16_t address)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t device_id = 0U;

    if (device == NULL || hi2c == NULL)
    {
        return HAL_ERROR;
    }

    memset(device, 0, sizeof(*device));
    device->hi2c = hi2c;
    device->address = address;

    /* Match the Arduino library's four probe attempts. */
    for (uint8_t attempt = 0U; attempt < 4U; attempt++)
    {
        status = GroveMultiSwitch_ReadDeviceID(device, &device_id);
        if (status == HAL_OK &&
            GroveMultiSwitch_GetVendorID(device_id) ==
                GROVE_MULTI_SWITCH_VENDOR_ID)
        {
            break;
        }
    }

    if (status != HAL_OK ||
        GroveMultiSwitch_GetVendorID(device_id) != GROVE_MULTI_SWITCH_VENDOR_ID)
    {
        return HAL_ERROR;
    }

    device->device_id = device_id;

    if (GroveMultiSwitch_GetProductID(device_id) ==
        GROVE_MULTI_SWITCH_PID_5_WAY)
    {
        device->button_count = 5U;
    }
    else if (GroveMultiSwitch_GetProductID(device_id) ==
             GROVE_MULTI_SWITCH_PID_6_POS_DIP)
    {
        device->button_count = 6U;
    }
    else
    {
        return HAL_ERROR;
    }

    /* Firmware probing is informational in the Arduino implementation. */
    (void)GroveMultiSwitch_ReadFirmwareVersion(device);

    /* Enable single/double/long-press event detection. */
    return GroveMultiSwitch_SetEventMode(device, 1U);
}

HAL_StatusTypeDef GroveMultiSwitch_ReadEvent(
    GroveMultiSwitch_HandleTypeDef *device,
    GroveMultiSwitch_EventTypeDef *event)
{
    uint8_t data[sizeof(uint32_t) + GROVE_MULTI_SWITCH_BUTTON_MAX] = {0};
    uint16_t length;
    HAL_StatusTypeDef status;

    if (device == NULL || event == NULL ||
        device->button_count == 0U ||
        device->button_count > GROVE_MULTI_SWITCH_BUTTON_MAX)
    {
        return HAL_ERROR;
    }

    length = (uint16_t)(sizeof(uint32_t) + device->button_count);
    status = GroveMultiSwitch_ReadRegister(device,
                                           GROVE_MULTI_SWITCH_CMD_GET_EVENT,
                                           data,
                                           length);
    if (status != HAL_OK)
    {
        return status;
    }

    memset(event, 0, sizeof(*event));
    event->event = GroveMultiSwitch_U32FromLittleEndian(data);
    memcpy(event->button,
           &data[sizeof(uint32_t)],
           device->button_count);

    /* Firmware 0.1 does not reliably report LEVEL_CHANGED. */
    if (device->firmware_version <= 1U && device->previous_button_valid)
    {
        for (uint8_t index = 0U; index < device->button_count; index++)
        {
            event->button[index] &= (uint8_t)~GROVE_MULTI_SWITCH_LEVEL_CHANGED;

            if ((event->button[index] ^ device->previous_button[index]) &
                GROVE_MULTI_SWITCH_RAW_STATUS)
            {
                event->button[index] |= GROVE_MULTI_SWITCH_LEVEL_CHANGED;
                event->event |= GROVE_MULTI_SWITCH_EVENT_PRESENT;
            }
        }
    }

    memcpy(device->previous_button,
           event->button,
           device->button_count);
    device->previous_button_valid = 1U;

    return HAL_OK;
}

uint32_t GroveMultiSwitch_GetDeviceID(
    const GroveMultiSwitch_HandleTypeDef *device)
{
    return (device == NULL) ? 0U : device->device_id;
}

uint8_t GroveMultiSwitch_GetButtonCount(
    const GroveMultiSwitch_HandleTypeDef *device)
{
    return (device == NULL) ? 0U : device->button_count;
}

const char *GroveMultiSwitch_GetFirmwareString(
    const GroveMultiSwitch_HandleTypeDef *device)
{
    return (device == NULL) ? NULL : device->firmware_string;
}
