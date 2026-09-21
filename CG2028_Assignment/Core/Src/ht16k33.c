#include "ht16k33.h"

#include <string.h>

static HAL_StatusTypeDef HT16K33_SendCommand(HT16K33_HandleTypeDef *display,
                                             uint8_t command)
{
    return HAL_I2C_Master_Transmit(display->hi2c,
                                   display->address,
                                   &command,
                                   1,
                                   100);
}

HAL_StatusTypeDef HT16K33_Init(HT16K33_HandleTypeDef *display,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t address)
{
    display->hi2c = hi2c;
    display->address = address;
    HT16K33_Clear(display);

    /* Turn on the internal oscillator, display, and maximum brightness. */
    if (HT16K33_SendCommand(display, 0x21) != HAL_OK ||
        HT16K33_SetBlink(display, 0) != HAL_OK ||
        HT16K33_SetBrightness(display, 15) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HT16K33_Update(display);
}

HAL_StatusTypeDef HT16K33_SetBrightness(HT16K33_HandleTypeDef *display,
                                         uint8_t brightness)
{
    if (brightness > 15U)
    {
        brightness = 15U;
    }

    return HT16K33_SendCommand(display, (uint8_t)(0xE0U | brightness));
}

HAL_StatusTypeDef HT16K33_SetBlink(HT16K33_HandleTypeDef *display,
                                    uint8_t enabled)
{
    /* 0x81 = display on, blink off; 0x83 = display on, 2 Hz blink. */
    return HT16K33_SendCommand(display, enabled ? 0x83U : 0x81U);
}

HAL_StatusTypeDef HT16K33_Update(HT16K33_HandleTypeDef *display)
{
    uint8_t packet[17];

    packet[0] = 0x00; /* First display RAM address. */
    memcpy(&packet[1], display->ram, sizeof(display->ram));

    return HAL_I2C_Master_Transmit(display->hi2c,
                                   display->address,
                                   packet,
                                   sizeof(packet),
                                   100);
}

void HT16K33_Clear(HT16K33_HandleTypeDef *display)
{
    memset(display->ram, 0, sizeof(display->ram));
}

void HT16K33_SetRow(HT16K33_HandleTypeDef *display,
                    uint8_t row,
                    uint8_t value)
{
    if (row < HT16K33_MATRIX_ROWS)
    {
        display->ram[row * 2U] = value;
        display->ram[(row * 2U) + 1U] = 0;
    }
}

void HT16K33_SetPixel(HT16K33_HandleTypeDef *display,
                      uint8_t row,
                      uint8_t column,
                      uint8_t on)
{
    if (row >= HT16K33_MATRIX_ROWS || column >= HT16K33_MATRIX_COLUMNS)
    {
        return;
    }

    uint8_t *row_data = &display->ram[row * 2U];
    uint8_t mask = (uint8_t)(1U << column);

    if (on)
    {
        row_data[0] |= mask;
    }
    else
    {
        row_data[0] &= (uint8_t)~mask;
    }
}
