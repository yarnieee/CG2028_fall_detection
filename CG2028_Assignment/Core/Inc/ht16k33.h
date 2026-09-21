#ifndef HT16K33_H
#define HT16K33_H

#include "stm32l4xx_hal.h"

#define HT16K33_DEFAULT_ADDRESS       (0x70U << 1)
#define HT16K33_MATRIX_ROWS           8U
#define HT16K33_MATRIX_COLUMNS        8U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    uint8_t ram[16];
} HT16K33_HandleTypeDef;

HAL_StatusTypeDef HT16K33_Init(HT16K33_HandleTypeDef *display,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t address);
HAL_StatusTypeDef HT16K33_Update(HT16K33_HandleTypeDef *display);
HAL_StatusTypeDef HT16K33_SetBrightness(HT16K33_HandleTypeDef *display,
                                         uint8_t brightness);
HAL_StatusTypeDef HT16K33_SetBlink(HT16K33_HandleTypeDef *display,
                                    uint8_t enabled);

void HT16K33_Clear(HT16K33_HandleTypeDef *display);
void HT16K33_SetPixel(HT16K33_HandleTypeDef *display,
                      uint8_t row,
                      uint8_t column,
                      uint8_t on);
void HT16K33_SetRow(HT16K33_HandleTypeDef *display,
                    uint8_t row,
                    uint8_t value);

#endif /* HT16K33_H */
