#ifndef SSD1306_H
#define SSD1306_H

#include "stm32l4xx_hal.h"

#define SSD1306_WIDTH                 128U
#define SSD1306_HEIGHT                64U
#define SSD1306_BUFFER_SIZE           (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)
#define SSD1306_DEFAULT_ADDRESS       (0x3CU << 1)

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    uint8_t buffer[SSD1306_BUFFER_SIZE];
    uint8_t cursor_x;
    uint8_t cursor_y;
} SSD1306_HandleTypeDef;

HAL_StatusTypeDef SSD1306_Init(SSD1306_HandleTypeDef *display,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t address);
HAL_StatusTypeDef SSD1306_Update(SSD1306_HandleTypeDef *display);
HAL_StatusTypeDef SSD1306_WriteCommand(SSD1306_HandleTypeDef *display,
                                       uint8_t command);

void SSD1306_Clear(SSD1306_HandleTypeDef *display);
void SSD1306_SetCursor(SSD1306_HandleTypeDef *display,
                       uint8_t x,
                       uint8_t y);
void SSD1306_DrawPixel(SSD1306_HandleTypeDef *display,
                       uint8_t x,
                       uint8_t y,
                       uint8_t on);
void SSD1306_WriteChar(SSD1306_HandleTypeDef *display, char character);
void SSD1306_WriteString(SSD1306_HandleTypeDef *display,
                         const char *text);
void SSD1306_DrawLine(SSD1306_HandleTypeDef *display,
                             int x0, int y0,
                             int x1, int y1,
                             uint8_t colour);

void SSD1306_FillRect(SSD1306_HandleTypeDef *display,
                             uint8_t x,
                             uint8_t y,
                             uint8_t width,
                             uint8_t height,
                             uint8_t colour);

void SSD1306_WriteStringColour(SSD1306_HandleTypeDef *display,
    const char *text,
    uint8_t foreground);

#endif /* SSD1306_H */
