#include "ssd1306.h"

#include <string.h>

static void SSD1306_GetGlyph(char character, uint8_t glyph[5])
{
    static const uint8_t uppercase[26][5] =
    {
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
        {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
        {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
        {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
        {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
        {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
        {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
        {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
        {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
        {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
        {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
        {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
        {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
        {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
        {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
        {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
        {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
        {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
        {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
        {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
        {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
        {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
        {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
        {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
        {0x61, 0x51, 0x49, 0x45, 0x43}  /* Z */
    };

    static const uint8_t digits[10][5] =
    {
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
        {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
        {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
        {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
        {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
        {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
        {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
        {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
        {0x06, 0x49, 0x49, 0x29, 0x1E}  /* 9 */
    };

    memset(glyph, 0, 5);

    if (character >= 'a' && character <= 'z')
    {
        character = (char)(character - 'a' + 'A');
    }

    if (character >= 'A' && character <= 'Z')
    {
        memcpy(glyph, uppercase[character - 'A'], 5);
    }
    else if (character >= '0' && character <= '9')
    {
        memcpy(glyph, digits[character - '0'], 5);
    }
    else
    {
        switch (character)
        {
        case ':':
            glyph[1] = 0x36;
            glyph[2] = 0x36;
            break;
        case '.':
            glyph[1] = 0x60;
            glyph[2] = 0x60;
            break;
        case '-':
            glyph[0] = 0x08;
            glyph[1] = 0x08;
            glyph[2] = 0x08;
            glyph[3] = 0x08;
            glyph[4] = 0x08;
            break;
        case '/':
            glyph[0] = 0x20;
            glyph[1] = 0x10;
            glyph[2] = 0x08;
            glyph[3] = 0x04;
            glyph[4] = 0x02;
            break;
        case '?':
            glyph[0] = 0x02;
            glyph[1] = 0x01;
            glyph[2] = 0x51;
            glyph[3] = 0x09;
            glyph[4] = 0x06;
            break;
        default:
            break; /* Space and unsupported characters remain blank. */
        }
    }
}

static HAL_StatusTypeDef SSD1306_WriteCommandWithParameter(
    SSD1306_HandleTypeDef *display,
    uint8_t command,
    uint8_t parameter)
{
    uint8_t packet[3] = {0x00, command, parameter};

    return HAL_I2C_Master_Transmit(display->hi2c,
                                   display->address,
                                   packet,
                                   sizeof(packet),
                                   100);
}

HAL_StatusTypeDef SSD1306_WriteCommand(SSD1306_HandleTypeDef *display,
                                       uint8_t command)
{
    uint8_t packet[2] = {0x00, command};

    return HAL_I2C_Master_Transmit(display->hi2c,
                                   display->address,
                                   packet,
                                   sizeof(packet),
                                   100);
}

HAL_StatusTypeDef SSD1306_Init(SSD1306_HandleTypeDef *display,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t address)
{
    display->hi2c = hi2c;
    display->address = address;
    display->cursor_x = 0;
    display->cursor_y = 0;
    SSD1306_Clear(display);

    HAL_Delay(100);

    /* Standard 128x64 SSD1306 initialization sequence. */
    if (SSD1306_WriteCommand(display, 0xAE) != HAL_OK || /* Display off */
        SSD1306_WriteCommandWithParameter(display, 0xD5, 0x80) != HAL_OK ||
        SSD1306_WriteCommandWithParameter(display, 0xA8, 0x3F) != HAL_OK ||
        SSD1306_WriteCommandWithParameter(display, 0xD3, 0x00) != HAL_OK ||
        SSD1306_WriteCommand(display, 0x40) != HAL_OK || /* Start line */
        SSD1306_WriteCommandWithParameter(display, 0x8D, 0x14) != HAL_OK ||
        SSD1306_WriteCommandWithParameter(display, 0x20, 0x02) != HAL_OK || /* Page addressing */
        SSD1306_WriteCommand(display, 0xA1) != HAL_OK || /* Segment remap */
        SSD1306_WriteCommand(display, 0xC8) != HAL_OK || /* COM scan direction */
        SSD1306_WriteCommandWithParameter(display, 0xDA, 0x12) != HAL_OK ||
        SSD1306_WriteCommandWithParameter(display, 0x81, 0x8F) != HAL_OK ||
        SSD1306_WriteCommandWithParameter(display, 0xD9, 0xF1) != HAL_OK ||
        SSD1306_WriteCommandWithParameter(display, 0xDB, 0x40) != HAL_OK ||
        SSD1306_WriteCommand(display, 0xA4) != HAL_OK || /* Resume RAM */
        SSD1306_WriteCommand(display, 0xA6) != HAL_OK || /* Normal display */
        SSD1306_WriteCommand(display, 0xAF) != HAL_OK)   /* Display on */
    {
        return HAL_ERROR;
    }

    return SSD1306_Update(display);
}

HAL_StatusTypeDef SSD1306_Update(SSD1306_HandleTypeDef *display)
{
    uint8_t packet[SSD1306_WIDTH + 1U];

    packet[0] = 0x40; /* Following bytes are display data. */

    for (uint8_t page = 0; page < (SSD1306_HEIGHT / 8U); page++)
    {
        if (SSD1306_WriteCommand(display, (uint8_t)(0xB0U + page)) != HAL_OK ||
            SSD1306_WriteCommand(display, 0x00) != HAL_OK ||
            SSD1306_WriteCommand(display, 0x10) != HAL_OK)
        {
            return HAL_ERROR;
        }

        memcpy(&packet[1],
               &display->buffer[page * SSD1306_WIDTH],
               SSD1306_WIDTH);

        if (HAL_I2C_Master_Transmit(display->hi2c,
                                   display->address,
                                   packet,
                                   sizeof(packet),
                                   100) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

void SSD1306_Clear(SSD1306_HandleTypeDef *display)
{
    memset(display->buffer, 0, sizeof(display->buffer));
    display->cursor_x = 0;
    display->cursor_y = 0;
}

void SSD1306_SetCursor(SSD1306_HandleTypeDef *display,
                       uint8_t x,
                       uint8_t y)
{
    display->cursor_x = x;
    display->cursor_y = y;
}

void SSD1306_DrawPixel(SSD1306_HandleTypeDef *display,
                       uint8_t x,
                       uint8_t y,
                       uint8_t on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        return;
    }

    uint32_t index = x + ((uint32_t)(y / 8U) * SSD1306_WIDTH);
    uint8_t mask = (uint8_t)(1U << (y % 8U));

    if (on)
    {
        display->buffer[index] |= mask;
    }
    else
    {
        display->buffer[index] &= (uint8_t)~mask;
    }
}

void SSD1306_WriteChar(SSD1306_HandleTypeDef *display, char character)
{
    uint8_t glyph[5];

    if (character == '\n')
    {
        display->cursor_x = 0;
        display->cursor_y = (uint8_t)(display->cursor_y + 8U);
        return;
    }

    if (display->cursor_x > (SSD1306_WIDTH - 6U))
    {
        display->cursor_x = 0;
        display->cursor_y = (uint8_t)(display->cursor_y + 8U);
    }

    if (display->cursor_y > (SSD1306_HEIGHT - 8U))
    {
        display->cursor_y = 0;
    }

    SSD1306_GetGlyph(character, glyph);

    for (uint8_t column = 0; column < 5U; column++)
    {
        for (uint8_t row = 0; row < 7U; row++)
        {
            SSD1306_DrawPixel(display,
                              (uint8_t)(display->cursor_x + column),
                              (uint8_t)(display->cursor_y + row),
                              (glyph[column] >> row) & 0x01U);
        }
    }

    display->cursor_x = (uint8_t)(display->cursor_x + 6U);
}

void SSD1306_WriteString(SSD1306_HandleTypeDef *display,
                         const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        SSD1306_WriteChar(display, *text++);
    }
}
