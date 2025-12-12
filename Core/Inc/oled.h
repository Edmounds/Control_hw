#ifndef OLED_H
#define OLED_H

#include "main.h"
#include <stdint.h>

/* Compatibility typedefs for legacy code */
typedef uint8_t u8;
typedef uint32_t u32;

#define OLED_SCL_PORT GPIOB
#define OLED_SCL_PIN  GPIO_PIN_9
#define OLED_SDA_PORT GPIOB
#define OLED_SDA_PIN  GPIO_PIN_8

#define OLED_SCLK_Set()                                                   \
  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCLK_Clr()                                                   \
  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)

#define OLED_SDIN_Set()                                                   \
  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDIN_Clr()                                                   \
  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_Init(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_Set_Pos(u8 x, u8 y);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size);
void OLED_ShowString(u8 x, u8 y, const char *str, u8 size);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size);
void OLED_ShowCHinese(u8 x, u8 y, u8 no);
void OLED_DrawBMP(u8 x0, u8 y0, u8 x1, u8 y1, const u8 *bmp);

#endif
