#include "oled.h"

#include "oledfont.h"
#include "bmp.h"
#include "gpio.h"

#define OLED_SLAVE_ADDR 0x78
#define OLED_WRITE_CMD  0x00
#define OLED_WRITE_DATA 0x40

static void OLED_GPIO_Config(void);
static void IIC_Start(void);
static void IIC_Stop(void);
static void IIC_Wait_Ack(void);
static void Write_IIC_Byte(uint8_t byte);
static void Write_IIC_Command(uint8_t command);
static void Write_IIC_Data(uint8_t data);
static void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
static uint32_t oled_pow(uint8_t m, uint8_t n);
static void OledDelay(void);

static void OLED_GPIO_Config(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_SCL_PORT, &gpio_init);

  OLED_SCLK_Set();
  OLED_SDIN_Set();
}

static void OledDelay(void)
{
  for (volatile uint16_t i = 0; i < 5; i++) {
    __NOP();
  }
}

static void IIC_Start(void)
{
  OLED_SDIN_Set();
  OLED_SCLK_Set();
  OledDelay();
  OLED_SDIN_Clr();
  OledDelay();
  OLED_SCLK_Clr();
}

static void IIC_Stop(void)
{
  OLED_SCLK_Set();
  OledDelay();
  OLED_SDIN_Clr();
  OledDelay();
  OLED_SDIN_Set();
}

static void IIC_Wait_Ack(void)
{
  OLED_SCLK_Set();
  OledDelay();
  OLED_SCLK_Clr();
  OledDelay();
}

static void Write_IIC_Byte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++) {
    if (byte & 0x80) {
      OLED_SDIN_Set();
    } else {
      OLED_SDIN_Clr();
    }
    byte <<= 1;
    OLED_SCLK_Set();
    OledDelay();
    OLED_SCLK_Clr();
    OledDelay();
  }
}

static void Write_IIC_Command(uint8_t command)
{
  IIC_Start();
  Write_IIC_Byte(OLED_SLAVE_ADDR);
  IIC_Wait_Ack();
  Write_IIC_Byte(OLED_WRITE_CMD);
  IIC_Wait_Ack();
  Write_IIC_Byte(command);
  IIC_Wait_Ack();
  IIC_Stop();
}

static void Write_IIC_Data(uint8_t data)
{
  IIC_Start();
  Write_IIC_Byte(OLED_SLAVE_ADDR);
  IIC_Wait_Ack();
  Write_IIC_Byte(OLED_WRITE_DATA);
  IIC_Wait_Ack();
  Write_IIC_Byte(data);
  IIC_Wait_Ack();
  IIC_Stop();
}

static void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
  if (cmd == OLED_DATA) {
    Write_IIC_Data(dat);
  } else {
    Write_IIC_Command(dat);
  }
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
  OLED_WR_Byte((uint8_t)(0xb0 + y), OLED_CMD);
  OLED_WR_Byte((uint8_t)(((x & 0xf0) >> 4) | 0x10), OLED_CMD);
  OLED_WR_Byte((uint8_t)(x & 0x0f), OLED_CMD);
}

void OLED_Display_On(void)
{
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x14, OLED_CMD);
  OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x10, OLED_CMD);
  OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_Clear(void)
{
  for (uint8_t i = 0; i < 8; i++) {
    OLED_WR_Byte((uint8_t)(0xb0 + i), OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    for (uint8_t n = 0; n < 128; n++) {
      OLED_WR_Byte(0x00, OLED_DATA);
    }
  }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size)
{
  uint8_t c = chr - ' ';
  if (x > 127) {
    x = 0;
    y += 2;
  }
  if (size == 16) {
    OLED_Set_Pos(x, y);
    for (uint8_t i = 0; i < 8; i++) {
      OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
    }
    OLED_Set_Pos(x, (uint8_t)(y + 1));
    for (uint8_t i = 0; i < 8; i++) {
      OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
  } else {
    OLED_Set_Pos(x, y);
    for (uint8_t i = 0; i < 6; i++) {
      OLED_WR_Byte(F6x8[c][i], OLED_DATA);
    }
  }
}

static uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;
  while (n--) {
    result *= m;
  }
  return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len,
                  uint8_t size)
{
  uint8_t enshow = 0;
  for (uint8_t t = 0; t < len; t++) {
    uint8_t temp = (uint8_t)((num / oled_pow(10, (uint8_t)(len - t - 1))) %
                             10);
    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        OLED_ShowChar((uint8_t)(x + (size / 2) * t), y, ' ', size);
        continue;
      } else {
        enshow = 1;
      }
    }
    OLED_ShowChar((uint8_t)(x + (size / 2) * t), y,
                  (uint8_t)(temp + '0'), size);
  }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
  uint8_t j = 0;
  while (str[j] != '\0') {
    OLED_ShowChar(x, y, (uint8_t)str[j], size);
    x += 8;
    if (x > 120) {
      x = 0;
      y = (uint8_t)(y + 2);
    }
    j++;
  }
}

void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
  OLED_Set_Pos(x, y);
  for (uint8_t t = 0; t < 16; t++) {
    OLED_WR_Byte((uint8_t)Hzk[(uint16_t)no * 2][t], OLED_DATA);
  }
  OLED_Set_Pos(x, (uint8_t)(y + 1));
  for (uint8_t t = 0; t < 16; t++) {
    OLED_WR_Byte((uint8_t)Hzk[(uint16_t)no * 2 + 1][t], OLED_DATA);
  }
}

void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
                  const uint8_t *bmp)
{
  uint16_t j = 0;
  uint8_t y_pages = (uint8_t)((y1 % 8 == 0) ? (y1 / 8) : (y1 / 8 + 1));

  for (uint8_t y = y0; y < y_pages; y++) {
    OLED_Set_Pos(x0, y);
    for (uint8_t x = x0; x < x1; x++) {
      OLED_WR_Byte(bmp[j++], OLED_DATA);
    }
  }
}

void OLED_Init(void)
{
  OLED_GPIO_Config();
  HAL_Delay(50);

  OLED_WR_Byte(0xAE, OLED_CMD);
  OLED_WR_Byte(0x00, OLED_CMD);
  OLED_WR_Byte(0x10, OLED_CMD);
  OLED_WR_Byte(0x40, OLED_CMD);
  OLED_WR_Byte(0xB0, OLED_CMD);
  OLED_WR_Byte(0x81, OLED_CMD);
  OLED_WR_Byte(0xFF, OLED_CMD);
  OLED_WR_Byte(0xA1, OLED_CMD);
  OLED_WR_Byte(0xA6, OLED_CMD);
  OLED_WR_Byte(0xA8, OLED_CMD);
  OLED_WR_Byte(0x3F, OLED_CMD);
  OLED_WR_Byte(0xC8, OLED_CMD);
  OLED_WR_Byte(0xD3, OLED_CMD);
  OLED_WR_Byte(0x00, OLED_CMD);
  OLED_WR_Byte(0xD5, OLED_CMD);
  OLED_WR_Byte(0x80, OLED_CMD);
  OLED_WR_Byte(0xD8, OLED_CMD);
  OLED_WR_Byte(0x05, OLED_CMD);
  OLED_WR_Byte(0xD9, OLED_CMD);
  OLED_WR_Byte(0xF1, OLED_CMD);
  OLED_WR_Byte(0xDA, OLED_CMD);
  OLED_WR_Byte(0x12, OLED_CMD);
  OLED_WR_Byte(0xDB, OLED_CMD);
  OLED_WR_Byte(0x30, OLED_CMD);
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x14, OLED_CMD);
  OLED_WR_Byte(0xAF, OLED_CMD);

  OLED_Clear();
}
