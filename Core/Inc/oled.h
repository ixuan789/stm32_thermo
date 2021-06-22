#ifndef __OLED_H
#define __OLED_H
#include "main.h"

//-----------------OLED端口定义---------------- 

#define SCLK_L() HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET)//SCL
#define SCLK_H() HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET)

#define MOSI_L() HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET)//MOSI
#define MOSI_H() HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET)

#define RST_L()  HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET)//RES
#define RST_H()  HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET)

#define DC_L()   HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET)//DC
#define DC_H()   HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET)

#define CS_L()   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)//CS
#define CS_H()   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)


#define OLED_CMD  0    //写命令
#define OLED_DATA 1    //写数据

void OLED_ClearPoint(uint8_t x,uint8_t y);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1);
void OLED_ScrollDisplay(uint8_t num,uint8_t space);
void OLED_WR_BP(uint8_t x,uint8_t y);
void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t BMP[]);
void OLED_Init(void);



#endif