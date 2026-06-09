#ifndef __APPLICATION_H
#define __APPLICATION_H

#define WS_1  0x0E
#define WS_0  0x08

#define WS_11 0xEE
#define WS_00 0x88

#include <stdint.h>

#include "spi.h"

#define LED_NUMS 5
#define WS_RESET_PERIODS 5

#define RGB_WAVELENGTH 160
#define RGBPWM_WAVELENGTH 15

#define CRC16_INIT (0xffff)

//Disable struct align
#pragma pack (1)
typedef struct ws_data
{
    uint8_t g[4];
    uint8_t r[4];
    uint8_t b[4];
} WS_data;
#pragma pack ()

// from miniPC packet:
typedef struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2];
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  uint8_t is_self_color_red;

  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;

  float forward_vel;
  float leftward_vel;
  uint8_t spintop_level;
  
  uint16_t crc16;
}VisionToGimbal_t;

// vofa sending for getting value for testing:
typedef struct __attribute__((packed)) VOFA
{
    float value[10];
    uint8_t tail[4];
}VOFA;

extern uint8_t rx_buffer[64]; // used for received message
extern VisionToGimbal_t packet;
extern VOFA vofa_report;

void ws2812_init(float wave_brightness);
void ws2812_refresh(void);
void ws2812_pure(uint8_t r, uint8_t g, uint8_t b);
void ws2812_rgbwave(int phase);
void ws2812_rgbpwmwave(int phase, int pulselength, uint8_t r, uint8_t g, uint8_t b);

void init(void);
void step(const float dt);
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size); // receive to idle callback

#endif
