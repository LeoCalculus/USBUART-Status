/**
 * @file ws2812.c
 * @author Liwei Xue (luo@tianyi.vc)
 * @brief WS2812 driver for STM32
 * @version 0.1
 * @date 2024-03-22
 * 
 * @copyright Copyright (c) Liwei Xue 2024
 * 
 */
#include <application.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <usart.h>

#define WS2812_SPI &hspi1

WS_data ws2812_spibuf[WS_RESET_PERIODS + LED_NUMS + WS_RESET_PERIODS];
uint8_t RGB_waveform[RGB_WAVELENGTH];
uint8_t rx_buffer[64];
VisionToGimbal_t packet;
VOFA vofa_report;

static uint16_t ms_counter = 0;
static uint16_t timeout_counter = 0;
static uint8_t blink_flag = 0;


void init(void){
    vofa_report.tail[0] = 0x00;
    vofa_report.tail[1] = 0x00;
    vofa_report.tail[2] = 0x80;
    vofa_report.tail[3] = 0x7f;
}

void step(const float dt){
    // packet was set somewhere else so just assign and send:
    // vofa_report.value[0] = packet.is_self_color_red;
    // vofa_report.value[1] = packet.head[0];
    // vofa_report.value[2] = packet.crc16;

    // HAL_UART_Transmit_DMA(&huart2, (void*)&vofa_report, sizeof(vofa_report));

    uint8_t r = packet.is_self_color_red ? 255 : 0;
    uint8_t g = 0;
    uint8_t b = packet.is_self_color_red ? 0 : 255;

    if (packet.mode == 1 || packet.mode == 2)
    {
        if (++ms_counter >= 100)
        {
            ms_counter = 0;
            blink_flag = !blink_flag;
        }

        if (blink_flag)
        {
            r = 0;
            g = 0;
            b = 0;
        }
    }
    else
    {
        ms_counter = 0;
        blink_flag = 0;
    }

    if (timeout_counter < 3000){
        timeout_counter++;
    } else {
        r = 128;
        g = 0;
        b = 128;
    }

    ws2812_pure(r, g, b);
    ws2812_refresh();
}

// CRC stuff:
const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFF;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        wCRC = (uint16_t)(wCRC >> 8) ^ wCRC_Table[(uint16_t)(wCRC ^ (uint16_t)chData) & 0x00ff];
    }
    return wCRC;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART1) {
        // check for packet first before memmove
        if (rx_buffer[0] == 'S' && rx_buffer[1] == 'P' && Size == sizeof(VisionToGimbal_t)){ // the packet must be exactly 39 bytes
            // check crc16 first
            uint16_t received_crc16 = (uint16_t)rx_buffer[37] | ((uint16_t)rx_buffer[38] << 8);
            uint16_t calculated_crc16 = Get_CRC16_Check_Sum(rx_buffer, Size-2, CRC16_INIT);
            if (calculated_crc16 == received_crc16) {
                memcpy(&packet, rx_buffer, sizeof(VisionToGimbal_t));
            }

        }
        timeout_counter = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, sizeof(rx_buffer)); // restart the dma
    }
}

/**
 * @brief Initialize ws2812 data, calculate RGB waveform, etc.
 * 
 * @param wave_brightness Brightness of RGB wave, 0~1
 */
void ws2812_init(float wave_brightness){
    for(int i=0;i<RGB_WAVELENGTH;i++){
        float omega=((float)i)/RGB_WAVELENGTH*2*3.1415926;
        RGB_waveform[i]=(uint8_t)(255.0f*wave_brightness*(0.5+cosf(omega)/2));
    }
}

/**
 * @brief Update colour to LED strip
 * 
 */
void ws2812_refresh(void){
    //ws2812_pure(5,20,10);
    HAL_SPI_Transmit_DMA(WS2812_SPI,(uint8_t *)ws2812_spibuf, sizeof(ws2812_spibuf));
    return;
}

/**
 * @brief Reset LEDs to all black
 * 
 */
void ws2812_resetbuf(void){
    memset(ws2812_spibuf,0,sizeof(ws2812_spibuf));
}

/**
 * @brief Generate SPI data for a given RGB value.
 * 
 * @param setr Red
 * @param setg Green
 * @param setb Blue
 * @return WS_data 
 */
WS_data ws2812_getData(uint8_t setr, uint8_t setg, uint8_t setb){
    WS_data data;
    for(int j=0;j<4;j++){
        data.r[j] = (setr & 0x80 ? WS_1 : WS_0) << 4;
        data.r[j] |= (setr & 0x40 ? WS_1 : WS_0) ;
        setr <<= 2;

        data.g[j] = (setg & 0x80 ? WS_1 : WS_0) << 4;
        data.g[j] |= (setg & 0x40 ? WS_1 : WS_0) ;
        setg <<= 2;

        data.b[j] = (setb & 0x80 ? WS_1 : WS_0) << 4;
        data.b[j] |= (setb & 0x40 ? WS_1 : WS_0) ;
        setb <<= 2;
    }
    return data;
}

/**
 * @brief Display pure colour on LEDs
 * 
 * @param r red
 * @param g green
 * @param b blue
 */
void ws2812_pure(uint8_t r, uint8_t g, uint8_t b){
    ws2812_resetbuf();
    for(int i=0;i<LED_NUMS;i++){
        ws2812_spibuf[WS_RESET_PERIODS + i]=ws2812_getData(r,g,b);
    }
}

/**
 * @brief Generate a RGB "wave" to ws2812 buffer
 * 
 * @param phase phase of "wave". Unit: nums of LED
 */
void ws2812_rgbwave(int phase){
    ws2812_resetbuf();
    for(int i=0;i<LED_NUMS;i++){
        uint8_t r = RGB_waveform[(i+phase)%RGB_WAVELENGTH];
        uint8_t g = RGB_waveform[(i+phase+RGB_WAVELENGTH/3)%RGB_WAVELENGTH];
        uint8_t b = RGB_waveform[(i+phase+RGB_WAVELENGTH*2/3)%RGB_WAVELENGTH];
        ws2812_spibuf[WS_RESET_PERIODS + i] = ws2812_getData(r,g,b);
    }
}

/**
 * @brief Generate a RGB "PWM" wave.
 * 
 * @param phase phase of "wave". Unit: nums of LED
 * @param pulselength pulse length Unit: nums of LED
 * @param r red
 * @param g green
 * @param b blue
 */
void ws2812_rgbpwmwave(int phase, int pulselength, uint8_t r, uint8_t g, uint8_t b){
    ws2812_resetbuf();
    WS_data pulsecolour=ws2812_getData(r,g,b);
    WS_data dark=ws2812_getData(0,0,0);
    for(int i=0;i<LED_NUMS;i++){
        if((i+phase)%RGBPWM_WAVELENGTH<pulselength){
            ws2812_spibuf[WS_RESET_PERIODS + i] = pulsecolour;
        }else{
            ws2812_spibuf[WS_RESET_PERIODS + i] = dark;
        }
    }
}
