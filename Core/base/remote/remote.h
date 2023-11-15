//
// Created by Administrator on 2023/11/15.
//

#ifndef EC_HW_PLATFORM_REMOTE_H
#define EC_HW_PLATFORM_REMOTE_H
//#include "base/common/connect.h"
#include "usart.h"
//#include "bsp/board/struct_typedef.h"

#define RC_RX_BUF_SIZE 36u
#define RC_FRAME_LEN 18u


class RC {
public:
    // remote switch 遥控器拨挡
    typedef enum RCSwitchState {
        UP,
        MID,
        DOWN,
    } RCSwitchState_e;

public:
    RC(UART_HandleTypeDef* huart = nullptr);

    void init(void);
    void reset(void);
    void handle(void);

    bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }
    void rxCallback(void);
    void idleCallback(void);
public:
    // connect state 遥控器连接状态
//    Connect connect_;

    // remote channel 遥控器通道
    struct RCChannel {
        int16_t r_row;
        int16_t r_col;
        int16_t l_row;
        int16_t l_col;
//        int16_t dial_wheel;
    } channel_;

    // remote switch 遥控器拨挡
    struct RCSwitch {
        RCSwitchState_e l;
        RCSwitchState_e r;
    } switch_;


private:
    UART_HandleTypeDef* huart_;
    uint8_t rx_buf_[RC_RX_BUF_SIZE], rx_data_[RC_FRAME_LEN];
    volatile uint8_t rx_len_;

    struct RCRaw {
        int16_t ch[5];
        uint8_t s[2];
    } rc_raw_;
};

#endif //EC_HW_PLATFORM_REMOTE_H
