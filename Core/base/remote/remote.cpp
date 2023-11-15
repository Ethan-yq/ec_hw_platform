/**
 ******************************************************************************
 * @file    remote.cpp/h
 * @brief   Remote control. 遥控器
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "Core/base/remote/remote.h"
#include <string.h>

// max time to wait for connect(ms)
const uint32_t rc_connect_timeout = 100;
const int16_t rc_ch_offset = 1024;
const int16_t rc_ch_min = 364;   // -660
const int16_t rc_ch_max = 1684;  // 660

RC::RC(UART_HandleTypeDef* huart)
        : huart_(huart), connect_(rc_connect_timeout) {
    rx_len_ = 0;
    switch_.l = DOWN;
    switch_.r = DOWN;
}

// Start UART(SBUS) receive. 打开UART接收
void RC::init(void) {
    rx_len_ = 0;
    reset();
    if (huart_ != nullptr) {
        __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
        HAL_UART_Receive_DMA(huart_, rx_buf_, 1);
    }
}

// Reset RC data. 重置遥控器数据
void RC::reset(void) {
    rc_raw_.ch[0] = 0;
    rc_raw_.ch[1] = 0;
    rc_raw_.ch[2] = 0;
    rc_raw_.ch[3] = 0;
    rc_raw_.s[0] = 0;
    rc_raw_.s[1] = 0;
    mouse_.x = 0;
    mouse_.y = 0;
    mouse_.z = 0;
    mouse_.press_l = false;
    mouse_.press_r = false;
    key_ = 0;
    rc_raw_.ch[4] = 0;

    channel_.r_row = 0;
    channel_.r_col = 0;
    channel_.l_row = 0;
    channel_.l_col = 0;
    channel_.dial_wheel = 0;
}

// Unpack data. 数据解包
void RC::handle(void) {
    // Check connection
    if (!connect_.check()) {
        reset();
        return;
    }

    // Unpack data. 数据解包
    rc_raw_.ch[0] = (rx_data_[0] | rx_data_[1] << 8) & 0x07ff;       // Channel 0
    rc_raw_.ch[1] = (rx_data_[1] >> 3 | rx_data_[2] << 5) & 0x07ff;  // Channel 1
    rc_raw_.ch[2] = (rx_data_[2] >> 6 | rx_data_[3] << 2 | rx_data_[4] << 10) &
                    0x07ff;                                          // Channel 2
    rc_raw_.ch[3] = (rx_data_[4] >> 1 | rx_data_[5] << 7) & 0x07ff;  // Channel 3
    rc_raw_.s[0] = (rx_data_[5] >> 4 & 0x0003);        // Switch left
    rc_raw_.s[1] = (rx_data_[5] >> 4 & 0x000C) >> 2;   // Switch right
    mouse_.x = rx_data_[6] | rx_data_[7] << 8;         // Mouse X axis
    mouse_.y = rx_data_[8] | rx_data_[9] << 8;         // Mouse Y axis
    mouse_.z = rx_data_[10] | rx_data_[11] << 8;       // Mouse Z axis
    mouse_.press_l = (rx_data_[12] != 0);              // Mouse left pressed
    mouse_.press_r = (rx_data_[13] != 0);              // Mouse right pressed
    key_ = rx_data_[14] | rx_data_[15] << 8;           // Keyboard
    rc_raw_.ch[4] = rx_data_[16] | rx_data_[17] << 8;  // Dial wheel

    updateKeyState();

    channel_.r_row = rc_raw_.ch[0] - rc_ch_offset;
    channel_.r_col = rc_raw_.ch[1] - rc_ch_offset;
    channel_.l_row = rc_raw_.ch[2] - rc_ch_offset;
    channel_.l_col = rc_raw_.ch[3] - rc_ch_offset;
    channel_.dial_wheel = rc_raw_.ch[4] - rc_ch_offset;
    if (rc_raw_.s[0] == 1) {
        switch_.r = UP;
    } else if (rc_raw_.s[0] == 2) {
        switch_.r = DOWN;
    } else if (rc_raw_.s[0] == 3) {
        switch_.r = MID;
    }
    if (rc_raw_.s[1] == 1) {
        switch_.l = UP;
    } else if (rc_raw_.s[1] == 2) {
        switch_.l = DOWN;
    } else if (rc_raw_.s[1] == 3) {
        switch_.l = MID;
    }
}

// Update connect status, restart UART(SBUS) receive.
// 更新连接状态，重新打开UART(SBUS)接收
void RC::rxCallback(void) {
    rx_len_++;
    if (huart_ != nullptr) {
        HAL_UART_Receive_DMA(huart_, rx_buf_ + rx_len_, 1);
    }
}

// Idle callback, update connect status.
// 空闲中断，判断数据长度，更新连接状态，重新打开UART(SBUS)接收
void RC::idleCallback(void) {
    // Check frame length. 判断帧数据长度
    if (rx_len_ >= RC_FRAME_LEN) {
        connect_.refresh();
        memcpy(rx_data_, rx_buf_ + rx_len_ - RC_FRAME_LEN, RC_FRAME_LEN);
    }
    rx_len_ = 0;
    if (huart_ != nullptr) {
        HAL_UART_AbortReceive(huart_);
        HAL_UART_Receive_DMA(huart_, rx_buf_, 1);
    }
}

KeyPressState updateOneKeyState(bool pressed , KeyPressState& state){
    if(pressed){
        state.is_pressed = true;
        if(state.press_state == RELEASE){
            state.press_state = JUST_PRESS;
            state.last_pressed_tick = HAL_GetTick();
        }
        else{
            state.press_state = PRESS;
            state.time = HAL_GetTick() - state.last_pressed_tick;
            if(state.time > 750){
                state.press_state = LONG_PRESS;
            }
        }
    }
    else{
        if(state.is_pressed){
            state.is_pressed = false;
            state.press_state = JUST_RELEASE;
        } else{
            state.press_state = RELEASE;
        }

        if(state.press_state == PRESS || state.press_state == LONG_PRESS){
            state.press_state = RELEASE;
            state.time = 0;
            state.last_released_tick = HAL_GetTick();
        }
    }
    return state;
}

void RC::updateKeyState(void) {
    updateOneKeyState(key_ & KEY_W, keys_state_.key_W_state);
    updateOneKeyState(key_ & KEY_S, keys_state_.key_S_state);
    updateOneKeyState(key_ & KEY_A, keys_state_.key_A_state);
    updateOneKeyState(key_ & KEY_D, keys_state_.key_D_state);
    updateOneKeyState(key_ & KEY_Q, keys_state_.key_Q_state);
    updateOneKeyState(key_ & KEY_E, keys_state_.key_E_state);
    updateOneKeyState(key_ & KEY_R, keys_state_.key_R_state);
    updateOneKeyState(key_ & KEY_F, keys_state_.key_F_state);
    updateOneKeyState(key_ & KEY_G, keys_state_.key_G_state);
    updateOneKeyState(key_ & KEY_Z, keys_state_.key_Z_state);
    updateOneKeyState(key_ & KEY_X, keys_state_.key_X_state);
    updateOneKeyState(key_ & KEY_C, keys_state_.key_C_state);
    updateOneKeyState(key_ & KEY_V, keys_state_.key_V_state);
    updateOneKeyState(key_ & KEY_B, keys_state_.key_B_state);
    updateOneKeyState(key_ & KEY_SHIFT, keys_state_.key_SHIFT_state);
    updateOneKeyState(key_ & KEY_CTRL, keys_state_.key_CTRL_state);
    updateOneKeyState(mouse_.press_l, keys_state_.mouse_L_state);
    updateOneKeyState(mouse_.press_r, keys_state_.mouse_R_state);
}//
// Created by Administrator on 2023/11/15.
//