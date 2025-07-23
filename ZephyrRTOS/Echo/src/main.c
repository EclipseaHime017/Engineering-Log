#include "ares/board/init.h"
#include "ares/ares_comm.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define G 9.80665f

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x)&0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

static sync_table_t *pack_tx = NULL;
static sync_table_t *pack_rx = NULL;
static sync_table_t *speed_tx = NULL;
static sync_table_t *imu_tx = NULL;

float motor_parameter_pack[9] = {90.0f,45.0f,-45.0f,-90.0f,-90.0f,-45.0f,45.0f,90.0f,30.0f};
/* 周期性通过 pack_tx 向 PC 发送 9 路电机参数；PC 通过 0x0101 帧发来目标角度 */
float motor_cmd_buffer[9] = {0};  // 接收命令的缓冲区
float motor_speed_pack[9] = {
  120.5f,   // 电机1速度
  -85.2f,   // 电机2速度 
  200.0f,   // 电机3速度
  -150.8f,  // 电机4速度
  95.3f,    // 电机5速度
  -45.6f,   // 电机6速度
  180.2f,   // 电机7速度
  -75.4f,   // 电机8速度
  110.9f    // 电机9速度
};

float imu_pack[7] = {
  1.0f,     // 四元数q0 (w)
  0.0f,     // 四元数q1 (x)
  0.0f,     // 四元数q2 (y)
  0.0f,     // 四元数q3 (z)
  0.15f,    // 角速度X (rad/s)
  -0.08f,   // 角速度Y (rad/s)
  0.25f     // 角速度Z (rad/s)
};

void tx_cb(int status)
{
    if (status == SYNC_PACK_STATUS_READ) {
        LOG_DBG("Motor state data sent to PC successfully!");
    } else if (status == SYNC_PACK_STATUS_WRITE) {
        LOG_DBG("Motor state data received from PC!");
    }
}

void rx_cb(int status)
{
    if (status == SYNC_PACK_STATUS_WRITE) {
        LOG_INF("Motor commands received from PC:");
        for (int i = 0; i < 9; ++i) {
            LOG_INF("motor[%d] = %.2f", i, motor_cmd_buffer[i]);
            // 这里可以添加电机控制逻辑
            // motor_set_angle(motors[i], motor_cmd_buffer[i]);
        }
    } else if (status == SYNC_PACK_STATUS_READ) {
        LOG_DBG("Motor command request sent to PC!");
    }
}

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
int main(void) 
{
  ares_bind_interface(&usb_bulk_interface, &dual_protocol);
  /*
   * 0x0201 : MCU -> Host  (MotorState-A)
   * 0x0101 : Host -> MCU  (MotorCmd)
   */
  pack_tx = dual_sync_add(&dual_protocol, 0x0201, (uint8_t *)motor_parameter_pack, 36, tx_cb);
  pack_rx = dual_sync_add(&dual_protocol, 0x0101, (uint8_t *)motor_cmd_buffer, 36, rx_cb);
  speed_tx = dual_sync_add(&dual_protocol, 0x0202, (uint8_t *)motor_speed_pack, 36, tx_cb);
  imu_tx = dual_sync_add(&dual_protocol, 0x0301, (uint8_t *)imu_pack, 28, tx_cb);

  LOG_INF("USB transfer initialized and sync pack created!");
  while (1) {
    /* 将当前 motor_parameter_pack 内容推送给上位机，20 ms 一次 */
    dual_sync_flush(&dual_protocol, pack_tx);
    k_msleep(2);
    dual_sync_flush(&dual_protocol, speed_tx);
    k_msleep(2);
    dual_sync_flush(&dual_protocol, imu_tx);
    k_msleep(2);
  }
}
