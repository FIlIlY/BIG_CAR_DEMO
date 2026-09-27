#ifndef QI3_PROTOCOL_H
#define QI3_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * This project uses STM32 HAL for the optional UART send helper below.
 * The frame-building functions themselves do not depend on a particular UART.
 */
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Qi-3-G chassis frame format (all multi-byte values are little-endian):
 *
 *   0A 0C LEN_L LEN_H CMD DATA... XOR
 *   |  |    |       |    |     |      |
 *   header  data length  command  data  checksum
 *
 * LEN is the number of bytes in DATA. The XOR starts at LEN_L and covers
 * LEN_L, LEN_H, CMD and every DATA byte.
 */
#define QI3_FRAME_MAX_SIZE 38u

typedef enum {
    QI3_OK = 0,          /* 指令已成功生成。 */
    QI3_ERR_NULL = -1,   /* 传入了空指针。 */
    QI3_ERR_RANGE = -2,  /* 参数超出底盘协议允许的范围。 */
    QI3_ERR_FORMAT = -3 /* 帧长度、帧头或 XOR 校验不正确。 */
} Qi3Status;

typedef struct {
    uint8_t bytes[QI3_FRAME_MAX_SIZE]; /* 待发送的原始字节。 */
    size_t size;                        /* 有效字节数，不是数组容量。 */
} Qi3Frame;

/* 将错误码转换为适合日志输出的英文短文本。 */
const char *qi3_status_string(Qi3Status status);

/* 检查帧头、长度和 XOR，发送前可用此函数做最后一次校验。 */
bool qi3_frame_is_valid(const Qi3Frame *frame);

/*
 * 通过 STM32 HAL UART 发送已经生成的 Qi-3-G 信息帧。
 *
 * huart 必须对应实际接到 Qi-3-G 控制器的串口（例如 &huart1）。
 * STM32F1 UART 的波特率、数据位、停止位等参数由 CubeMX/usart.c 配置。
 */
HAL_StatusTypeDef qi3_send_frame(UART_HandleTypeDef *huart,
                                 const Qi3Frame *frame,
                                 uint32_t timeout);

/*
 * 命令 0x01：使能或停止底盘电机。enable=true 为使能。
 * 信息帧：0A 0C 01 00 01 ENABLE XOR
 *         |  |        |  |      |
 *         帧头 LEN=1 CMD=01  01=使能/00=停止  校验
 * 例如使能帧：0A 0C 01 00 01 01 01。
 */
Qi3Status qi3_motor_power(Qi3Frame *frame, bool enable);

/*
 * 命令 0x02：按线速度控制底盘。
 * velocity_mm_s：前进方向线速度，单位 mm/s，范围 -2000~2000。
 * yaw_cdeg：转向角速度，单位 0.01 度，范围 -2500~2500。
 * in_place_mm_s：原地旋转线速度，单位 mm/s，范围 -1000~1000。
 * 普通前进、后退或转向时应将 in_place_mm_s 设为 0。
 * 信息帧：0A 0C 06 00 02 VEL_L VEL_H YAW_L YAW_H ANG_L ANG_H XOR
 * 例如 velocity=100、yaw=0、in_place=0：
 *         0A 0C 06 00 02 64 00 00 00 00 00 60。
 */
Qi3Status qi3_move_mm_s(Qi3Frame *frame,
                         int16_t velocity_mm_s,
                         int16_t yaw_cdeg,
                         int16_t in_place_mm_s);

/*
 * 命令 0x03：使能或解除驻车。enable=true 为驻车。
 * 信息帧：0A 0C 01 00 03 ENABLE XOR，其中 ENABLE=01 驻车、00 解除驻车。
 * 例如驻车帧：0A 0C 01 00 03 01 03。
 */
Qi3Status qi3_park(Qi3Frame *frame, bool enable);

/*
 * 命令 0x04：按轮端转速控制底盘。
 * velocity_rpm：前进方向转速，单位 rpm，范围 -120~120。
 * yaw_cdeg：转向角速度，单位 0.01 度，范围 -2500~2500。
 * in_place_rpm：原地旋转转速，单位 rpm，范围 -60~60。
 * 普通运动时应将 in_place_rpm 设为 0。
 * 信息帧：0A 0C 06 00 04 VEL_L VEL_H YAW_L YAW_H RPM_L RPM_H XOR。
 */
Qi3Status qi3_move_rpm(Qi3Frame *frame,
                        int16_t velocity_rpm,
                        int16_t yaw_cdeg,
                        int16_t in_place_rpm);

/*
 * 命令 0x05：执行底盘原点校准。校准期间应保持车体静止。
 * 信息帧：0A 0C 01 00 05 01 05。
 */
Qi3Status qi3_origin_calibration(Qi3Frame *frame);

/*
 * 命令 0x11：请求底盘返回固件版本信息。
 * 信息帧：0A 0C 01 00 11 01 11。
 * 底盘收到后通过串口返回版本信息。
 */
Qi3Status qi3_query_version(Qi3Frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* QI3_PROTOCOL_H */
