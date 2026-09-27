#include "qi3_protocol.h"

#include <string.h>

enum {
    QI3_HEADER_0 = 0x0A,               /* 固定帧头第 1 字节。 */
    QI3_HEADER_1 = 0x0C,               /* 固定帧头第 2 字节。 */
    QI3_CMD_MOTOR_POWER = 0x01,        /* 电机使能。 */
    QI3_CMD_MOVE_MM_S = 0x02,          /* mm/s 运动控制。 */
    QI3_CMD_PARK = 0x03,               /* 驻车控制。 */
    QI3_CMD_MOVE_RPM = 0x04,           /* rpm 运动控制。 */
    QI3_CMD_ORIGIN_CALIBRATION = 0x05, /* 原点校准。 */
    QI3_CMD_QUERY_VERSION = 0x11       /* 查询固件版本。 */
};

/* 按协议的小端序写入一个 16 位无符号整数；负数按补码发送。 */
static void write_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8u);
}

/* 计算指定字节区间的逐字节 XOR 校验值。 */
static uint8_t xor_bytes(const uint8_t *bytes, size_t size)
{
    uint8_t result = 0u;
    size_t i;

    for (i = 0u; i < size; ++i) {
        result ^= bytes[i];
    }
    return result;
}

/*
 * 组装通用协议帧。
 * data_size 只表示 DATA 区长度；校验覆盖长度字段、命令和 DATA 区，
 * 所以校验起始地址是 bytes[2]，参与校验的总字节数为 data_size + 3。
 */
static Qi3Status make_frame(Qi3Frame *frame,
                            uint8_t command,
                            const uint8_t *data,
                            uint8_t data_size)
{
    if (frame == NULL || (data_size > 0u && data == NULL)) {
        return QI3_ERR_NULL;
    }

    frame->bytes[0] = QI3_HEADER_0;
    frame->bytes[1] = QI3_HEADER_1;
    write_u16_le(&frame->bytes[2], data_size);
    frame->bytes[4] = command;
    if (data_size > 0u) {
        memcpy(&frame->bytes[5], data, data_size);
    }
    frame->bytes[5u + data_size] = xor_bytes(&frame->bytes[2], data_size + 3u);
    frame->size = (size_t)data_size + 6u;
    return QI3_OK;
}

/* 组装只有一个字节数据的命令，如使能、驻车和版本查询。 */
static Qi3Status make_one_byte_command(Qi3Frame *frame,
                                       uint8_t command,
                                       bool enable)
{
    const uint8_t value = enable ? 1u : 0u;
    return make_frame(frame, command, &value, 1u);
}

/* 组装两种运动命令共有的 6 字节数据区，并检查参数范围。 */
static Qi3Status make_motion_command(Qi3Frame *frame,
                                     uint8_t command,
                                     int16_t velocity,
                                     int16_t yaw_cdeg,
                                     int16_t in_place,
                                     int16_t velocity_min,
                                     int16_t velocity_max,
                                     int16_t in_place_min,
                                     int16_t in_place_max)
{
    uint8_t data[6];

    if (frame == NULL) {
        return QI3_ERR_NULL;
    }
    if (velocity < velocity_min || velocity > velocity_max
        || yaw_cdeg < -2500 || yaw_cdeg > 2500
        || in_place < in_place_min || in_place > in_place_max) {
        return QI3_ERR_RANGE;
    }

    write_u16_le(&data[0], (uint16_t)velocity);
    write_u16_le(&data[2], (uint16_t)yaw_cdeg);
    write_u16_le(&data[4], (uint16_t)in_place);
    return make_frame(frame, command, data, sizeof(data));
}

const char *qi3_status_string(Qi3Status status)
{
    switch (status) {
    case QI3_OK: return "OK";
    case QI3_ERR_NULL: return "null argument";
    case QI3_ERR_RANGE: return "value out of range";
    case QI3_ERR_FORMAT: return "invalid frame";
    default: return "unknown error";
    }
}

bool qi3_frame_is_valid(const Qi3Frame *frame)
{
    uint16_t data_size;

    if (frame == NULL || frame->size < 6u || frame->size > QI3_FRAME_MAX_SIZE
        || frame->bytes[0] != QI3_HEADER_0 || frame->bytes[1] != QI3_HEADER_1) {
        return false;
    }

    data_size = (uint16_t)frame->bytes[2]
                | ((uint16_t)frame->bytes[3] << 8u);
    return data_size <= 32u
           && frame->size == (size_t)data_size + 6u
           && frame->bytes[frame->size - 1u]
                  == xor_bytes(&frame->bytes[2], (size_t)data_size + 3u);
}

HAL_StatusTypeDef qi3_send_frame(UART_HandleTypeDef *huart,
                                 const Qi3Frame *frame,
                                 uint32_t timeout)
{
    if (huart == NULL || frame == NULL || !qi3_frame_is_valid(frame)) {
        return HAL_ERROR;
    }

    /* HAL_UART_Transmit 的 pData 参数没有声明为 const，发送过程不会修改帧。 */
    return HAL_UART_Transmit(huart,
                             (uint8_t *)frame->bytes,
                             (uint16_t)frame->size,
                             timeout);
}

Qi3Status qi3_motor_power(Qi3Frame *frame, bool enable)
{
    /* 帧格式：0A 0C 01 00 01 [01=使能/00=停止] XOR。 */
    return make_one_byte_command(frame, QI3_CMD_MOTOR_POWER, enable);
}

Qi3Status qi3_move_mm_s(Qi3Frame *frame,
                        int16_t velocity_mm_s,
                        int16_t yaw_cdeg,
                        int16_t in_place_mm_s)
{
    /* 帧格式：0A 0C 06 00 02 [速度2] [Yaw2] [原地速度2] XOR。 */
    return make_motion_command(frame, QI3_CMD_MOVE_MM_S,
                               velocity_mm_s, yaw_cdeg, in_place_mm_s,
                               -2000, 2000, -1000, 1000);
}

Qi3Status qi3_park(Qi3Frame *frame, bool enable)
{
    /* 帧格式：0A 0C 01 00 03 [01=驻车/00=解除] XOR。 */
    return make_one_byte_command(frame, QI3_CMD_PARK, enable);
}

Qi3Status qi3_move_rpm(Qi3Frame *frame,
                       int16_t velocity_rpm,
                       int16_t yaw_cdeg,
                       int16_t in_place_rpm)
{
    /* 帧格式：0A 0C 06 00 04 [速度2] [Yaw2] [原地转速2] XOR。 */
    return make_motion_command(frame, QI3_CMD_MOVE_RPM,
                               velocity_rpm, yaw_cdeg, in_place_rpm,
                               -120, 120, -60, 60);
}

Qi3Status qi3_origin_calibration(Qi3Frame *frame)
{
    /* 固定帧：0A 0C 01 00 05 01 05。 */
    return make_one_byte_command(frame, QI3_CMD_ORIGIN_CALIBRATION, true);
}

Qi3Status qi3_query_version(Qi3Frame *frame)
{
    /* 固定帧：0A 0C 01 00 11 01 11。 */
    return make_one_byte_command(frame, QI3_CMD_QUERY_VERSION, true);
}
