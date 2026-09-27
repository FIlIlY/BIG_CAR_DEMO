/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口与 USART2 DMA 收发测试
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "qi3_protocol.h"

/* 本文件专用的头文件 --------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/* USER CODE END Includes */

/* 本文件专用的类型定义 ------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* 本文件专用的宏定义 --------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* 本文件专用的宏 ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* 本文件专用的变量 ----------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* 本文件内函数的前置声明 ----------------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* 本文件的用户自定义代码 ----------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define USART2_HOST_RX_BUFFER_SIZE 64U  /* USART2 每次 DMA 接收的字节数。 */
#define HOST_COMMAND_MAX_LENGTH 64U     /* 上位机单条命令最多 63 个字符，末尾留出字符串结束符。 */
#define HOST_COMMAND_QUEUE_DEPTH 5U     /* 环形队列留一个空槽区分满和空，实际可暂存 4 条命令。 */
#define HOST_RESPONSE_QUEUE_DEPTH 9U    /* 环形队列实际可排队 8 条 USART2 应答。 */
#define HOST_RESPONSE_MAX_LENGTH 32U    /* 单条应答最多 31 个字符，末尾留出字符串结束符。 */

/* USART2 的 DMA 临时接收区。接收回调会把字节逐个拼接成以换行符结束的命令。 */
static uint8_t usart2_host_rx_buffer[USART2_HOST_RX_BUFFER_SIZE];
/* 当前正在拼接的上位机命令；可容纳最多 HOST_COMMAND_MAX_LENGTH - 1 个字符。 */
static char host_command_line[HOST_COMMAND_MAX_LENGTH];
/* 当前命令长度和溢出标记只在 USART2 接收回调中更新。 */
static volatile uint8_t host_command_line_length;
static volatile uint8_t host_command_line_overflow;
/* 命令环形队列：接收中断负责写入，主循环负责取出并执行。 */
static char host_command_queue[HOST_COMMAND_QUEUE_DEPTH][HOST_COMMAND_MAX_LENGTH];
static volatile uint8_t host_command_queue_head;
static volatile uint8_t host_command_queue_tail;
/* 应答队列由主循环写入、USART2 DMA 发送完成回调读出。 */
static char host_response_queue[HOST_RESPONSE_QUEUE_DEPTH][HOST_RESPONSE_MAX_LENGTH];
static volatile uint8_t host_response_queue_head;
static volatile uint8_t host_response_queue_tail;
static volatile uint8_t host_response_queue_overflow_pending;
/* 接收中断无法直接等待发送应答，用标记通知主循环上报输入过长或队列已满。 */
static volatile uint8_t host_line_too_long_pending;
static volatile uint8_t host_queue_full_pending;
static volatile uint8_t usart2_rx_restart_failed;

typedef enum
{
  HOST_COMMAND_OK,
  HOST_COMMAND_UNKNOWN,
  HOST_COMMAND_FORMAT_ERROR,
  HOST_COMMAND_RANGE_ERROR,
  HOST_COMMAND_UART_ERROR
} HostCommandResult;

/* 在主循环中解析并执行一条上位机命令，然后把 Qi-3-G 帧从 USART1 发出。 */
static HostCommandResult execute_host_command(char *command);
/* 通过 USART2 返回简短的 ASCII 执行结果。 */
static void send_host_response(const char *response);

/* USER CODE END 0 */

/**
  * @brief  主程序入口：初始化外设，接收上位机命令并通过 USART1 控制 Qi-3-G 底盘。
  * @retval int  按嵌入式程序约定不会返回。
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU 初始化 ----------------------------------------------------------------*/

  /* 初始化 HAL 库、复位外设状态，并配置 Flash 接口和系统节拍定时器 SysTick。 */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 按下方配置系统时钟：使用外部晶振和 PLL 提供系统时钟。 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化 GPIO、DMA 和三个串口：USART2 接上位机，USART1 向 Qi-3-G 底盘控制器发命令。 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USART2 配置为 115200 波特率、8 数据位、无校验、1 个停止位；
   * PA2 为 TX、PA3 为 RX，收发分别使用 DMA1 Stream6 和 Stream5。
   */
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 启动 USART2 的 DMA 空闲接收，作为上位机到 STM32 的命令输入通道。
   * 命令按 ASCII 文本传输，每条命令必须以回车或换行结束；若 USB 转串口分多段送达，
   * 接收回调会先拼接字节，直到收到行结束符才把完整命令交给主循环。
   * 支持 MOTOR_ON、MOTOR_OFF、STOP、PARK、UNPARK、CALIBRATE；
   * 运动命令为 MOVE_MM_S 速度 转向角速度 原地速度，或 MOVE_RPM 转速 转向角速度 原地转速。
   * 参数之间可用空格或逗号分隔；STM32 返回 OK SENT 表示帧已从 USART1 发出。
   */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                   usart2_host_rx_buffer,
                                   sizeof(usart2_host_rx_buffer)) != HAL_OK)
  {
    Error_Handler();
  }
  /* 关闭 DMA 半传输中断；只有接收空闲或缓冲区收满时才处理收到的字节。 */
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* 命令处理主循环 ------------------------------------------------------------*/
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 如果接收中断期间发现命令过长或队列已满，在主循环中排队通知上位机。
     * 应答使用 USART2 TX DMA，避免阻塞主循环，也避免和 RX DMA 的中断重启互相影响。
     */
    if (host_line_too_long_pending != 0U)
    {
      host_line_too_long_pending = 0U;
      send_host_response("ERR LINE_TOO_LONG\r\n");
    }
    if (host_queue_full_pending != 0U)
    {
      host_queue_full_pending = 0U;
      send_host_response("ERR COMMAND_QUEUE_FULL\r\n");
    }

    /* 读写指针不同表示队列中至少有一条完整命令。
     * 先把命令复制到局部缓冲区，再推进读指针，之后接收回调即可复用原队列槽位。
     */
    if (host_command_queue_head != host_command_queue_tail)
    {
      char command[HOST_COMMAND_MAX_LENGTH];
      uint8_t queue_slot = host_command_queue_head;

      memcpy(command, host_command_queue[queue_slot], sizeof(command));
      host_command_queue_head =
          (uint8_t)((queue_slot + 1U) % HOST_COMMAND_QUEUE_DEPTH);

      HostCommandResult result = execute_host_command(command);
      switch (result)
      {
        case HOST_COMMAND_OK:
          send_host_response("OK SENT\r\n");
          break;
        case HOST_COMMAND_UNKNOWN:
          send_host_response("ERR UNKNOWN_COMMAND\r\n");
          break;
        case HOST_COMMAND_FORMAT_ERROR:
          send_host_response("ERR FORMAT\r\n");
          break;
        case HOST_COMMAND_RANGE_ERROR:
          send_host_response("ERR RANGE\r\n");
          break;
        case HOST_COMMAND_UART_ERROR:
        default:
          send_host_response("ERR USART1_TX\r\n");
          break;
      }
    }

    /* 主循环把排队的应答交给 USART2 TX DMA；DMA 完成回调负责释放队首槽位。 */
    if ((host_response_queue_head != host_response_queue_tail) &&
        (huart2.gState == HAL_UART_STATE_READY))
    {
      uint8_t response_slot = host_response_queue_head;
      uint16_t response_length =
          (uint16_t)strlen(host_response_queue[response_slot]);

      (void)HAL_UART_Transmit_DMA(&huart2,
                                  (uint8_t *)host_response_queue[response_slot],
                                  response_length);
    }

    /* 应答队列曾经满时，等队列腾出位置后再通知上位机。 */
    if (host_response_queue_overflow_pending != 0U)
    {
      uint8_t next_tail =
          (uint8_t)((host_response_queue_tail + 1U) % HOST_RESPONSE_QUEUE_DEPTH);
      if (next_tail != host_response_queue_head)
      {
        host_response_queue_overflow_pending = 0U;
        send_host_response("ERR RESPONSE_QUEUE_FULL\r\n");
      }
    }

    /* 如果接收事件后无法重新启动 DMA，停止运行并留在错误处理处。 */
    if (usart2_rx_restart_failed != 0U)
    {
      Error_Handler();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief 配置系统时钟：使用 HSE 外部晶振和 PLL 生成 168 MHz 的 SYSCLK。
  * @retval 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 使能电源控制模块，并选择稳压器输出电压档位 1，以支持较高主频。 */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* 打开 HSE 外部高速时钟，并配置 PLL：HSE / 4 * 168 / 2 = 168 MHz。 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* SYSCLK 取 PLL 输出；AHB 不分频，APB1 分频为 4，APB2 分频为 2。 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* USART2 接收上位机命令；本回调只收集字节，不执行底盘控制或阻塞发送。 */
  if (huart != &huart2)
  {
    return;
  }

  /* 接收事件由线路空闲或 DMA 缓冲区收满触发；半传输事件已关闭。
   * Size 是本次 DMA 收到的字节数。将其逐字节加入命令行，直到遇到 CR 或 LF。
   */
  if ((HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_HT) &&
      (Size > 0U))
  {
    if (Size > sizeof(usart2_host_rx_buffer))
    {
      /* HAL 正常不会报告超过 DMA 缓冲区的长度；此检查用于保护数组边界。 */
      Size = sizeof(usart2_host_rx_buffer);
    }

    for (uint16_t i = 0U; i < Size; ++i)
    {
      char received_byte = (char)usart2_host_rx_buffer[i];

      if ((received_byte == '\r') || (received_byte == '\n'))
      {
        /* 一条命令以回车或换行结束。CRLF 会连续产生两个结束符，空行会被忽略。 */
        if (host_command_line_overflow != 0U)
        {
          host_line_too_long_pending = 1U;
        }
        else if (host_command_line_length > 0U)
        {
          uint8_t queue_slot = host_command_queue_tail;
          uint8_t next_tail =
              (uint8_t)((queue_slot + 1U) % HOST_COMMAND_QUEUE_DEPTH);

          /* 环形队列采用单生产者/单消费者结构，保留一个空槽区分队列满和队列空。 */
          if (next_tail == host_command_queue_head)
          {
            host_queue_full_pending = 1U;
          }
          else
          {
            memcpy(host_command_queue[queue_slot],
                   host_command_line,
                   host_command_line_length);
            host_command_queue[queue_slot][host_command_line_length] = '\0';
            /* 先写完整条命令，再更新写指针，主循环据此判断新命令已就绪。 */
            host_command_queue_tail = next_tail;
          }
        }

        host_command_line_length = 0U;
        host_command_line_overflow = 0U;
      }
      else if (host_command_line_overflow == 0U)
      {
        if ((received_byte == '\0') ||
            (host_command_line_length >= (HOST_COMMAND_MAX_LENGTH - 1U)))
        {
          /* 拒绝二进制空字节或超长输入；直到行结束符才重新开始收集命令。 */
          host_command_line_overflow = 1U;
        }
        else
        {
          host_command_line[host_command_line_length] = received_byte;
          host_command_line_length++;
        }
      }
    }
  }

  /* USART2 RX DMA 使用普通模式，每次空闲事件或缓冲区收满后会停止。
   * 重新启动后继续关闭半传输中断，等待上位机的下一段输入。
   */
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                   usart2_host_rx_buffer,
                                   sizeof(usart2_host_rx_buffer)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else
  {
    /* 由主循环进入统一错误处理；不要在中断服务程序内停留在死循环。 */
    usart2_rx_restart_failed = 1U;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* USART2 应答通过 DMA 发送；发送完成后释放当前应答槽位。 */
  if ((huart == &huart2) &&
      (host_response_queue_head != host_response_queue_tail))
  {
    host_response_queue_head =
        (uint8_t)((host_response_queue_head + 1U) % HOST_RESPONSE_QUEUE_DEPTH);
  }
}

/* 将一条简短应答复制到队列，确保 DMA 使用的数据在发送完成前一直有效。 */
static void send_host_response(const char *response)
{
  uint8_t queue_slot = host_response_queue_tail;
  uint8_t next_tail =
      (uint8_t)((queue_slot + 1U) % HOST_RESPONSE_QUEUE_DEPTH);
  size_t response_length = strlen(response);

  if (next_tail == host_response_queue_head)
  {
    host_response_queue_overflow_pending = 1U;
    return;
  }

  if (response_length >= HOST_RESPONSE_MAX_LENGTH)
  {
    response_length = HOST_RESPONSE_MAX_LENGTH - 1U;
  }
  memcpy(host_response_queue[queue_slot], response, response_length);
  host_response_queue[queue_slot][response_length] = '\0';
  host_response_queue_tail = next_tail;
}

/* 严格解析一个十进制有符号 16 位参数，区分格式错误和数值越界。 */
static HostCommandResult parse_int16_argument(char *text, int16_t *value)
{
  char *end;
  long parsed_value;

  if ((text == NULL) || (value == NULL))
  {
    return HOST_COMMAND_FORMAT_ERROR;
  }

  errno = 0;
  parsed_value = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return HOST_COMMAND_FORMAT_ERROR;
  }
  if ((errno == ERANGE) || (parsed_value < INT16_MIN) ||
      (parsed_value > INT16_MAX))
  {
    return HOST_COMMAND_RANGE_ERROR;
  }

  *value = (int16_t)parsed_value;
  return HOST_COMMAND_OK;
}

/* 解析一条上位机文本命令并经 USART1 发送对应的 Qi-3-G 二进制控制帧。 */
static HostCommandResult execute_host_command(char *command)
{
  char *token;
  char *argument_1;
  char *argument_2;
  char *argument_3;
  int16_t value_1;
  int16_t value_2;
  int16_t value_3;
  bool is_rpm_command = false;
  Qi3Frame frame;
  Qi3Status frame_status;
  HostCommandResult parse_status;

  token = strtok(command, " ,\t");
  if (token == NULL)
  {
    return HOST_COMMAND_FORMAT_ERROR;
  }

  /* 命令名称不区分大小写，参数必须是十进制整数。 */
  for (char *letter = token; *letter != '\0'; ++letter)
  {
    if ((*letter >= 'a') && (*letter <= 'z'))
    {
      *letter = (char)(*letter - 'a' + 'A');
    }
  }

  if (strcmp(token, "MOTOR_ON") == 0)
  {
    if (strtok(NULL, " ,\t") != NULL) return HOST_COMMAND_FORMAT_ERROR;
    frame_status = qi3_motor_power(&frame, true);
  }
  else if (strcmp(token, "MOTOR_OFF") == 0)
  {
    if (strtok(NULL, " ,\t") != NULL) return HOST_COMMAND_FORMAT_ERROR;
    frame_status = qi3_motor_power(&frame, false);
  }
  else if (strcmp(token, "STOP") == 0)
  {
    /* 停车命令发送零线速度，不等同于关闭电机使能。 */
    if (strtok(NULL, " ,\t") != NULL) return HOST_COMMAND_FORMAT_ERROR;
    frame_status = qi3_move_mm_s(&frame, 0, 0, 0);
  }
  else if (strcmp(token, "PARK") == 0)
  {
    if (strtok(NULL, " ,\t") != NULL) return HOST_COMMAND_FORMAT_ERROR;
    frame_status = qi3_park(&frame, true);
  }
  else if (strcmp(token, "UNPARK") == 0)
  {
    if (strtok(NULL, " ,\t") != NULL) return HOST_COMMAND_FORMAT_ERROR;
    frame_status = qi3_park(&frame, false);
  }
  else if (strcmp(token, "CALIBRATE") == 0)
  {
    if (strtok(NULL, " ,\t") != NULL) return HOST_COMMAND_FORMAT_ERROR;
    frame_status = qi3_origin_calibration(&frame);
  }
  else if ((strcmp(token, "MOVE_MM_S") == 0) ||
           (strcmp(token, "MOVE_RPM") == 0))
  {
    /* 运动命令必须带三个参数：速度、转向角速度、原地旋转速度。 */
    is_rpm_command = (strcmp(token, "MOVE_RPM") == 0);
    argument_1 = strtok(NULL, " ,\t");
    argument_2 = strtok(NULL, " ,\t");
    argument_3 = strtok(NULL, " ,\t");
    if ((argument_1 == NULL) || (argument_2 == NULL) ||
        (argument_3 == NULL) || (strtok(NULL, " ,\t") != NULL))
    {
      return HOST_COMMAND_FORMAT_ERROR;
    }

    parse_status = parse_int16_argument(argument_1, &value_1);
    if (parse_status != HOST_COMMAND_OK) return parse_status;
    parse_status = parse_int16_argument(argument_2, &value_2);
    if (parse_status != HOST_COMMAND_OK) return parse_status;
    parse_status = parse_int16_argument(argument_3, &value_3);
    if (parse_status != HOST_COMMAND_OK) return parse_status;

    if (is_rpm_command)
    {
      frame_status = qi3_move_rpm(&frame, value_1, value_2, value_3);
    }
    else
    {
      frame_status = qi3_move_mm_s(&frame, value_1, value_2, value_3);
    }
  }
  else
  {
    return HOST_COMMAND_UNKNOWN;
  }

  if (frame_status == QI3_ERR_RANGE)
  {
    return HOST_COMMAND_RANGE_ERROR;
  }
  if (frame_status != QI3_OK)
  {
    return HOST_COMMAND_FORMAT_ERROR;
  }

  /* OK SENT 表示 STM32 已把帧发出 USART1，不代表下游控制器已执行该指令。 */
  if (qi3_send_frame(&huart1, &frame, 100U) != HAL_OK)
  {
    return HOST_COMMAND_UART_ERROR;
  }
  return HOST_COMMAND_OK;
}

/* USER CODE END 4 */

/**
  * @brief  HAL 或外设初始化发生不可恢复错误时进入此函数。
  * @retval 无
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 关闭全局中断并停在此处，便于连接调试器检查错误现场。 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  启用 full assert 模式时报告断言失败的位置。
  * @param  file  发生断言失败的源文件名。
  * @param  line  发生断言失败的源代码行号。
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 可在此添加错误输出，例如通过调试串口打印 file 和 line，便于定位断言失败位置。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT：启用 HAL 参数断言时编译以上断言处理函数。 */
