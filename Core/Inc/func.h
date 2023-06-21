#ifndef INC_func_H_
#define INC_func_H_

#include "stm32f1xx_hal.h"
#include"CRC.h"
#include"CONFIG.h"
#include "stdbool.h"
#include "string.h"

//#define POLLING
//#define IWG
#define DMA

#define Tx_Rx_EN_Pin GPIO_PIN_4
#define Tx_Rx_EN_GPIO_Port GPIOA
#define MAX_SIZE_UARTSEND 5
#define UARTRX_BUFF_SIZE 50
#define MAX_SIZE_UARTRECV 5
#ifdef IWG
extern IWDG_HandleTypeDef hiwdg;
#endif
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

static uint8_t uartSend_Store[5][50];
static uint8_t uartSend_Store_Count[5];
static uint8_t uartSend_Store_Size = 0;
static uint8_t flag_send_uart = 0;
static uint32_t time_send_uart = 0;



static uint8_t UART_DATA[50][50];
static uint8_t Size_UART_DATA = 0;
static uint8_t Count_UART_DATA[50];

static uint8_t uartReciver_buffer[UARTRX_BUFF_SIZE];
static uint8_t uartReciver_Store[5][50];
static uint8_t uartRecive_Store_Count[5];
static uint8_t uartReciver_Store_Size =0;

 void uartSend_DMA(uint8_t* Pdata, uint8_t size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
static void Serial_Process(void);
static void Uart_Package_Process(uint8_t* uartPackage , uint8_t uartPackageCout);
#ifdef IWG
static void MX_IWDG_Init(void);
#endif


static uint8_t flag_uart =0;

static uint8_t get_digital[2] = {0xFF,0xFF};
static uint8_t last_digital = 0xff;
//static uint8_t output_digital[30];
static uint32_t time_i2c;
static uint8_t flag_i2c = 0;
static uint32_t time_delay;
static uint8_t data_i2c[30];

static uint8_t data_set[2] = {0xFF,0xFF};
static uint8_t rx_buffer[50];
static uint8_t DATA[50][50];
static uint8_t sizeDATA;
static uint8_t countDATA[50];
static uint16_t Timer[50];
//static uint8_t count_sensor =0;
static uint8_t cmd_ID[50];
static uint8_t cmd_ID_toggle[50];
//static uint8_t select_func[50];
static uint32_t real_time[50];
static bool read_adc_error = false;
static uint16_t data_adc[8];

static uint8_t DATA_toggle[20][50];
static uint8_t sizeDATA_toggle;

static uint8_t debug[20];


static uint32_t digital_read_timer;
static uint32_t adc_read_timer;

static uint8_t connected;

static uint8_t digital_pcf8575(uint8_t portP_);
static uint16_t adc_ads1115(uint8_t portA_);
static void get_data_adc();
void function(uint8_t* input , uint8_t size);
static void read_digital();//0 port
static void read_adc(uint8_t* input,uint8_t count_adc);
static void write_digital(); //2 port
static void scan_i2c(); //3//i2cd
static void write_i2c(uint8_t* input); //4 i2cd
static void read_i2c(uint8_t* input,uint8_t count_i2c); //5 i2c
static void request_i2c(uint8_t* input,uint8_t size); //6 i2c
static void check_status(uint8_t* input);
static void read_RS485(uint8_t* cmd,uint8_t size);
static void write_to_STM(uint8_t* input,uint8_t count);
static void read_output_digital(uint8_t* input);
static void toggle_output(uint8_t data_pin);
static void toggle(uint8_t* input,uint8_t count);
 void tx_crc(uint8_t c1_CRC);
static void logic_or(uint8_t* input,uint8_t size);
static void write_pin_digital(uint8_t data_pin, uint8_t data);
static void input_toggle(uint8_t *data , uint8_t data_pin);
 void loop(void);
static void toggle_port(uint8_t port);
static void read_input_digital();
static void rs485(uint8_t* payload,uint8_t size);
static uint8_t flag_rs485 = 0;
static uint8_t flag_rs485_rx =0;
static uint8_t count_adc =0;
static uint8_t Reciver_rs485_Count;
static uint8_t uartReciver_rs485[30];


#endif



