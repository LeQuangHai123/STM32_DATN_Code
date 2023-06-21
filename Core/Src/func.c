#include "func.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART1){
		if(Size >= 50){
			Size = 50;
		}

		memcpy(&uartReciver_Store[uartReciver_Store_Size][0],uartReciver_buffer,Size);
		uartRecive_Store_Count[uartReciver_Store_Size] = Size;
		uartReciver_Store_Size++;

		if(uartReciver_Store_Size >= MAX_SIZE_UARTRECV){
			uartReciver_Store_Size = MAX_SIZE_UARTRECV - 1;
		}

		flag_uart = 1;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartReciver_buffer, UARTRX_BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

	}
#ifdef DMA
	if(huart->Instance == USART2)
	{
		Reciver_rs485_Count = Size;
		flag_rs485_rx = 1;
		HAL_GPIO_WritePin(Tx_Rx_EN_GPIO_Port, Tx_Rx_EN_Pin, GPIO_PIN_RESET);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartReciver_rs485, 30);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
	}
#endif

}

void Serial_Process(void){
	for(int i = 0 ; i < uartReciver_Store_Size ; i++){
		Uart_Package_Process(&uartReciver_Store[i][0],uartRecive_Store_Count[i]);
	}
	memset(uartRecive_Store_Count,0,uartReciver_Store_Size);
	uartReciver_Store_Size = 0;

}

void Uart_Package_Process(uint8_t* uartPackage , uint8_t uartPackageCout)
{
		uint16_t crc_payload = uartPackage[uartPackageCout-1];
		crc_payload <<= 8;
		crc_payload |= uartPackage[uartPackageCout-2];

		uint16_t crc_check = CRC16(uartPackage, uartPackageCout-2);

		if((crc_payload == crc_check) && (uartPackageCout > 2)){
			memcpy(&UART_DATA[Size_UART_DATA][0],uartPackage,uartPackageCout);
			Count_UART_DATA[Size_UART_DATA] = uartPackageCout;
			Size_UART_DATA++;
		}
}




void loop(void){

	if((HAL_GetTick() - time_i2c >= time_delay)&&(flag_i2c))
	{
		uint8_t data_Recvice[data_i2c[data_i2c[2]+1]];
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_Init(&hi2c2);
		// Khởi tạo giao thức I2C và nhận dữ liệu từ địa chỉ CB
		if(HAL_I2C_Master_Receive(&hi2c2, data_i2c[3]<<1, data_Recvice, data_i2c[data_i2c[2]+1],300) == HAL_OK)
		{
			for(uint8_t i =0;i<2;i++)
			 	rx_buffer[i] = data_i2c[i];
			rx_buffer[2] = data_i2c[data_i2c[2]+1]+1;//lenght +address
			rx_buffer[3] = data_i2c[3];
			for(uint8_t i =0;i<data_i2c[data_i2c[2+2+data_i2c[2]-3]];i++)
				rx_buffer[i+4] = data_Recvice[i];
			tx_crc(rx_buffer[2]+3);

		}
		flag_i2c = 0;
	}

	  if((HAL_GetTick() - digital_read_timer > 200) && (sizeDATA_toggle > 0)){
		  read_input_digital();
		  digital_read_timer = HAL_GetTick();
	  }

	  if(HAL_GetTick() - adc_read_timer > 500){
		  get_data_adc();
		  adc_read_timer = HAL_GetTick();
	  }

	  if(sizeDATA > 0)
	  {
#ifdef IWG
		  MX_IWDG_Init();
#endif
		  for(int i=0;i < sizeDATA;i++)
		  {
#ifdef POLLING
			  if((HAL_GetTick() - real_time[i] >= Timer[i]) && (!flag_i2c))
#endif
#ifdef DMA
			  if((HAL_GetTick() - real_time[i] >= Timer[i]) && (!flag_i2c)&&(!flag_rs485)&&(!flag_rs485_rx))
#endif
			  {
				  function(&DATA[i][0],countDATA[i]);
				  real_time[i] = HAL_GetTick();
			  }

		  }
	  }

	  if(flag_send_uart && (HAL_GetTick() - time_send_uart > 10))
	  {
		flag_send_uart = 0;
		uint8_t uart_error;
		uart_error = HAL_UART_Transmit_DMA(&huart1, &uartSend_Store[0][0], uartSend_Store_Count[0]);
		if((uart_error == HAL_BUSY) || (uart_error == HAL_ERROR)){
			flag_send_uart =1;
			time_send_uart = HAL_GetTick();
		}
	  }



	  if(Size_UART_DATA > 0)
	  {
		  function(&UART_DATA[0][0],Count_UART_DATA[0]);
		  for(int i=0;i<Size_UART_DATA;i++)
		  {
			  memcpy(&UART_DATA[i][0],&UART_DATA[i+1][0],Count_UART_DATA[i+1]);
		  }
		  memcpy(&Count_UART_DATA[0],&Count_UART_DATA[1],Size_UART_DATA);
		  Size_UART_DATA--;
	  }

	  if(flag_uart)
	  {
		  Serial_Process();
		  flag_uart = 0;
	  }
	  if(flag_rs485_rx)
	  {
		  rs485(uartReciver_rs485,Reciver_rs485_Count);
	  }



}










void function(uint8_t* input , uint8_t size)
{
		switch(input[1])
		{
		case 0:
			read_output_digital(input);
			break;
		case 1:
			read_adc(input,size);
			break;
		case 2 :
			write_digital(input);
			break;
		case 3:
			scan_i2c(input);
			break;
		case 4:
			write_i2c(input);
			break;
		case 5:
			read_i2c(input,size);
			break;
		case 6:
			request_i2c(input,size);
			break;
		case 7:
			check_status(input);
			break;
		case 8:
			read_RS485(input,size);
			break;
		case 9:
			write_to_STM(input,size);
			break;
		case 10:
			read_output_digital(input);
			break;
		case 12:
			logic_or(input,size);
			break;
		default :
			break;
		}
}

static void logic_or(uint8_t* input,uint8_t size){

	uint8_t input_data = digital_pcf8575(1 - input[3]);
	uint8_t position;
	uint8_t pin_data = 0x00;
//	debug = input_data;
	for(int i = 0; i < input[4] ; i++){
		position  = 0x01;
		position &= (input_data >> input[5+i])&0x01;
		pin_data |= (~position);
		pin_data &= 0x01;
//		debug = pin_data;
	}
	for(int i = 0; i < input[2] - input[4] -1; i++){
		write_pin_digital(input[input[4]+5+i], pin_data);
	}

}



static void check_status(uint8_t* input){
	if(input[3] == 1 && !connected)
	{
		sizeDATA = 0;
		memset(countDATA,0,50);
		memset(cmd_ID,0,50);
		sizeDATA_toggle = 0;
		memset(DATA_toggle,0,20*50);
		memset(cmd_ID_toggle,0,50);
		connected = true;
	}
	else if(input[3] == 0)
	{
		connected = false;
		data_set[0] = 0xFF;
		data_set[1] = 0xFF;
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		HAL_I2C_Master_Transmit(&hi2c1,address_PCF8575<<1,(uint8_t*)data_set,2,30);
	}
}






/********************************Digital******************************************/
static uint8_t digital_pcf8575(uint8_t portP_)
{
	uint8_t pcf8575_digital[2] = {0xff,0xff};
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);
	HAL_I2C_Master_Receive(&hi2c1, address_PCF8575<<1,pcf8575_digital, 2, 200);
	return pcf8575_digital[portP_];
}


static void read_input_digital()
{
	uint8_t digi = digital_pcf8575(1);
			if(digi != 0Xff)
			{
				if(last_digital != digi){
					for(int i = 0 ; i < sizeDATA_toggle; i++){
						input_toggle(&DATA_toggle[i][0],digi);

					}
					rx_buffer[0] = ID;
					rx_buffer[1] = 11;//cmd write digital
					rx_buffer[2] = 1; //length
					rx_buffer[3] = digital_pcf8575(0); //data address
					tx_crc(4);


				}

					last_digital = digi;
			}
			else
			{
				last_digital = 0;
			}

}

static void write_digital(uint8_t* input)
{
	data_set[0] = digital_pcf8575(0);
	data_set[1] = 0xFF;
	for(int i=0;i<input[2];i+=2)
	{
		data_set[0] = (~(1<<input[3+i])& data_set[0] ) | (input[4+i]<<input[3+i]);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		HAL_I2C_Master_Transmit(&hi2c1,address_PCF8575<<1,(uint8_t*)data_set,2,30);
	}

	rx_buffer[0] = ID;
	rx_buffer[1] = 2;//cmd write digital
	rx_buffer[2] = 1; //length
	rx_buffer[3] = digital_pcf8575(0); //data address
	tx_crc(4);

}


static void read_output_digital(uint8_t* input)
{
	get_digital[1] = digital_pcf8575(0);
	for(int k=0;k<2;k++)
		rx_buffer[k] = input[k];
	rx_buffer[2] = 1;
	rx_buffer[3] = get_digital[1];
	tx_crc(4);

}


static void input_toggle(uint8_t *data , uint8_t data_pin){
	uint8_t position;
	uint8_t data_port = 0x00;
	for(int i = 0; i < data[1] ; i++){
		position  = 0x01;
		position &= data_pin >> data[2+i];
		if((position&0x01) == 0){
			for(int k = 0; k < (data[0]-data[1]-1) ; k++){
				data_port |= (1<<data[data[1]+2+k]);
			}
		}
	}
	toggle_port(data_port);

}

static void toggle_output(uint8_t data_pin)
{
	get_digital[1] = digital_pcf8575(0);
	get_digital[0] = (1<<data_pin);
	data_set[0] = get_digital[0]^get_digital[1];
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);
	HAL_I2C_Master_Transmit(&hi2c1,address_PCF8575<<1,(uint8_t*)data_set,2,30);
	rx_buffer[0] = ID;
	rx_buffer[1] = 11;//cmd write digital
	rx_buffer[2] = 1; //length
	rx_buffer[3] = digital_pcf8575(0); //data address
	tx_crc(4);

}

static void write_pin_digital(uint8_t data_pin, uint8_t data){
	data_set[0] = digital_pcf8575(0);
	data_set[0] =  (~(1<<data_pin)& data_set[0] ) | (((~data)&0x01)<<data_pin);
	data_set[1] = 0xFF;
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);
	HAL_I2C_Master_Transmit(&hi2c1,address_PCF8575<<1,(uint8_t*)data_set,2,20);
}


static void toggle_port(uint8_t port)
{
	data_set[0]=digital_pcf8575(0);
	for(int i=0;i<8;i++)
	{
		if(port & (1<<i))
		{
			data_set[0] ^= (1<<i);
		}
	}
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);

	HAL_I2C_Master_Transmit(&hi2c1,address_PCF8575<<1,(uint8_t*)data_set,2,20);
}


/********************************I2C******************************************/
static void read_i2c(uint8_t* input,uint8_t count_i2c)
{
	//BHV150FVI
	uint8_t data_re[input[4]];
	HAL_I2C_DeInit(&hi2c2);
	HAL_I2C_Init(&hi2c2);
	if(HAL_I2C_Master_Receive(&hi2c2,input[3]<<1,data_re,input[4],200)==HAL_OK)
	{
		for(uint8_t i =0;i<input[4];i++)
			rx_buffer[i] = input[i];
		rx_buffer[2] = input[4]+1; //length
		rx_buffer[3] = input[3];//data address.
		for(uint8_t i =0;i<input[2+2];i++)
			rx_buffer[i+4] = data_re[i];
		tx_crc(rx_buffer[2]+3);

	}

}


/*SCAN I2C*/
static void scan_i2c(uint8_t* input)
{

	rx_buffer[0] = ID;
	rx_buffer[1] = 3;
	rx_buffer[2] = 0;
	for(uint8_t i =1 ; i<128;i++)
	{
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_Init(&hi2c2);
		if(HAL_I2C_IsDeviceReady(&hi2c2, i<<1,2,2) == HAL_OK)
		{
			rx_buffer[2]+=1;
			rx_buffer[rx_buffer[2]+2]= i;
		}
	}
	tx_crc(rx_buffer[2]+3);

}


/* REQUEST I2C */
static void request_i2c(uint8_t* input,uint8_t size)
{

	memcpy(data_i2c,input,size);
		//lưu giá trị đọc được vào biến data_t_t
		uint8_t data_t_t[input[2]-3];//length of data - 1 time delay -1 number of byte - 1 byte address i2c
		for(int i =0;i<input[2]-3;i++)
			data_t_t[i] = input[4+i];
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_Init(&hi2c2);
		///khởi tạo giao thức I2C và gửi dữ liệu đến địa chỉ CB
		if(HAL_I2C_Master_Transmit(&hi2c2, input[3]<<1, (uint8_t *)data_t_t,input[2]-3, 300) == HAL_OK)
		{
			time_i2c = HAL_GetTick();
			time_delay = input[10];
			flag_i2c = 1;
		}
}


static void write_i2c(uint8_t* input)
{
	uint8_t data[input[2]-1];
	for(int i=0;i<input[2]-1;i++)
		data[i] = input[4+i];
	HAL_I2C_DeInit(&hi2c2);
	HAL_I2C_Init(&hi2c2);
	HAL_I2C_Master_Transmit(&hi2c2,input[3]<<1,data,input[2]-1,120);
}


/********************************ADC******************************************/
static uint16_t adc_ads1115(uint8_t portA_)
{
	read_adc_error = false;
	HAL_I2C_DeInit(&hi2c1);
	uint8_t ads1115_cmd[3]={0x01,0x00,0x83};
	uint8_t ads1115_read[2] = {0x00,0x00};
	uint16_t ads1115_adc = 0;
	ads1115_cmd[1]=0xC3 + 0x10*portA_;
	if(portA_<4)
	{
		ads1115_cmd[1]=0xC3 + 0x10*portA_;
		HAL_I2C_Init(&hi2c1);
		if(HAL_I2C_Master_Transmit(&hi2c1,address_ADS1115<<1 ,(uint8_t*)ads1115_cmd, 3, 200)!=HAL_OK){
			read_adc_error = true;
			return 0;
		}
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		if(HAL_I2C_Master_Transmit(&hi2c1,address_ADS1115<<1 ,0X00,1, 100)!=HAL_OK){
			read_adc_error =true;
			return 0;
		}
		HAL_I2C_DeInit(&hi2c1);
		HAL_Delay(10);
		for(uint8_t i = 0;i<10;i++){

			HAL_I2C_Init(&hi2c1);
			if(HAL_I2C_Master_Receive(&hi2c1,address_ADS1115<<1,(uint8_t*)ads1115_read, 2, 200)!= HAL_OK){
				read_adc_error = true;
				return 0;
			}
			HAL_I2C_DeInit(&hi2c1);
			HAL_Delay(5);
			ads1115_adc = (ads1115_adc + (uint16_t)((ads1115_read[0]<<8)|ads1115_read[1]))/2;
		}
		if(ads1115_adc < 3){
			ads1115_adc = 0;
		}
	}
	else if(portA_>3  && portA_<8){
		ads1115_cmd[1]=0xC3 + 0x10*(portA_-4);

		HAL_I2C_Init(&hi2c1);
		if(HAL_I2C_Master_Transmit(&hi2c1,address_ADS1115a<<1 ,(uint8_t*)ads1115_cmd, 3, 200)!=HAL_OK){
			read_adc_error = true;
			return 0;
		}
		HAL_I2C_DeInit(&hi2c1);

		HAL_I2C_Init(&hi2c1);
		if(HAL_I2C_Master_Transmit(&hi2c1,address_ADS1115a<<1 ,0X00,1, 100)!=HAL_OK){
			read_adc_error =true;
			return 0;
		}
		HAL_I2C_DeInit(&hi2c1);

		HAL_Delay(10);
		for(uint8_t i = 0;i<10;i++){

			HAL_I2C_Init(&hi2c1);
			if(HAL_I2C_Master_Receive(&hi2c1,address_ADS1115a<<1,(uint8_t*)ads1115_read, 2, 200)!= HAL_OK){
				read_adc_error = true;
				return 0;
			}
			HAL_I2C_Init(&hi2c1);
			HAL_Delay(5);
			ads1115_adc = (ads1115_adc + (uint16_t)((ads1115_read[0]<<8)|ads1115_read[1]))/2;
		}
		if(ads1115_adc < 0){
			ads1115_adc = 0;
		}

	}
	return ads1115_adc;
}

static void get_data_adc()
{
	uint16_t adc_read = 0;
	uint8_t address[8] = {2,3,0,1,6,7,4,5};
	adc_read = adc_ads1115(address[count_adc]);
	if(read_adc_error == false){
		data_adc[count_adc]=adc_read;
	}
	count_adc++;
	if(count_adc>=8)
	{
		count_adc =0;
	}
}

static void read_adc(uint8_t* input, uint8_t count_adc)
{
	uint16_t adc_value;
	int j=0;
	for(int i=0;i<input[2];i++)
	{
		adc_value = data_adc[i];
		rx_buffer[3+j] = i;
		rx_buffer[4+j] = (adc_value >> 8) & 0xff;// data MSB
		rx_buffer[5+j] = adc_value & 0xff; // data LSB
		j+=3;
	}
	for(uint8_t i =0;i<2;i++)
		rx_buffer[i] = input[i];
	rx_buffer[2] = input[2]*3; //length
	tx_crc(rx_buffer[2]+3);

}




/*****************************RS485******************************************/
static void read_RS485(uint8_t* cmd,uint8_t size)
{
	uint8_t command_tx[cmd[2]-1+2]; // sub by number of Byte to transmit to sensor
	uint8_t c=0; // to count number array transmit
	uint8_t command_rx[cmd[size-4]]; //array to recevie from sensor
	for(int i=0;i<size;i++)
	{
		if(i>=3 && i<cmd[2]-1+3)
		{
			command_tx[c]=cmd[i];
			c++;
		}
	}
	uint16_t crc_modbus;
	crc_modbus = CRC16_Modbus(command_tx,c);
	for(int i=0;i<2;i++)
		command_tx[c+i]=(uint8_t)((crc_modbus & (0xFF00>>(8*i))) >> (8-(8*i)));
	uint16_t temp;
	temp = command_tx[c];
	command_tx[c] = command_tx[c+1];
	command_tx[c+1] = temp;
	//humi,temp.
	HAL_GPIO_WritePin(Tx_Rx_EN_GPIO_Port,Tx_Rx_EN_Pin, GPIO_PIN_SET);
//	HAL_UART_Transmit_DMA(&huart2, command_tx, c+2);
	HAL_UART_Transmit(&huart2, command_tx, c+2,300);
	HAL_GPIO_WritePin(Tx_Rx_EN_GPIO_Port, Tx_Rx_EN_Pin, GPIO_PIN_RESET);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartReciver_rs485, 30);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
#ifdef DMA
		flag_rs485 = 1;
		rx_buffer[2] = cmd[size-4]-2;//length
		rx_buffer[3] = cmd[size-3];//cmd_ID -> RS485 Address (to indicating type which sever want to read)

#endif
#ifdef POLLING
	HAL_GPIO_WritePin(Tx_Rx_EN_GPIO_Port, Tx_Rx_EN_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive(&huart2, command_rx, cmd[size-4],300);
	crc_modbus = CRC16_Modbus((uint8_t*)command_rx,cmd[size-4]);
	if(crc_modbus==0)
	{
       rx_buffer[0] = ID;
       rx_buffer[1] = 8; // cmd
       rx_buffer[2] = cmd[size-4]-2;//length
       int k;
       for(k=0;k<rx_buffer[2];k++)
       {
    	   if(k==0)
    		   rx_buffer[k+3] = cmd[size-3];//cmd_ID -> RS485 Address (to indicating type which sever want to read)
    	   else
    		   rx_buffer[k+3] = command_rx[k];
       }
       tx_crc(rx_buffer[2]+3);
       for( int i=0;i<30;i++)
    	   rx_buffer[i] =0;
	}
#endif
}


static void rs485(uint8_t* payload,uint8_t size)
{
   	flag_rs485 = 0;
   	flag_rs485_rx= 0;
	uint16_t crc_modbus;
	crc_modbus = CRC16_Modbus((uint8_t*)payload,size);
	if(crc_modbus==0)
	{
       rx_buffer[0] = ID;
       rx_buffer[1] = 8; // cmd
//       rx_buffer[2] = cmd[size_cmd-4]-2;//length
       int k;
       for(k=1;k<rx_buffer[2];k++)
    	   rx_buffer[k+3] = payload[k];
       tx_crc(rx_buffer[2]+3);
       for( int i=0;i<30;i++)
    	   rx_buffer[i] =0;
	}
}
// size_change = true sizeData++
static void store_data_cmd(uint8_t* data, uint8_t* source, uint8_t lenght, uint8_t position, uint8_t size_change){
	memset(data,0,50);
	memcpy(data,source,lenght);
	cmd_ID[position] = source[lenght-3];
	countDATA[position] = lenght;
	Timer[position] = ((source[lenght-2] << 8) | source[lenght-1]) ;
	real_time[position] = HAL_GetTick();
	if(size_change){
		sizeDATA++;
	}

}
void store_data_cmd_toggle(uint8_t* data, uint8_t* source, uint8_t lenght, uint8_t position, uint8_t size_change){
	memset(data,0,50);
	memcpy(data,source,lenght);
	cmd_ID_toggle[position] = source[lenght];
	if(size_change){
		sizeDATA_toggle++;
	}

}
/********************************Set Timer to get data******************************************/
static void write_to_STM(uint8_t* input,uint8_t count)
{
	uint8_t data_store_buff[30] = {0};
	uint8_t data_store_lenght;
	data_store_buff[0] = input[0];
	data_store_buff[1] = input[3];
	memcpy(debug,input,count);
	switch(input[3]){
		case 11:{
			data_store_buff[2] = input[2] -  2;
			for(int i = 0 ; i < data_store_buff[2] ; i++){
				data_store_buff[i+3] =  input[i+4];
			}
			data_store_buff[data_store_buff[2]+3] = input[input[2] + 2];
			data_store_lenght = data_store_buff[2] + 4;
			if(sizeDATA_toggle == 0){
				store_data_cmd_toggle(&DATA_toggle[0][0],&data_store_buff[2],data_store_lenght-1,0,true);
			}
			else{
				uint8_t cmd_toggle_id;
				for(int i = 0; i<=sizeDATA_toggle ;i++){
					cmd_toggle_id = DATA_toggle[i][DATA_toggle[i][0]+1];
					if((data_store_buff[data_store_lenght-1] == cmd_toggle_id) && (i != sizeDATA_toggle)){
						store_data_cmd_toggle(&DATA_toggle[i][0],&data_store_buff[2],data_store_lenght-1,i,false);
						break;
					}
					else if(i >= sizeDATA_toggle){
						store_data_cmd_toggle(&DATA_toggle[i][0],&data_store_buff[2],data_store_lenght-1,i,true);
						break;
					}
				}
			}
			for(int i=0;i<2;i++)
				rx_buffer[i] = input[i];

			rx_buffer[2]=1;
			rx_buffer[3] = data_store_buff[data_store_lenght-1];
			tx_crc(4);

		}
		break;
		default:{
			data_store_buff[2] = input[2] -  4;
			for(int i = 0 ; i < data_store_buff[2] ; i++){
				data_store_buff[i+3] =  input[i+4];
			}
			for(int i=0;i<3;i++){
				data_store_buff[data_store_buff[2]+i+3] = input[input[2] + i];
			}
			data_store_lenght = data_store_buff[2] + 6;

			if(sizeDATA == 0){
					store_data_cmd(&DATA[0][0],data_store_buff,data_store_lenght,0,true);
			}
			else{
				for(int i = 0; i<=sizeDATA ;i++){
					if((data_store_buff[data_store_lenght-3] == cmd_ID[i]) && (i != sizeDATA)){
						store_data_cmd(&DATA[i][0],data_store_buff,data_store_lenght,i,false);
						break;
					}
					else if(i >= sizeDATA){
						store_data_cmd(&DATA[i][0],data_store_buff,data_store_lenght,i,true);
						break;
					}
				}
			}

			for(int i=0;i<2;i++)
					rx_buffer[i] = input[i];

			rx_buffer[2]=1;
			rx_buffer[3] = data_store_buff[data_store_lenght-3];
			tx_crc(4);


		}
		break;

	}

}


/********************************Transmit to LORA******************************************/
 void tx_crc(uint8_t c1_CRC)
{
	uint16_t CRC_txdata = CRC16((uint8_t*)rx_buffer,c1_CRC);
	rx_buffer[c1_CRC]= (CRC_txdata&0xFF);
	rx_buffer[c1_CRC+1]= ((CRC_txdata>>8)&0xFF);
	uartSend_DMA(rx_buffer, c1_CRC+2);
	memset(rx_buffer,0,50);
//	HAL_UART_Transmit(&huart1,rx_buffer,c1_CRC+2,200);
}

 void uartSend_DMA(uint8_t* Pdata, uint8_t size){
	uint8_t uart_error;

	uartSend_Store_Count[uartSend_Store_Size] = size;
	memcpy(&uartSend_Store[uartSend_Store_Size][0],Pdata,size);
	uartSend_Store_Size++;

	if(uartSend_Store_Size >= MAX_SIZE_UARTSEND){
		uartSend_Store_Size = MAX_SIZE_UARTSEND - 1;
	}

	if(uartSend_Store_Size <= 1){

		uart_error = HAL_UART_Transmit_DMA(&huart1, &uartSend_Store[0][0], uartSend_Store_Count[0]);

		if((uart_error == HAL_BUSY) || (uart_error == HAL_ERROR)){
			uartSend_Store_Count[0] = size;
			memcpy(&uartSend_Store[0][0],Pdata,size);
			uartSend_Store_Size++;

			if(uartSend_Store_Size >= MAX_SIZE_UARTSEND){
				uartSend_Store_Size = MAX_SIZE_UARTSEND-1;
			}

			flag_send_uart =1;
			time_send_uart = HAL_GetTick();
		}
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
#ifdef IWG
	HAL_IWDG_Refresh(&hiwdg);
#endif
	if(huart->Instance == USART1){

		memset(&uartSend_Store[0][0],0,uartSend_Store_Count[0]);
		uartSend_Store_Count[0] = 0;

		if(uartSend_Store_Size > 0){
			uartSend_Store_Size--;
		}

		if(uartSend_Store_Size > 0){

			for(int i = 0 ; i < uartSend_Store_Size ; i++){
				memcpy(&uartSend_Store[i][0],&uartSend_Store[i+1][0],uartSend_Store_Count[i+1]);
				uartSend_Store_Count[i] = uartSend_Store_Count[i+1];
			}
			uint8_t uart_error;
			uart_error = HAL_UART_Transmit_DMA(&huart1, &uartSend_Store[0][0], uartSend_Store_Count[0]);
			if((uart_error == HAL_BUSY) || (uart_error == HAL_ERROR)){
				flag_send_uart =1;
				time_send_uart = HAL_GetTick();
			}

		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartReciver_buffer, UARTRX_BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

	}
#ifdef DMA
	if(huart->Instance == USART2){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartReciver_rs485, 30);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
	}
#endif
}
#ifdef IWG
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 2999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
#endif
