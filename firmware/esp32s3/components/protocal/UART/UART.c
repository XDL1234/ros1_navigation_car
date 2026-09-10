#include "UART.h"

#define TAG "UART"
#define PRINT(window, fmt, args...) printf("{"#window"}"fmt"\n", ##args)

Ring_Buffer_t uart_ringbuf[2];//0-user,1-lidar
uint16_t uart0_rx_len = 0;


void uart_init(int baud,int UART_NUM,int TXD_PIN, int RXD_PIN,int RX_BUF_SIZE,int TX_BUF_SIZE) 
{    

    const uart_config_t uart_config = {   			//串口参数结构体
        .baud_rate = baud,               			//波特率
        .data_bits = UART_DATA_8_BITS,	 			//数据位设置为8位
        .parity = UART_PARITY_DISABLE,				//禁用奇偶校验
        .stop_bits = UART_STOP_BITS_1,				//停止位设置为1位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,		//流量控制模式禁用（CTS/RTS）
        .source_clk = UART_SCLK_APB,				//设置时钟源
    };
	

    if(uart_driver_install(							//安装串口驱动
    					UART_NUM,					//串口号
						RX_BUF_SIZE * 2,		 	//接收缓冲区
						TX_BUF_SIZE * 2,      			//发送缓冲区
						0,       					//串口事件队列大小
						NULL,   					//串口句柄
						0      					  	//用于分配中断的标志
						) == ESP_OK) printf("串口%d驱动安装 成功\n",UART_NUM);
    else printf("串口%d驱动安装 失败\n",UART_NUM);

    if(uart_param_config(         						//配置参数
    				  UART_NUM,						//串口号
					  &uart_config					//串口参数结构体传入
					  ) == ESP_OK)printf("串口%d参数配置 成功\n",UART_NUM);
    else printf("串口%d参数配置 失败\n",UART_NUM);

    if(uart_set_pin(         							//设置引脚
    			 UART_NUM,      					//串口号
    			 TXD_PIN, 							//发送引脚
				 RXD_PIN, 							//接收引脚
				 UART_PIN_NO_CHANGE,				//CTS
				 UART_PIN_NO_CHANGE 				//RTS
				 ) == ESP_OK) printf("串口%d引脚设置 成功\n",UART_NUM);
    else printf("串口%d引脚设置 失败\n",UART_NUM);

}


void uart1_send_imu(void* pvPara)
{
    uint8_t buffer[15];  // 数据帧缓冲区
    uint8_t sumcheck = 0; // 校验和变量
    uint8_t addcheck = 0; // 附加校验变量
    uint8_t index = 0;    // 缓冲区索引

	while(1)
	{
		sumcheck = 0;
        addcheck = 0;
        index = 0;
		// 将欧拉角转换为 int16，并放大100倍
		// int16_t roll_int = (int16_t)(proto_imu_data.euler[0] * 100.0f);
		// int16_t pitch_int = (int16_t)(proto_imu_data.euler[1] * 100.0f);
		// int16_t yaw_int = (int16_t)(proto_imu_data.euler[2] * 100.0f);
		uint8_t fusion_sta=0;
		// 帧头 (0xAB)
		buffer[index++] = 0xAB;
		// 源地址 (假设为 0xDC, 匿名飞控的默认地址)
		buffer[index++] = 0xDC;
		// 目标地址 (0xFE, 上位机地址)
		buffer[index++] = 0xFE;
		// 功能码 (ID: 0x03 表示飞控姿态：欧拉角格式)
		buffer[index++] = 0x03;
		// 数据长度 (7字节数据)
		buffer[index++] = 7;
		buffer[index++] = 0;  // 数据长度高字节为0

		// 欧拉角数据 (int16, 角度扩大100倍)
		// buffer[index++] = (uint8_t)(roll_int & 0xFF);
		// buffer[index++] = (uint8_t)((roll_int >> 8) & 0xFF);
		// buffer[index++] = (uint8_t)(pitch_int & 0xFF);
		// buffer[index++] = (uint8_t)((pitch_int >> 8) & 0xFF);
		// buffer[index++] = (uint8_t)(yaw_int & 0xFF);
		// buffer[index++] = (uint8_t)((yaw_int >> 8) & 0xFF);

		// 融合状态 (uint8)
		buffer[index++] = fusion_sta;

		// 计算校验和和附加校验 (从帧头开始到DATA区结束)
		for (int i = 0; i < index; i++)
		{
			sumcheck += buffer[i];
			addcheck += sumcheck;
		}

		// 添加校验和和附加校验值
		buffer[index++] = sumcheck;
		buffer[index++] = addcheck;

		// 发送数据帧
 		uart_write_bytes(UART_NUM_1, (const char*)buffer, index);

		vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 延时
	}
}

void uart0_send_motor(void* pvPara)
{
    context_pack_t *ctx = (context_pack_t*)pvPara;
	static uint8_t rx_buffer[256];
	static uint8_t *data_ptr = rx_buffer;  // 缓冲区指针
	bool frame_flag = false;
	float target = 0, kp = 0, ki = 0, kd = 0;
	
    while (1)
    {

        while(Uart_Available(USER_UART_NUM)) 
        {
            uint8_t data = Uart_Read(USER_UART_NUM);
            PRINT(debug, "data=%x\n",data);
            
            if(data == '<')  // 帧头
            {
                data_ptr = &rx_buffer[0];  // 重置指针到缓冲区开始
                *data_ptr++ = data;             // 存储帧头
                frame_flag = false;
            }
            else if(data == '>' )  // 帧尾且已接收数据
            {
                *data_ptr = data;  // 存储帧尾
                *(data_ptr + 1) = '\0';  // 添加字符串结束符
				frame_flag = true;
			}
			else
			{
				 // 存储数据
                *data_ptr++ = data;
                // 防止缓冲区溢出
                if(data_ptr >= &rx_buffer[sizeof(rx_buffer) - 1]) 
				{
                    data_ptr = rx_buffer;
                }
			}
                
        }  
		if(frame_flag)
		{
			const char* p = strstr((const char*)rx_buffer, "target:");
            if(p) target = atof(p + 7);
	
            p = strstr((const char*)rx_buffer, "P:");
            if(p) kp = atof(p + 2);

            p = strstr((const char*)rx_buffer, "I:");
            if(p) ki = atof(p + 2);
            
            p = strstr((const char*)rx_buffer, "D:");
            if(p) kd = atof(p + 2);

			// 更新PID参数和目标速度
            mc_set_expect_rpm(ctx->mc.right, target);
            mc_set_pid_params(ctx->mc.right, kp, ki, kd);
			PRINT(debug, "Update: Target=%.2f, P=%.2f, I=%.2f, D=%.2f\n",
            target, kp, ki, kd);
            frame_flag=false;
		}
        
		vTaskDelay(pdMS_TO_TICKS(10));
    }
	vTaskDelete(NULL);
}
	

// 串口0接收任务 Serial port 0 Receives tasks
void Uart0_Rx_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Uart0_Rx_Task with core:%d", xPortGetCoreID());
    uint16_t temp_len = 255;
    uint8_t* temp_data = (uint8_t*) malloc(temp_len);

    while (1)
    {
        // 从串口0读取数据，并将读取的数据缓存到ringbuffer里。
        const int rxBytes = uart_read_bytes(USER_UART_NUM, temp_data, temp_len, 1 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            for (int i = 0; i < rxBytes; i++)
            {
                RingBuffer_Push(&uart_ringbuf[USER_UART_NUM], temp_data[i]);
            }
        }
		
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(temp_data);
    vTaskDelete(NULL);
}

// 串口1接收任务 Serial port 1 Receives tasks
void Uart1_Rx_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Uart1_Rx_Task with core:%d", xPortGetCoreID());
    uint16_t temp_len = 255;
    uint8_t* temp_data = (uint8_t*) malloc(temp_len);

    while (1)
    {
        // 从串口0读取数据，并将读取的数据缓存到ringbuffer里。
        const int rxBytes = uart_read_bytes(LIDAR_UART_NUM, temp_data, temp_len, 1 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            for (int i = 0; i < rxBytes; i++)
            {
                RingBuffer_Push(&uart_ringbuf[LIDAR_UART_NUM], temp_data[i]);
            }
        }
		
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(temp_data);
    vTaskDelete(NULL);
}

void  uart_task(context_pack_t *ctx)
{
    //xTaskCreate(uart1_send_imu, "uart1_send_Task", 4096 , NULL, 5, NULL);
    //xTaskCreatePinnedToCore(Uart1_Rx_Task, "Uart1_Rx_Task", 5*1024, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(Uart0_Rx_Task, "Uart0_Rx_Task", 5*1024, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(uart0_send_motor, "Uart0_Tx_Task", 5*1024, ctx, 6, NULL,0);
}


bool UART_HARDWARE_init(void) 
{
	uart_init(baud_R,USER_UART_NUM,UART0_TXD_PIN,UART0_RXD_PIN,UART0_RX0_BUF_SIZE,0);  //调试串口
	//uart_init(baud_R,LIDAR_UART_NUM,UART1_TXD_PIN,UART1_RXD_PIN,UART1_RX0_BUF_SIZE,0);  //激光雷达串口
	RingBuffer_Init(&uart_ringbuf[USER_UART_NUM], UART0_RX0_BUF_SIZE);
    RingBuffer_Init(&uart_ringbuf[LIDAR_UART_NUM], UART1_RX0_BUF_SIZE);
    return true;
}

// 通过串口发送一串数据 Send a string of data through serial port 0
int Uart_Send_Data(int uart_num, uint8_t* data, uint16_t len)
{
    const int txBytes = uart_write_bytes(uart_num, data, len);
    return txBytes;
}

// 通过串口发送一个字节 Send a byte through serial port 0
int Uart_Send_Byte(int uart_num, uint8_t data)
{
    uint8_t data1 = data;
    const int txBytes = uart_write_bytes(uart_num, &data1, 1);
    return txBytes;
}

// 返回串口缓存数据的数量
// Return the amount of cached data in serial port 0
uint16_t Uart_Available(int uart_num)
{
    return RingBuffer_Get_Used_Count(&uart_ringbuf[uart_num]);
}

// 从串口缓存数据中读取一个字节
// Reads a byte from serial port 0 cache data
uint8_t Uart_Read(int uart_num)
{
    return RingBuffer_Pop(&uart_ringbuf[uart_num]);
}

// 清除串口的缓存数据
// Clear cache data from serial port 0
void Uart_Clean_Buffer(int uart_num)
{
    RingBuffer_Clean_Queue(&uart_ringbuf[uart_num]);
}









