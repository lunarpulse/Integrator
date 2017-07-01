#include<stdint.h>
#include "stm32f4xx.h"
#include "../Drivers/hal_gpio_driver.h"
#include "../Drivers/hal_spi_driver.h"
#include "../Drivers/hal_i2c_driver.h"
#include "../Drivers/hal_uart_driver.h"

#include "../App/spi_main.h"
#include "../App/uart_main.h"
#include "../App/i2c_main.h"

#include "../App/led.h"
#include "../Drivers/uart_debug.h"

/** Connection diagram 
		
								
				IMU on board --
											 |
											 V													USART or through PPM encoder
				Receptor  ->  Integrator  -> communicator -------> Flight controller
								 USART							SPI
																	-> programmer with different jack
				
**/

/* SPI handle for our SPI device 
	SPI is required to talk to the communicator and also programming
*/

spi_handle_t SpiHandle;
i2c_handle_t i2c_handle;
uart_handle_t uart_handle, debug_handle;

int TestReady = 0;


/* slave will reply this data, when master issues read command */
uint8_t slave_reply_data[4]={ 0x55, 0xaa, 0x55, 0xaa};

/* master read/write buffers */
uint8_t master_write_data[]={ 0xa, 0xb, 0xc, 0xd };

uint8_t master_read_buffer[4];


/* I2C handle for our SPI device 
	I2C is required to talk to the communicator and also programming
*/
#define I2C_MASTER_MODE_EN //master mode on if this line is active

//TODO: need to substiitute the address of slave for mpu9520
#define SLAVE_OWN_ADDRESS      (uint8_t)0x53;
#define SLAVE_ADDRESS_READ    (uint8_t) 0xA7
#define SLAVE_ADDRESS_WRITE    (uint8_t) 0xA6

#define GENERAL_CALL_ADDRESS    (uint8_t)0x00

#define MASTER_WRITE_CMD       0xC1
#define MASTER_READ_CMD        0XC2

#define READ_LEN    5
#define WRITE_LEN   5

//SPI or I2C Payloads
uint8_t master_tx_buffer[5]={0xa5, 0x55, 0xa5, 0x55, 0xb0};
uint8_t master_rx_buffer[5];

uint8_t slave_tx_buffer[5]={0xa5, 0x55, 0xa5, 0x55, 0xc0};
uint8_t slave_rx_buffer[5];

uint8_t master_write_req;
uint8_t master_read_req;

uint8_t slave_rcv_cmd;

//USART messages
uint8_t message1[] = "STM32F4xx Discovery board \n UART Sample App test\n June , 2016 \n";
uint8_t message2[] = "Invalid Command !!! \n";
uint8_t message3[] = "Success !! \n";
uint8_t rx_buffer[4];


/* I2C extern functions finished */
extern void  hal_i2c_enable_peripheral(I2C_TypeDef *i2cx);
extern  void hal_gpio_driver_set_alternate_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value);
extern void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
extern void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
extern void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
/* I2C extern functions finished */

/* I2C configuration */


///////////////////////////////////Initialisation Functions////////////////////////////////
/* configure gpio for spi functionality */
void spi_gpio_init(void)
{
	gpio_pin_conf_t spi_conf;
	
	
	_HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* configure GPIOB_PIN_13 for SPI CLK functionality */ 
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOB,SPI_CLK_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* configure GPIOB_PIN_14 for SPI MISO functionality */ 
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MISO_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* configure GPIOB_PIN_15 for SPI MOSI functionality */ 
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MOSI_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
}
/* configure gpio for I2C functionality */
void i2c_gpio_init()
{
		gpio_pin_conf_t i2c_scl, i2c_sda;
	
	 _HAL_RCC_GPIOB_CLK_ENABLE();

	
	i2c_scl.pin = I2C1_SCL_LINE;
	i2c_scl.mode = GPIO_PIN_ALT_FUN_MODE;
	i2c_scl.op_type = GPIO_PIN_OP_TYPE_OPEN_DRAIN;
	i2c_scl.pull = GPIO_PIN_PULL_UP;
	i2c_scl.speed = GPIO_PIN_SPEED_HIGH;
	
	hal_gpio_set_alt_function(GPIOB,I2C1_SCL_LINE,GPIO_PIN_AF4_I2C123);
	
	hal_gpio_init(GPIOB, &i2c_scl);
	
 
	
	
	i2c_sda.pin = I2C1_SDA_LINE;
	i2c_sda.mode = GPIO_PIN_ALT_FUN_MODE;
	i2c_sda.op_type = GPIO_PIN_OP_TYPE_OPEN_DRAIN;
	i2c_sda.pull = GPIO_PIN_PULL_UP;
	i2c_sda.speed = GPIO_PIN_SPEED_HIGH;
	
	hal_gpio_set_alt_function(GPIOB,I2C1_SDA_LINE,GPIO_PIN_AF4_I2C123);
	hal_gpio_init(GPIOB, &i2c_sda);

	
}
/* configure gpio for USART functionality */

void uart_gpio_init(void)
{
	gpio_pin_conf_t uart_pin_conf;
	
/*enable the clock for the GPIO port A */
	_HAL_RCC_GPIOA_CLK_ENABLE();  
	
/*configure the GPIO_PORT_A_PIN_2 as TX */
	uart_pin_conf.pin = USARTx_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOA,USARTx_TX_PIN,USARTx_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);

	/*configure the GPIO_PORT_A_PIN_3 as RX */
	uart_pin_conf.pin = USARTx_RX_PIN;
	hal_gpio_set_alt_function(GPIOA,USARTx_RX_PIN,USARTx_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);

}


/////////////////////////////////////other functions before main/////////////////////
/* some delay generation */
void delay_gen( )
{
	uint32_t cnt = 800000;
	while(cnt--);
}

//hang on here, if applicaton can not proceed due to error
	
void assert_error(void)
{
	while(1)
	{
	  led_toggle(GPIOD,LED_RED);
		delay_gen();
	}
}

/* function used to compare two buffers */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
	return 0;
}
/**
	from I2C to clear interrupts
**/
void gpio_btn_interrupt_handler(void)
{
	hal_gpio_clear_interrupt(0);

	TestReady = SET;
	
}
void pass_function(void)
{
	
}
/**
	from USART
**/

void error_handler(void)
{
	 while(uart_handle.tx_state != HAL_UART_STATE_READY );
	 hal_uart_tx(&uart_handle,message2, sizeof(message2)-1);
	
}

void handle_cmd(int cmd, int led )
{
	if(cmd == 'H')
			{
				if(led == (int) 0xff )
				{
					led_turn_on(GPIOD,LED_ORANGE);
			 	 led_turn_on(GPIOD,LED_BLUE);
				 led_turn_on(GPIOD,LED_GREEN);
				 led_turn_on(GPIOD,LED_RED);
					
				}else
				{
				led_turn_on(GPIOD,led);
				}
				
				hal_uart_tx(&uart_handle,message3, sizeof(message3)-1);
			}else if (cmd == 'L')
			{
				if(led == (int) 0xff )
				{
					led_turn_off(GPIOD,LED_ORANGE);
			 	 led_turn_off(GPIOD,LED_BLUE);
				 led_turn_off(GPIOD,LED_GREEN);
				 led_turn_off(GPIOD,LED_RED);
					
				}else
				led_turn_off(GPIOD,led);
				
				hal_uart_tx(&uart_handle,message3, sizeof(message3)-1);
			} 
			else
			{
				error_handler();
			}
	
}

// this function parses the command and takes action 
void 	parse_cmd(uint8_t *cmd)
{
	
	if( cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'D' )
	{
		if(cmd[3] == 'O' )
		{
			handle_cmd(cmd[4],LED_ORANGE);
			
		}else if(cmd[3] == 'B' )
		{
			handle_cmd(cmd[4],LED_BLUE);
			
		}else if(cmd[3] == 'G' )
		{
				handle_cmd(cmd[4],LED_GREEN);
			
		}else if(cmd[3] == 'R' )
		{
	    	handle_cmd(cmd[4],LED_RED);
		}else if (cmd[3] == 'A' )
		{
			handle_cmd(cmd[4],0xff);
		}
		else 
		{
			;
		}
		
		
		
	}else
	{
		error_handler();
		
	}
	
}

/*This callback will be called by the driver when driver finishes the transmission of data */
void app_tx_cmp_callback(void *size)
{
 //nothing to send to the ultrasonic devices at the moment
	
}

/*This callback will be called by the driver when the application receives the command */
void app_rx_cmp_callback(void *size)
{
	//we got a command,  parse it
	// integrate the data from the ultrasonic devices
	// data done EKF from the slaves?
	
	// Sample IMU and record the data from it to fuse it
	parse_cmd(rx_buffer);
	
}
////////////////////////////Main function//////////////////////////////////
int main(void)
{
	uint32_t i=0;
	uint8_t addrcmd[CMD_LENGTH];
	uint8_t ack_buf[2];
	
	uint32_t val;
	/* initialising and configures GPIOs for peripherals */

	spi_gpio_init();
	i2c_gpio_init();
	uart_gpio_init();
	/* To use LED */
	led_init();
	
	/* Configure USER Button interrupt*/
	//TODO -- not needed, replace with timer or timing request from I2C slaves
	hal_gpio_configure_interrupt(0, INT_FALLING_EDGE, gpio_btn_interrupt_handler);

	_HAL_RCC_GPIOA_CLK_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE, pass_function);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);

	/* SPI setup */
	/* enable the clock for the SPI2 */
	_HAL_RCC_SPI2_CLK_ENABLE() ;
	
	/*fill up the handle structure */
	SpiHandle.Instance               = SPI_2;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE;
	SpiHandle.Init.Mode              = SPI_MASTER_MODE_SEL;
	
	SpiHandle.State = HAL_SPI_STATE_READY;
	
	
	/* Call driver API to initialize the SPI device */
	hal_spi_init(&SpiHandle);
	
		/* Enable the IRQs in the NVIC */
  NVIC_EnableIRQ(SPI2_IRQn);

	/* I2C set up */
	_HAL_RCC_I2C1_CLK_ENABLE() ;
	i2c_handle.Instance = I2C_1;
	i2c_handle.Init.ack_enable = I2C_ACK_ENABLE;
	i2c_handle.Init.AddressingMode = I2C_ADDRMODE_7BIT; 
	i2c_handle.Init.ClockSpeed = 100000;
	i2c_handle.Init.DutyCycle = I2C_FM_DUTY_2; //care needs to taken 
	i2c_handle.Init.GeneralCallMode = 0;
	i2c_handle.Init.NoStretchMode = I2C_ENABLE_CLK_STRETCH;
	i2c_handle.Init.OwnAddress1 = SLAVE_OWN_ADDRESS;
	
  NVIC_EnableIRQ(I2Cx_ER_IRQn);
  NVIC_EnableIRQ(I2Cx_EV_IRQn);
	
	hal_i2c_init(&i2c_handle);
  hal_i2c_enable_peripheral(i2c_handle.Instance);
	
	hal_gpio_enable_interrupt(0,EXTI0_IRQn);
	
	//val = i2c_handle.Instance->CR1;
	i2c_handle.State = HAL_I2C_STATE_READY;
	
	/* USART set up */
	/*enable the clock for the USART2 Peripheral */
	_HAL_RCC_USART2_CLK_ENABLE();   
	
	uart_handle.Instance          = USART_2;

	uart_handle.Init.BaudRate     = USART_BAUD_9600;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = UART_STOPBITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX_RX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;

/*fill out the application callbacks for UART*/
	uart_handle.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle.rx_cmp_cb = app_rx_cmp_callback;
	
	 hal_uart_init(&uart_handle);

/*enable the IRQ of USART2 peripheral */
	 NVIC_EnableIRQ(USARTx_IRQn);

  while(uart_handle.tx_state != HAL_UART_STATE_READY );
	/*Send the message */
	//uint8_t message1[] = "STM32F4xx Discovery board \n UART Sample App test\n June , 2016 \n";
	hal_uart_tx(&uart_handle,message1, sizeof(message1)-1);

	
/* First initilaize the Debug UART */
	hal_debug_uart_init( DEBUG_USART_BAUD_9600);
	
	uart_printf("SPI master Application Running ... \n");
	/** set up ready and waiting for user input -- from I2C
	TODO: Application will start in a condition this can be removed.
	**/

/* set up ready and waiting for user input */
/* Wait for user Button press before starting the communication. Toggles LED_ORANGE until then */
  while(TestReady != SET)
  {
    led_toggle(GPIOD,LED_ORANGE);
		//LED3 (orange)
    delay_gen();
  }
	
	 led_turn_off(GPIOD,LED_ORANGE);
	
/******************************************************************************/
/*                                                                            */
/*                         Integrator recieves data                           */
/*                         and integrate them to perform collision avoidance  */
/*                         Write and read commands                            */
/******************************************************************************/

while(1)
{	
	//for continous I2C and SPI
	// asking mpu9520 with an interval and talk to spi if needed.
	/* USART block */
	//TODO : EXT1 or EXT0 pull down check the lin
	// if the signal function is set to low then -> interrupt to get this instead of while
	
	
	//check again 
	while(uart_handle.rx_state != HAL_UART_STATE_READY );
	/*receive the message */
	hal_uart_rx(&uart_handle,rx_buffer, 5 );
	
	/* SPI block */
	//check for state ready 
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command */
	addrcmd[0] = (uint8_t) CMD_MASTER_WRITE;
	addrcmd[1] = (uint8_t) ( CMD_MASTER_WRITE >> 8 );
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,CMD_LENGTH );
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	
	/* read back the ACK bytes from the slave */
	hal_spi_master_rx(&SpiHandle,ack_buf, ACK_LEN);
	
	/* wait untill ACK reception finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* did we rcv the valid ACK from slave ?? */
	if(ack_buf[1] == 0XE5 && ack_buf[0] == 0xD5 )
	{
		//correct ack 
		led_toggle(GPIOD,LED_GREEN);
		memset(ack_buf,0,(unsigned)2);
	}
	else
	{
		//invalide ack 
		assert_error();
		memset(ack_buf,0,2);
	}

	/* NOW send the data stream */
	hal_spi_master_tx(&SpiHandle, master_write_data,DATA_LENGTH );
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	delay_gen();

//	read from slave

	/* Master READ command */
	addrcmd[0] = (uint8_t) CMD_MASTER_READ;
	addrcmd[1] = (uint8_t) ( CMD_MASTER_READ >> 8 );
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,CMD_LENGTH );
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	
	/* read back the ACK bytes from the slave */
	hal_spi_master_rx(&SpiHandle,ack_buf, ACK_LEN);
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	if(ack_buf[1] == 0XE5 && ack_buf[0] == 0xD5 )
	{
		//correct ack 
		led_toggle(GPIOD,LED_GREEN);
		memset(ack_buf,0,2);
	}
	else
	{
		//invalide ack 
		assert_error();
		memset(ack_buf,0,2);
	}
	
	/* start receiving from the slave */
	hal_spi_master_rx(&SpiHandle,master_read_buffer, DATA_LENGTH);
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* compare the data rcvd form slave, with what slave supposed to send */
	if(Buffercmp(master_read_buffer,slave_reply_data,DATA_LENGTH))
	{
		// we didnt rcv what needs to be rcvd !!! Error !
		led_toggle(GPIOD,LED_RED);
	}else
	{
		//Rcvd correct data 
		led_toggle(GPIOD,LED_BLUE);
		for(i=0;i<DATA_LENGTH;i++)
		uart_printf("Data Received from slave: %x\n",master_read_buffer[i]);
	}
			
	delay_gen();
	
	/* I2C block - interval with a timmer and measure it*/
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	/* first send the master write cmd to slave */
	master_write_req = MASTER_WRITE_CMD;
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	master_write_req = WRITE_LEN;
	/* Now send the number of bytes to be written */
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
	
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	/* NOW send the data stream */
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,master_tx_buffer,WRITE_LEN);
	
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	

	
	/* first send the master read cmd to slave */
	master_read_req = MASTER_READ_CMD;
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_read_req,1);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	master_read_req = READ_LEN;
	/* Now send the number of bytes to be read */
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_read_req,1);
	
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	memset(master_rx_buffer,0, 5);
	/* NOW read the data stream */
	hal_i2c_master_rx(&i2c_handle,SLAVE_ADDRESS_READ,master_rx_buffer,READ_LEN);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	if ( Buffercmp(slave_tx_buffer,master_rx_buffer,READ_LEN))
	{
		led_turn_on(GPIOD,LED_RED);
	}else
		led_toggle(GPIOD,LED_BLUE);
	
	//led_turn_on(GPIOD,LED_ORANGE);
	//TODO: instead of delay make this work with a timer
	//delay_gen();
	
	/* Uart section  */
	
	
}

	return 0;
}

/////////////////////SPI helper funcations and IRQ handlers///////////////////////

/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	/* call the driver api to process this interrupt */
  hal_spi_irq_handler(&SpiHandle);
}

/////////////////////I2C helper funcations and IRQ handlers///////////////////////

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if(  ((uint16_t)0x0001) == GPIO_Pin)
 {
	// BSP_LED_On(LED5);
   TestReady = SET;
	 
	 
 }
}

void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(& i2c_handle);
}


/**
  * @brief  This function handles I2C event interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C data transmission
  */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(& i2c_handle);
}

/////////////////////USARTx helper funcations and IRQ handlers///////////////////////
//This is the ISR for the USARTx interrupt , defined in the hal_uart_driver.h
void USARTx_IRQHandler(void)
{
  hal_uart_handle_interrupt(&uart_handle);
}

