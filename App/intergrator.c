#include<stdint.h>
#include "stm32f4xx.h"
#include "../Drivers/hal_gpio_driver.h"
#include "../Drivers/hal_spi_driver.h"
#include "../Drivers/hal_i2c_driver.h"
#include "../Drivers/hal_uart_driver.h"
#include "../Drivers/imu.h"
#include "../Drivers/servo.h"
#include "../Drivers/hal_millis.h"

#include "../App/spi_main.h"
#include "../App/uart_main.h"
#include "../App/i2c_main.h"

#include "../App/led.h"
#include "../Drivers/uart_debug.h"
#include <stdio.h>
#include <stdlib.h> //free
#include <math.h>
//Safer asprintf macro
#define Sasprintf(write_to, ...) { \
		char *tmp_string_for_extend = (write_to); \
		asprintf(&(write_to), __VA_ARGS__); \
		free(tmp_string_for_extend); \
}

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
uart_handle_t uart_handle1,uart_handle2,uart_handle3,uart_handle6, debug_handle;

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
#define SLAVE_OWN_ADDRESS      (uint8_t)0x68 //mpu9520 x68 without Ado
//#define SLAVE_ADDRESS_READ    (uint8_t) 0xA7
//#define SLAVE_ADDRESS_WRITE    (uint8_t) 0xA6

#define GENERAL_CALL_ADDRESS    (uint8_t)0x00

//#define MASTER_WRITE_CMD       0xC1
//#define MASTER_READ_CMD        0XC2

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


/* project variables */

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
	{/*uart1*/
		/*enable the clock for the GPIO port A */
	_HAL_RCC_GPIOA_CLK_ENABLE();  
	
	/*configure the GPIO_PORT_A_PIN_9 as TX */
	uart_pin_conf.pin = USART1_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOA,USART1_TX_PIN,USART1_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);

	/*configure the GPIO_PORT_A_PIN_10 as RX */
	uart_pin_conf.pin = USART1_RX_PIN;
	hal_gpio_set_alt_function(GPIOA,USART1_RX_PIN,USART1_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);
	}
	{/*uart2*/
		/*enable the clock for the GPIO port D */  
	_HAL_RCC_GPIOD_CLK_ENABLE();
	/*configure the GPIO_PORT_D_PIN_5 as TX */
	uart_pin_conf.pin = USART2_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOD,USART2_TX_PIN,USART2_TX_AF);
	hal_gpio_init(GPIOD ,&uart_pin_conf);

	/*configure the GPIO_PORT_D_PIN_6 as RX */
	uart_pin_conf.pin = USART2_RX_PIN;
	hal_gpio_set_alt_function(GPIOD,USART2_RX_PIN,USART2_TX_AF);
	hal_gpio_init(GPIOD ,&uart_pin_conf);
	}
	{/*uart3*/
	/*enable the clock for the GPIO port D */  
	_HAL_RCC_GPIOD_CLK_ENABLE();
	/*configure the GPIO_PORT_D_PIN_8 as TX */
	uart_pin_conf.pin = USART3_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOD,USART3_TX_PIN,USART3_TX_AF);
	hal_gpio_init(GPIOD ,&uart_pin_conf);

	/*configure the GPIO_PORT_D_PIN_9 as RX */
	uart_pin_conf.pin = USART3_RX_PIN;
	hal_gpio_set_alt_function(GPIOD,USART3_RX_PIN,USART3_TX_AF);
	hal_gpio_init(GPIOD ,&uart_pin_conf);
	}
	{/*uart6*/
	/*enable the clock for the GPIO port D */  
	_HAL_RCC_GPIOD_CLK_ENABLE();
	/*configure the GPIO_PORT_G_PIN_14 as TX */
	uart_pin_conf.pin = USART6_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOG,USART6_TX_PIN,USART6_TX_AF);
	hal_gpio_init(GPIOG ,&uart_pin_conf);

	/*configure the GPIO_PORT_G_PIN_9 as RX */
	uart_pin_conf.pin = USART6_RX_PIN;
	hal_gpio_set_alt_function(GPIOG,USART6_RX_PIN,USART6_TX_AF);
	hal_gpio_init(GPIOG ,&uart_pin_conf);
	}

}
void pin_init(){
	// Pin definitions 
	//TODO: GPIO init
	//int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
	int adoPin = 8; //TODO: after defining TIMER PWM, SPI? anypin closer or easier to map
	int myLed = 13;
/*TODO: GPIO init*/
	
	//Servo myservo; //download the servo library and inspect how it works and use it in C code later
	const int esc_sig_pin = SERVO_PIN; // PA5 TIMER2
	
//	pinMode(intPin, INPUT);
//  digitalWrite(intPin, LOW);
//  pinMode(adoPin, OUTPUT);
//  digitalWrite(adoPin, HIGH);
//  pinMode(myLed, OUTPUT);
//  digitalWrite(myLed, HIGH);
}

/////////////////////////////////////other functions before main/////////////////////
/* some delay generation */
//void delay_gen(uint32_t cnt)
//{
//	//uint32_t cnt = 800000;
//	while(cnt--);
//}
/**
IMU init function
**/
void imu_init(i2c_handle_t *handle, ImuState_t *imu_state)
{
	
	imu_state->GyroMeasError = (float)PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
	imu_state->GyroMeasDrift = (float)PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	// There is a tradeoff in the beta parameter between accuracy and response speed.
	// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
	// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
	// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
	// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
	// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
	// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
	// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

	imu_state->beta = sqrt(3.0f / 4.0f) * imu_state->GyroMeasError;   // compute beta
	imu_state->zeta = sqrt(3.0f / 4.0f) * imu_state->GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	//import math library to calculate
	
	imu_state->gscale =  GFS_250DPS;
	imu_state->ascale = AFS_2G;
	imu_state->mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	imu_state->mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	for(int i = 0; i < 3; i++){
		imu_state->magCalibration[i]= 0.f;
		imu_state->magBias[i]= 0.f;
		imu_state->magScale[i]= 0.f;
		imu_state->accelBias[i]= 0.f;
		imu_state->gyroBias[i]= 0.f;
		imu_state->q[i+1] =0.0f;
		imu_state->eInt[i] =0.0f;
	}
	imu_state->delt_t = 0; // used to control display output rate
	imu_state->prevMeasure = 0, imu_state->sumCount = 0; // used to control display output rate
	imu_state->deltat = 0.0f, imu_state->sum = 0.0f; // integration interval for both filter schemes
	imu_state->lastUpdate = 0, imu_state->firstUpdate = 0; // used to calculate integration interval
	imu_state->Now = 0;
	/*
   IMU calculation parameters
	*/
	imu_state->q[0] =1.0f;
	
	/* IMU identification, check and calibration */
	
	uint8_t c;
	
  c = readBytes(handle, MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if (SerialDebug) {
		char *temp = NULL;
		Sasprintf(temp, "MPU9250\nI AM %x  I should be %x", c, 0x71); 
		uart_printf(temp);
		free(temp);
  }
  delay(20);
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    if (SerialDebug) {
      uart_printf("MPU9250 is online, now self-testing\n");
    }
    MPU9250SelfTest(handle, imu_state->SelfTest); // Start by performing self test and reporting values
    if (SerialDebug) {
			char *temp = NULL;
			for(int i = 0; i< 6; i++)
			{
				Sasprintf(temp, "x-axis self test: acceleration trim within : %3.1f%% of factory value\n", imu_state->SelfTest[i]); 
			}
			uart_printf(temp);
			free(temp);
    }
    delay(10);
    getAres(imu_state);
    getGres(imu_state);
    getMres(imu_state);
    calibrateMPU9250(handle, imu_state->gyroBias, imu_state->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    if (SerialDebug) {
			char *temp = NULL;
			Sasprintf(temp, " MPU9250 calibrated and its bias\n x   y   z  \n %i %i %i mg\n%3.3f %3.3f %3.3fo/s\n", (int)(1000 * imu_state->accelBias[0]),(int)(1000 * imu_state->accelBias[1]),(int)(1000 * imu_state->accelBias[2]),imu_state->gyroBias[0],imu_state->gyroBias[1],imu_state->gyroBias[2]);
			uart_printf(temp);
			free(temp);
    }
    delay(10);
    initMPU9250(handle, imu_state);
    if (SerialDebug) {
      uart_printf("MPU9250 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    uint8_t d = readByte(handle, AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    if (SerialDebug) {
			char *temp = NULL;
			Sasprintf(temp, "AK8963 \nI AM %x  I should be %x", d, 0x48); 
			uart_printf(temp);
			free(temp);
    }
    delay(10);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(handle, imu_state, imu_state->magCalibration);
		if (SerialDebug) {
     uart_printf("AK8963 initialized for active data mode....\n"); // Initialize device for active mode read of magnetometer
    }
    {
      //magcalMPU9250(magBias, magScale);
      float magbias[3] = {0, 0, 0};
      magbias[0] = 54.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
      magbias[1] = 280.;  // User environmental x-axis correction in milliGauss
      magbias[2] = -448.;  // User environmental x-axis correction in milliGauss
			
      imu_state->magBias[0] = (float) magbias[0] * imu_state->mRes * imu_state->magCalibration[0]; // save mag biases in G for main program
      imu_state->magBias[1] = (float) magbias[1] * imu_state->mRes * imu_state->magCalibration[1];
      imu_state->magBias[2] = (float) magbias[2] * imu_state->mRes * imu_state->magCalibration[2];

      // Get soft iron correction estimate hardcoded now but it can be monitored and corrected when new soft iron is introduced.
      imu_state->magScale[0] = 0.92;
      imu_state->magScale[1] = 1.03;
      imu_state->magScale[2] = 1.05;
    }
    if (SerialDebug) {
			uart_printf("Calibration values: \n");
			char *temp = NULL;
			Sasprintf(temp, "X-Axis sensitivity adjustment value %3.2f\nY-Axis sensitivity adjustment value %3.2f\nZ-Axis sensitivity adjustment value %3.2f\n", imu_state->magCalibration[0], imu_state->magCalibration[1], imu_state->magCalibration[2]); 
			uart_printf(temp);
			free(temp);
    }
    delay(5);
  }
  else
  {
    if (SerialDebug) {
		char *temp = NULL;
		Sasprintf(temp, "Could not connect to MPU9250: 0x%x", c); 
		uart_printf(temp);
		free(temp);
    }
    while (1) ; // Loop forever if communication doesn't happen
  }
	/* IMU identification, check and calibration */
	
}

//hang on here, if applicaton can not proceed due to error
	
void assert_error(void)
{
	while(1)
	{
	  led_toggle(GPIOD,LED_RED);
		delay(5);
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
/*error handler uart1*/
void error_handler(void)
{
	 while(uart_handle1.tx_state != HAL_UART_STATE_READY );
	 hal_uart_tx(&uart_handle1,message2, sizeof(message2)-1);
	
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
				
				hal_uart_tx(&uart_handle1,message3, sizeof(message3)-1);
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
				
				hal_uart_tx(&uart_handle1,message3, sizeof(message3)-1);
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
	uint16_t ultrasonic_sensor_address[SENSORNUM] = {34, 24}; //secsor collections};//

	uint32_t val;
	/* operation variables */
		
	/* operation variables */
	/* IMU */
	ImuState_t imu_state;
	SensorData sensorData;
	/* IMU */
	/* Servo */
	uint32_t mid_point = (expSetting.pwmLimit - expSetting.arm) / 2 + expSetting.arm; //TODO is it possible to put in the structure?
	//int angular_speed = 0; //initial angular speed
	//int limit = 132; // anticlockwise PWM upper speed limit for the ESC
	//int arm = 53; //clockwise PWM starting point with highest speed
	//int speed_offset = 20; // limit to 5 hz maximum
	//int reading_time = 35; // waiting time for a reading in ms( milli seconds) 6.5 per metre
	//const int readings = 72; // how many times reading in an angular speed 72 constant for 2 times of statistical basis.
	//int waiting_time = 100 / expSetting.measurmentFrequency * expSetting.requiredSampleNumber; //overall time staying in an angular speed
	States_t state;
	state = NONE;
	motorStates motorStatus = STOP;
	/* Servo */
	/* Clock init */
	//SystemInit();
	/* Clock init */
	/* initialising and configures GPIOs for peripherals */
	
	spi_gpio_init();
	i2c_gpio_init();
	uart_gpio_init();
	/* pin init for servo and */
	pin_init();
	
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
	/* IMU init */
	SysTick_Init();
	imu_init(&i2c_handle, &imu_state);
	/* USART set up */
	/*enable the clock for the USART2 Peripheral */
	_HAL_RCC_USART2_CLK_ENABLE();   
	
	uart_handle2.Instance          = USART_2;

	uart_handle2.Init.BaudRate     = USART_BAUD_9600;
	uart_handle2.Init.WordLength   = USART_WL_1S8B;
	uart_handle2.Init.StopBits     = UART_STOPBITS_1;
	uart_handle2.Init.Parity       = UART_PARITY_NONE;
	uart_handle2.Init.Mode         = UART_MODE_TX_RX;
	uart_handle2.Init.OverSampling = USART_OVER16_ENABLE;

	_HAL_RCC_USART1_CLK_ENABLE();   
	
	uart_handle1.Instance          = USART_1;

	uart_handle1.Init.BaudRate     = USART_BAUD_9600;
	uart_handle1.Init.WordLength   = USART_WL_1S8B;
	uart_handle1.Init.StopBits     = UART_STOPBITS_1;
	uart_handle1.Init.Parity       = UART_PARITY_NONE;
	uart_handle1.Init.Mode         = UART_MODE_TX_RX;
	uart_handle1.Init.OverSampling = USART_OVER16_ENABLE;

	_HAL_RCC_USART3_CLK_ENABLE();   
	
	uart_handle3.Instance          = USART_3;

	uart_handle3.Init.BaudRate     = USART_BAUD_9600;
	uart_handle3.Init.WordLength   = USART_WL_1S8B;
	uart_handle3.Init.StopBits     = UART_STOPBITS_1;
	uart_handle3.Init.Parity       = UART_PARITY_NONE;
	uart_handle3.Init.Mode         = UART_MODE_TX_RX;
	uart_handle3.Init.OverSampling = USART_OVER16_ENABLE;

	_HAL_RCC_USART6_CLK_ENABLE();   
	
	uart_handle6.Instance          = USART_6;

	uart_handle6.Init.BaudRate     = USART_BAUD_9600;
	uart_handle6.Init.WordLength   = USART_WL_1S8B;
	uart_handle6.Init.StopBits     = UART_STOPBITS_1;
	uart_handle6.Init.Parity       = UART_PARITY_NONE;
	uart_handle6.Init.Mode         = UART_MODE_TX_RX;
	uart_handle6.Init.OverSampling = USART_OVER16_ENABLE;

/*fill out the application callbacks for UART*/
	uart_handle1.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle1.rx_cmp_cb = app_rx_cmp_callback;
	
	hal_uart_init(&uart_handle1);

	uart_handle2.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle2.rx_cmp_cb = app_rx_cmp_callback;
	
	hal_uart_init(&uart_handle2);

	uart_handle3.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle3.rx_cmp_cb = app_rx_cmp_callback;
	
	hal_uart_init(&uart_handle3);

	uart_handle6.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle6.rx_cmp_cb = app_rx_cmp_callback;
	
	hal_uart_init(&uart_handle6);

/*enable the IRQ of USART2 peripheral */
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(USART6_IRQn);

  while(uart_handle1.tx_state != HAL_UART_STATE_READY );
	/*Send the message */
	//uint8_t message1[] = "STM32F4xx Discovery board \n UART Sample App test\n June , 2016 \n";
	hal_uart_tx(&uart_handle1,message1, sizeof(message1)-1);

	
/* First initilaize the Debug UART */
	hal_debug_uart_init(DEBUG_USART_BAUD_9600);
	
	//uart_printf("SPI master Application Running ... \n");
	/** set up ready and waiting for user input -- from I2C
	TODO: Application will start in a condition this can be removed.
	**/

	uart_printf("set up finished\n");
/* set up ready and waiting for user input */
/* Wait for user Button press before starting the communication. Toggles LED_ORANGE until then */
  while(TestReady != SET)
  {
    led_toggle(GPIOD,LED_ORANGE);
		//LED3 (orange)
    delay(5);
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
		sampleIMUtoSensor(&i2c_handle, &imu_state, &sensorData);
		/*//Servo motor run
		//TODO implement PWM servo driver
		uint32_t requiredTime = expSetting.requiredSampleTime * 1000;
		uint32_t experiment_time_loop = 0;
		//uint32_t start_experiment_time =  millis();
		//check pause or restart - timer needed
		//for (int i = 0; i < SENSORNUM; i++) {
			//expLoopStatus.waitingTime[i] = expLoopStatus.waitingDelay; //initialising timer of each module
			//measurements[i].sensorID = i;
		//}
		
		expLoopStatus.pwmSpeed += expSetting.rotationSpeedIncrement;
		sampleIMU(&i2c_handle, &imu_state);
		//for continous I2C and SPI
		// asking mpu9520 with an interval and talk to spi if needed.*/
		/* USART block */
		//TODO : EXT1 or EXT0 pull down check the lin
		// if the signal function is set to low then -> interrupt to get this instead of while
		
		
		//check again 
		while(uart_handle1.rx_state != HAL_UART_STATE_READY );
		/*receive the message */
		hal_uart_rx(&uart_handle1,rx_buffer, 5 );
		
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
		delay(5);
		
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
		delay(5);

	//	read from slave

		/* Master READ command */
		addrcmd[0] = (uint8_t) CMD_MASTER_READ;
		addrcmd[1] = (uint8_t) ( CMD_MASTER_READ >> 8 );
		
		/* first send the master write cmd to slave */
		hal_spi_master_tx(&SpiHandle, addrcmd,CMD_LENGTH );
		
		/* application can block here, or can do other task untill above tx finishes */
		while(SpiHandle.State != HAL_SPI_STATE_READY );
		
		/* this dealy helps for the slave to be ready with the ACK bytes */
		delay(5);
		
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
				
		delay(5);
		
//		/* I2C block - interval with a timmer and measure it*/
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		/* first send the master write cmd to slave */
//		master_write_req = MASTER_WRITE_CMD;
//		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		
//		master_write_req = WRITE_LEN;
//		/* Now send the number of bytes to be written */
//		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
//		
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		
//		/* NOW send the data stream */
//		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,master_tx_buffer,WRITE_LEN);
//		
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		

//		
//		/* first send the master read cmd to slave */
//		master_read_req = MASTER_READ_CMD;
//		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_read_req,1);
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		
//		master_read_req = READ_LEN;
//		/* Now send the number of bytes to be read */
//		hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_read_req,1);
//		
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		
//		memset(master_rx_buffer,0, 5);
//		/* NOW read the data stream */
//		hal_i2c_master_rx(&i2c_handle,SLAVE_ADDRESS_READ,master_rx_buffer,READ_LEN);
//		while(i2c_handle.State != HAL_I2C_STATE_READY);
//		
//		if ( Buffercmp(slave_tx_buffer,master_rx_buffer,READ_LEN))
//		{
//			led_turn_on(GPIOD,LED_RED);
//		}else
//			led_toggle(GPIOD,LED_BLUE);
		
		//led_turn_on(GPIOD,LED_ORANGE);
		//TODO: instead of delay make this work with a timer
		//delay(5);
		
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
void USART1_IRQHandler(void)
{
  hal_uart_handle_interrupt(&uart_handle1);
}
void USART2_IRQHandler(void)
{
  hal_uart_handle_interrupt(&uart_handle2);
}
void USART3_IRQHandler(void)
{
  hal_uart_handle_interrupt(&uart_handle3);
}
void USART6_IRQHandler(void)
{
  hal_uart_handle_interrupt(&uart_handle6);
}

/* SysTick Exception handler */
void SysTick_Handler(void)
{
		ticker++;
		led_toggle(GPIOD,LED_ORANGE);
}
