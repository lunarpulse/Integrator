#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <Wire.h>

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
  #define Y_FINE_GAIN      0x04
  #define Z_FINE_GAIN      0x05
  #define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC   0x07
  #define YA_OFFSET_H      0x08
  #define YA_OFFSET_L_TC   0x09
  #define ZA_OFFSET_H      0x0A
  #define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define MPU9250CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#define AHRS true         // set to false for basic data read


#define TIMEGAP 10


// structure to hold experimental sensor data from sensor nodes
typedef struct { //total 32 byte of
  word angularSpeed = 0x0000;  // 2 bytes (speed value : 2byte word 16bit pwm; number sample points wrod 0-63655| byte)
  word msgLength = 0x0000;    // 1-14 data

  word data0 = 0x0000;    // 1 data points in words
  word data1 = 0x0000;    // 1 data points in words
  word data2 = 0x0000;    // 1 data points in words
  word data3 = 0x0000;    // 1 data points in words
  word data4 = 0x0000;    // 1 data points in words
  word data5 = 0x0000;    // 1 data points in words
  word data6 = 0x0000;    // 1 data points in words
  word data7 = 0x0000;    // 1 data points in words
  word data8 = 0x0000;    // 1 data points in words
  word data9 = 0x0000;    // 1 data points in words
  word data10 = 0x0000;   // 1 data points in words
  word data11 = 0x0000;   // 1 data points in words
  word data12 = 0x0000;   // 1 data points in words
  word data13 = 0x0000;   // 1 data points in words
  // total 1 to 13 words of data
} ExperimentPayload;

// the possible states of the state-machine
typedef enum {  NONE, GOT_N, GOT_D, GOT_I, GOT_S, GOT_M, GOT_F, GOT_R, GOT_P, GOT_U } states;
//GOT_D can be a direction indicator
//MOTOR status enums
typedef enum {STOP, RESTART, RUN} motorStates; //how about CL, ACL ?

// structure to hold experimental configuration data from users
typedef struct {
  uint8_t numb_transmit_record = 3;
  unsigned int sensorNumber = SENSORNUM;
  unsigned int rotationSpeedIncrement = 1;   // I length of bootloader hex data (bytes)
  unsigned int measurmentFrequency = 20; //F
  unsigned int requiredSampleTime = 20; // required time each sampleing loop
  unsigned int requiredSampleNumber = 18; //M
  unsigned int rotationDirection = 1; //1 for clockwise 0 for anticlockwise;
  unsigned int pauseTime = 1; //1 for clockwise 2 for anticlockwise;
  unsigned int restartValue = 1; //1 for clockwise 2 for anticlockwise;
  unsigned int speed_offset = 20;
  unsigned int pwmLimit = 132;
  unsigned int arm = 53;
  float coefficient_yaw = 0.1;
  int sinage[SENSORNUM] = {1};
  unsigned int increment_waiting_time [SENSORNUM] = {2};
} ExperimentSetting;

typedef struct {
  unsigned int pwmSpeed = 90;   // I length of bootloader hex data (bytes)
  unsigned int sampleCounter = 0; //when came out of the measurement loop then store the counter here and catch up from where left off
  uint32_t waitingDelay = 30; //frequency delay between measuemrents
  bool completeMeasurementLoop = false;
  unsigned int prevRotation = 1;
  float sound_speed = 340.29;
  unsigned int repeatedErrorCount[SENSORNUM] = {0};
  unsigned int noErrorCount[SENSORNUM] = {0};
  uint32_t waitingTime[SENSORNUM] = {0};
  bool triggered[SENSORNUM] = {false};
  bool recieved[SENSORNUM] = {false};
} ExperimentStatus;

typedef struct {
  float xa = 0.0f; //accelerometer
  float ya = 0.0f; //no need of typecast from double 0.0 to float or int 0 to float
  float za = 0.0f; //no extra code by compilers.
  float xg = 0.0f; //gyroscope
  float yg = 0.0f;
  float zg = 0.0f;
  float xm = 0.0f; // magnetometor
  float ym = 0.0f;
  float zm = 0.0f;
  float temperature = 0.0f;
  float yaw = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float freq = 0.0f;
} GyroValue;

typedef struct {
  int16_t xa = 0;
  int16_t ya = 0;
  int16_t za = 0;
  int16_t xg = 0;
  int16_t yg = 0;
  int16_t zg = 0;
  int16_t xm = 0;
  int16_t ym = 0;
  int16_t zm = 0;
  int16_t yaw = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  int16_t temperature = 0;
  float freq = 0.0f;
} GyroOutput;

typedef struct {
  uint8_t sensorID;
  GyroValue gyroValue;
  uint16_t range = 0;
  bool error = false;
  float prev_time_elasped = 0;
  uint32_t timeStamp = 0;
  int16_t reflected_yaw = 0;
  int16_t reflected_pitch = 0;
  int16_t reflected_roll = 0;
} SensorData;

typedef struct {
  GyroOutput gyroOut;
  bool error = false;
  uint32_t timeStampB = 0;
  uint32_t timeStampE = 0;
} GyroData;

//mpu settings
// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};