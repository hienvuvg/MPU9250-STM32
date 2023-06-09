/*
 * MPU9250.c
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */

#include "mpu9250.h"
#include "main.h"

// AK8963 Magnetometer
// MPU9250 is upgraded from MPU6515

static uint8_t dest_temp[1];
static uint8_t _buffer[21];
uint8_t _mag_adjust[3];

#ifdef MPU9250_SPI

__weak void MPU9250_OnActivate()
{
}

static inline void MPU9250_Activate()
{
	MPU9250_OnActivate();
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK) {return -1;}
	else {}

	return receivedbyte;
}

void MPU_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MPU9250_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01){
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU9250_Deactivate();
}

void MPU_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MPU9250_Activate();
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	MPU9250_Deactivate();
}
#endif

#ifdef MPU9250_I2C

void MPU_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead){
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_I2C_Master_Transmit(&MPU9250_I2C,MPU9250_I2C_ADDR,&data,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&MPU9250_I2C,MPU9250_I2C_ADDR,pBuffer,NumByteToRead,HAL_MAX_DELAY);
}

void MPU_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite){
	HAL_I2C_Mem_Write(&MPU9250_I2C,MPU9250_I2C_ADDR,WriteAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumByteToWrite,HAL_MAX_DELAY);
}

#endif

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
	readRegister(subAddress);
	MPU_Write(&data, subAddress, 1);
	HAL_Delay(10); // This is necessary
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	MPU_Read(dest, subAddress, count);
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// store the data for write
	writeRegister(I2C_SLV0_DO,data);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t whoAmI(){
	// read the WHO AM I register
	readRegisters(WHO_AM_I,1,_buffer);

	// return the register value
	return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int whoAmIAK8963(){
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I,1,_buffer);
	// return the register value
	return _buffer[0];
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMNT_1,PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI();
	if((who != 0x71) &&( who != 0x73))
	{
		return 1;
	}

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2,SEN_ENABLE);

	// setting accel range to 16G as default
	writeRegister(ACCEL_CONFIG, ACCEL_RANGE);

	// setting the gyro range to 2000DPS as default
	writeRegister(GYRO_CONFIG, GYRO_RANGE);

	// setting bandwidth to 184Hz as default
	writeRegister(ACCEL_CONFIG2,DLPF_184);

	// setting gyro bandwidth to 184Hz
	writeRegister(CONFIG,DLPF_184);

	// setting the sample rate divider to 0 as default
	writeRegister(SMPDIV,0x00);

	// enable I2C master mode
	writeRegister(USER_CTRL,I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL,I2C_MST_CLK);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 0x48 )
	{
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL,7,_buffer);

	// successful init, return 0
	return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
	writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
	writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	writeRegister(ACCEL_CONFIG2,bandwidth);
	writeRegister(CONFIG,bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(SMPDIV,19);

	if(srd > 9)
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL,7,_buffer);

	}
	else
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL,7,_buffer);
	}

	writeRegister(SMPDIV, srd);
}

/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(int16_t* GyroData, int16_t* AccData, int16_t* MagData, int16_t* Temperature)
{
	// grab the data from the MPU9250
	readRegisters(ACCEL_OUT, 21, _buffer);

	// combine into 16 bit values
	AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];

	Temperature[0] = (((int16_t)_buffer[6]) << 8) | _buffer[7];

	GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];

	MagData[0] = (((int16_t)_buffer[15]) << 8) | _buffer[14];
	MagData[1] = (((int16_t)_buffer[17]) << 8) | _buffer[16];
	MagData[2] = (((int16_t)_buffer[19]) << 8) | _buffer[18];

	/* Original
	MagData[0] = (int16_t)((float)magx * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
	MagData[1] = (int16_t)((float)magy * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
	MagData[2] = (int16_t)((float)magz * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f));
	*/

	/* Example
	_magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	 */
}

// ***********************  My Program *************************//

uint8_t readRegister(uint8_t subAddress){
	MPU_Read(dest_temp, subAddress, 1);
	return dest_temp[0];
}

uint8_t MPU6515_Init(void){
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMNT_1,PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI();
	if((who != 0x71) &&( who != 0x73))
	{
		return 1;
	}

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2,SEN_ENABLE);

	// setting accel range to 16G as default
	writeRegister(ACCEL_CONFIG, ACCEL_RANGE);

	// setting the gyro range to 2000DPS as default
	writeRegister(GYRO_CONFIG, GYRO_RANGE);

	// setting bandwidth to 184Hz as default
	writeRegister(ACCEL_CONFIG2,DLPF_184);

	// setting gyro bandwidth to 184Hz
	writeRegister(CONFIG,DLPF_184);

	// setting the sample rate divider to 0 as default
	writeRegister(SMPDIV,0x00);

	// enable I2C master mode
	writeRegister(USER_CTRL,I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL,I2C_MST_CLK);

	return 0;
}

uint8_t AK8963_Init(void){
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 0x48 )
	{
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL,7,_buffer);

	// successful init, return 0
	return 0;
}

/* writes a byte to MPU9250 register given a register address and data */
// Action:
// 	0: Reset bit(s)
//  1: Set bit(s)
void MyWriteRegister(uint8_t subAddress, uint8_t in_data, uint8_t action){
	uint8_t RegValue = readRegister(subAddress);
	uint8_t data;

	if (action) data = RegValue | in_data; // Set
	else data = RegValue & ~in_data; // Reset

	MPU_Write(&data, subAddress, 1); // Set
	HAL_Delay(10); // This is necessary
}
