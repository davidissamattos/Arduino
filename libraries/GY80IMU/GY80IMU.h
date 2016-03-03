#ifndef GY80IMU_h
#define GY80IMU_h

#include "Arduino.h"
#include "Wire.h"


//I2C Address definitions for IMU
#define gyroscope_address 0x69
#define acelerometer_address 0x53
#define compass_address 0x1E
#define barometer_address 0x77

//////////////// ACCELEROMETER ///////////////////////////////////
//Register configuration address for the compass
#define compass_CRA 0x00
#define compass_CRB 0x01
#define compass_CRMode 0x02
#define compass_XMSB 0x03
#define compass_XLSB 0x04
#define compass_ZMSB 0x05
#define compass_ZLSB 0x06
#define compass_YMSB 0x07
#define compass_YLSB 0x08
#define ErrorCode_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ErrorCode_1_Num 1

//////////////// ACCELEROMETER ///////////////////////////////////
//Register configuration for the accelerometer
/* ------- Register names ------- */
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0x0F // 00001111
#define ADXL345_BW_800  0x0E // 00001110
#define ADXL345_BW_400  0x0D // 00001101  
#define ADXL345_BW_200  0x0C // 00001100
#define ADXL345_BW_100  0x0B // 00001011  
#define ADXL345_BW_50   0x0A // 00001010 
#define ADXL345_BW_25   0x09 // 00001001 
#define ADXL345_BW_12   0x08 // 00001000 
#define ADXL345_BW_6    0x07 // 00000111
#define ADXL345_BW_3    0x06 // 00000110

//Interrupt pin
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

//Interrupt bit position
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

//Flags
#define ADXL345_OK    1 // no error
#define ADXL345_ERROR 0 // indicates error is predent

#define ADXL345_NO_ERROR   0 // initial state
#define ADXL345_READ_ERROR 1 // problem reading accel
#define ADXL345_BAD_ARG    2 // bad method argument




//////////////// GYROSCOPE ///////////////////////////////////
#define L3G4200D_WHO_AM_I  		0x0F
#define L3G4200D_CTRL_REG1 		0x20
#define L3G4200D_CTRL_REG2 		0x21
#define L3G4200D_CTRL_REG3 		0x22
#define L3G4200D_CTRL_REG4 		0x23
#define L3G4200D_CTRL_REG5 		0x24
#define L3G4200D_REFERENCE 		0x25
#define L3G4200D_OUT_TEMP  		0x26
#define L3G4200D_STATUS_REG		0x27
#define L3G4200D_OUT_X_L 		0x28
#define L3G4200D_OUT_X_H 		0x29
#define L3G4200D_OUT_Y_L 		0x2A
#define L3G4200D_OUT_Y_H 		0x2B
#define L3G4200D_OUT_Z_L 		0x2C
#define L3G4200D_OUT_Z_H 		0x2D
#define L3G4200D_FIFO_CTRL_REG  0x2E
#define L3G4200D_FIFO_SRC_REG   0x2F
#define L3G4200D_INT1_CFG 		0x30
#define L3G4200D_INT1_SRC 		0x31
#define L3G4200D_INT1_TSH_XH 	0x32
#define L3G4200D_INT1_TSH_XL 	0x33
#define L3G4200D_INT1_TSH_YH 	0x34
#define L3G4200D_INT1_TSH_YL 	0x35
#define L3G4200D_INT1_TSH_ZH 	0x36
#define L3G4200D_INT1_TSH_ZL 	0x37
#define L3G4200D_INT1_DURATION 	0x38





//////////////// Data types and structure ///////////////////////////////////
struct compassData
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct gyroData
{
	double XAxis;
	double YAxis;
	double ZAxis;
};

struct accelData
{
	double XAxis;
	double YAxis;
	double ZAxis;
};

struct baroData
{
	float Altitude;
};



//////////////// Class definitions ///////////////////////////////////
class GY80IMU
{
	  /***************************************************/
	  /*	Public										 */
	  /***************************************************/
	public:
	  //Constructor
	  GY80IMU();
	  	  
//////COMP public functions
	  void configCOMP(int option);
	  compassData readCOMP();
	  int setScaleCOMP(float gauss);//valores: 0.88 1.3 1.9 2.5 4 4.7 5.6 8.1
	  int compass_gain;
	    
//////ACCEL public functions
	  bool status;  // set when error occurs
	  byte error_code;       // Initial state 
	  accelData readACCEL();
	  void configACCEL();
	  void setAxisGainsACCEL(float gainX, float gainY, float gainZ);
	  void setAxisOffsetACCEL(int x, int y, int z);
	  void set_BWACCEL(char bw_code);

//////GYRO public functions
	  void configGYRO(int scale);
	  gyroData readGYRO();
	  byte readRegisterGYRO(byte address);
	  void setGainsGYRO(double g1, double g2, double g3);

//////BARO public functions
	  //void configBARO(int option);
	  
	  

	  /***************************************************/
	  /*	Private										 */
	  /***************************************************/

	private:
///////Compass private functions and data
	  float m_Scale; // escala da bussola
	  int configModeCOMP; //modo de configuracao da bussola
	  uint8_t regValue; //registrado da bussola
	  
	  
///////gyro private
	  void writeToGYRO(byte address, byte val);	  	  
	  double gyroGains[3];
	  double dps2rads;
	  
///////accelerometer private
	  double accelGains[3];        // counts to Gs
	  uint8_t accelbuff[6]; //buffer de leitura do accel
	  int accelnumbytes;
	  void powerOnACCEL();
	  void writeToACCEL(byte address, byte val);
	  void readFromACCEL(byte address, int num, byte accelbuff[]);
	  void getAxisGainsACCEL(double *_gains);
	  void getAxisOffsetACCEL(int* x, int* y, int*z); 

//////BARO private functions
	   	  
};
#endif