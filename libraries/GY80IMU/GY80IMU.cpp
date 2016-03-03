/*****************************************************************************************
Essa biblioteca foi criada por DAVID ISSA MATTOS, apartir de um subconjunto de funcoes
de diversas outras bibliotecas já existentes.

A biblioteca do accelerometro foi desenvolvida por:
-   20100515 TJS Initial Creation 
-   20100524 TJS Modified to run with Kevin Stevenard's driver
Outras modificacoes foram baseadas em http://morf.lv/modules.php?name=tutorials&lasit=31


A biblioteca da bussola foi desenvolvida por:
- 	2011 Love Electronics (loveelectronics.co.uk)

A biblioteca do gyroscopio foi desenvolvida por:
-   10/05/14 Samwell Freeman.
-	Sparkfun electronics

A biblioteca do barometro foi desenvolvidada por:
-

Funcoes principais:

Configuracao dos sensores
- configGYRO
- configACCEL
- configBARO
- configCOMP

Leitura dos sensores
- readACCEL
- readCOMP
- readGYRO
- readBARO

Estruturas de dados:
- compassData
- gyroData
- accelData
- baroData
****************************************************************************************/



#include "Arduino.h"
#include "GY80IMU.h"

GY80IMU::GY80IMU()
{
	//Compass
	 regValue = B00100000;//Default
	 m_Scale = 1;
	 
	 
	//Accelerometer
     status = ADXL345_OK;
     error_code = ADXL345_NO_ERROR;
	 accelnumbytes = 6;
	 
	  dps2rads = 3.141592/180;
}


	  /***************************************************/
	  /*	COMPASS										 */
	  /***************************************************/
void GY80IMU::configCOMP(int option)
{
	//Default escale
	setScaleCOMP(1.3);
	/* Byte config
		datasheet bit configuration
	*/
	byte config;
	//Configura o registrador A
	Wire.beginTransmission(compass_address);
	Wire.write(compass_CRA);
		config = B01110000; //Media de 8 medidas a 15Hz sem bias
    Wire.write(config);
	Wire.endTransmission();
	//Configura o registrador B
	Wire.beginTransmission(compass_address);
	Wire.write(compass_CRB);
		config = regValue;//Configuracao default dos ganhos
    Wire.write(config);
	Wire.endTransmission();
	switch(option)
	{
		//Single Mode
		case 1:
			//Configura o registrador de modo
	    	Wire.beginTransmission(compass_address);
	    	Wire.write(compass_CRMode);
				config = B00000001; //Configuracao do modo single mesurement
		    Wire.write(config);
			Wire.endTransmission();
			delay(6);
		break;
		//Continuous Mode
		case 2:
			//Configura o registrador de modo
	    	Wire.beginTransmission(compass_address);
	    	Wire.write(compass_CRMode);
				config = B00000000; //Configuracao do modo continuous mesurement
		    Wire.write(config);
			Wire.endTransmission();
			delay(6);
		break;
	}
	configModeCOMP=option;
}
compassData GY80IMU::readCOMP()
{
	int length = 6;
	
	
	//Preparing for a Single Measure Mode
    if (configModeCOMP==1)//SIngle Measure Mode
	{
		Wire.beginTransmission(compass_address);
		Wire.write(compass_CRMode);//Registrador de inicio da leitura 
		byte config = B00000001; //Configuracao do modo single mesurement
		Wire.write(config);
		Wire.endTransmission();
		delay(6);
		Wire.beginTransmission(compass_address);
	    Wire.write(compass_XMSB);//Registrador de inicio da leitura 
	    Wire.endTransmission();
	    Wire.beginTransmission(compass_address);
	    Wire.requestFrom(compass_address, length);
	}
	
    if (configModeCOMP==2) //Continuous Mode
	{
		Wire.beginTransmission(compass_address);
	    Wire.write(compass_XMSB);//Registrador de inicio da leitura 
	    Wire.endTransmission();
	    Wire.beginTransmission(compass_address);
	    Wire.requestFrom(compass_address, length);
		delay(67);//necessary delay for 15Hz
	}
	
	
	
	uint8_t buffer[length];
    if(Wire.available() == length)
    {
  	  for(uint8_t i = 0; i < length; i++)
  	  {
  		  buffer[i] = Wire.read();
  	  }
    }
    compassData data;
	compassData raw;
    raw.XAxis = (buffer[0] << 8) | buffer[1];
    raw.ZAxis = (buffer[2] << 8) | buffer[3];
    raw.YAxis = (buffer[4] << 8) | buffer[5];
    Wire.endTransmission();
	data.XAxis =raw.XAxis*m_Scale;
	data.YAxis =raw.YAxis*m_Scale;
	data.ZAxis =raw.ZAxis*m_Scale;
	return data;
}
int GY80IMU::setScaleCOMP(float gauss)
{
	regValue = 0x00;
	if(gauss == 0.88)
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(gauss == 1.3)
	{
		regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(gauss == 1.9)
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(gauss == 2.5)
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(gauss == 4.0)
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	else
	{
		return ErrorCode_1_Num;
	}
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	
}



	  /***************************************************/
	  /*	GYRO										 */
	  /***************************************************/

//Gyro functions
void GY80IMU::configGYRO(int scale)
{
	// Enable x, y, z and turn off power down:
	//Output data rate = 100Hz
	//Cut off  = 25 
	writeToGYRO(L3G4200D_CTRL_REG1,0b00011111);
	//Configure HPF (high pass filter)
	//Normal mode lendo o valor de reset
	//Cut off freq = 8 Hz com ODR=100Hz
	writeToGYRO(L3G4200D_CTRL_REG2,0b00000000);
	//Configure interrupts
	//Todas as interrupcoes estao desabilitadas
	writeToGYRO(L3G4200D_CTRL_REG3,0b00000000);
	//Configure full scale etc
	//Continuous mode
	//first address is LSB, second is MSB
	//No Self-test
    if(scale == 250)
	{
		writeToGYRO(L3G4200D_CTRL_REG4,0b00000000);
		setGainsGYRO(0.00875,0.00875,0.00875);
    }
	else if(scale == 500)
	{
		writeToGYRO(L3G4200D_CTRL_REG4,0b00010000);
		setGainsGYRO(0.01750,0.01750,0.01750);
    }
	else
	{
		writeToGYRO(L3G4200D_CTRL_REG4,0b00110000);
		setGainsGYRO(0.070,0.070,0.070);
    }
	//Configure High Pass Filtering outputs
	//Sem HPF
	//Sem selecao de interrupcao
	//Boot e o normal
	//FIFO desabilitada
	//Sem selecao de output
	writeToGYRO(L3G4200D_CTRL_REG5,0b00000000);
	
	
}

gyroData GY80IMU::readGYRO()
{
	gyroData data;
	//Sensitivity = 0.00875 dps/LSB
	data.XAxis=0; data.YAxis=0; data.ZAxis=0;
    int X_out, Y_out, Z_out;
	
	//Leitura de X
	byte X0 = readRegisterGYRO(L3G4200D_OUT_X_L);
	byte X1 = readRegisterGYRO(L3G4200D_OUT_X_H);
	int X11 = X1<<8;
	X_out = X0+X11;
	//Leitura de Y
	byte Y0 = readRegisterGYRO(L3G4200D_OUT_Y_L);
	byte Y1 = readRegisterGYRO(L3G4200D_OUT_Y_H);
	int Y11 = Y1<<8;
	Y_out = Y0+Y11;
	//Leitura de Z
	byte Z0 = readRegisterGYRO(L3G4200D_OUT_Z_L);
	byte Z1 = readRegisterGYRO(L3G4200D_OUT_Z_H);
	int Z11 = Z1<<8;
	Z_out = Z0+Z11;
	
	data.XAxis = X_out * gyroGains[0]*dps2rads;
    data.YAxis = Y_out * gyroGains[1]*dps2rads;
 	data.ZAxis = Z_out * gyroGains[2]*dps2rads; 
	 
	return data;   
}

void GY80IMU::writeToGYRO(byte address, byte val)
{
    Wire.beginTransmission(gyroscope_address); // start transmission to device 
    Wire.write(address);             // send register address
    Wire.write(val);                 // send value to write
    Wire.endTransmission();         // end transmission
}

byte GY80IMU::readRegisterGYRO(byte address)
{
	Wire.beginTransmission(gyroscope_address);
	Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(gyroscope_address,1); 
	while(!Wire.available());// waiting
	byte B = Wire.read();
	return B;
}

void GY80IMU::setGainsGYRO(double g1, double g2, double g3)
{
	gyroGains[0]=g1;
	gyroGains[1]=g2;
	gyroGains[2]=g3;
}

	  /***************************************************/
	  /*	ACCELEROMETER								 */
	  /***************************************************/
void GY80IMU::configACCEL()
{
	double gravity = 9.80665;
   //Ganhos para escala de +-2g
   // Ou seja temos 4g de escala/10bits -> ganho = 4g/2^10 = 1/256
    accelGains[0] = 0.00390625*gravity;
    accelGains[1] = 0.00390625*gravity;
    accelGains[2] = 0.00390625*gravity; 
	powerOnACCEL();
	//set_BWACCEL(ADXL345_BW_12);//bandwidth de 12.5
	writeToACCEL(ADXL345_DATA_FORMAT, 0x00); //reseta em modo de 10 bits +-2g 
}
void GY80IMU::powerOnACCEL()
{
    //Turning on the ADXL345
	// Sem o link/autosleep/sleep
    writeToACCEL(ADXL345_POWER_CTL, 8); 
}
accelData GY80IMU::readACCEL()
{
	accelData data;
	data.XAxis=0; data.YAxis=0; data.ZAxis=0;
    int X_out, Y_out, Z_out;
	//Lendo X
	Wire.beginTransmission(acelerometer_address);
	Wire.write(ADXL345_DATAX0);
    Wire.endTransmission();
	
	//Wire.beginTransmission(acelerometer_address);
    Wire.requestFrom(acelerometer_address,6); 
    if(Wire.available()<=6)   
    {
      byte X0 = Wire.read();
      byte X1 = Wire.read(); 
      byte Y0 = Wire.read();
      byte Y1 = Wire.read(); 
      byte Z0 = Wire.read();
      byte Z1 = Wire.read(); 
      int X11 = X1<<8;
      int Z11 = Z1<<8;
      int Y11 = Y1<<8;
	  
      X_out = X0+X11;
   	  Y_out = Y0+Y11;
      Z_out = Z0+Z11;
  	  }
	  //Wire.endTransmission();
	  data.XAxis = X_out * accelGains[0];
	  data.YAxis = Y_out * accelGains[1];
	  data.ZAxis = Z_out * accelGains[2]; 
	 
	 return data;   
}
//write by to address
void GY80IMU::writeToACCEL(byte address, byte val) 
{
  Wire.beginTransmission(acelerometer_address); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}
// Reads num bytes starting from address register on device in to accelbuff array
void GY80IMU::readFromACCEL(byte address, int num, byte accelbuff[])
{
  Wire.beginTransmission(acelerometer_address); // start transmission to device 
  for(char i=0;i<num;i++)
  {
  	Wire.write(address+i);             // sends address to read from
  }
  
  Wire.endTransmission();         // end transmission
  delay(6);
  Wire.beginTransmission(acelerometer_address); // start transmission to device
  Wire.requestFrom(acelerometer_address, accelnumbytes);    // request 6 bytes from device
 
  if(Wire.available() == accelnumbytes)
  {
	  for(uint8_t i = 0; i < accelnumbytes; i++)
	  {
		  accelbuff[i] = Wire.read();
	  }
  }
  //eliminando a verificacao de erro
  Wire.endTransmission();         // end transmission
}
void GY80IMU::setAxisGainsACCEL(float gainX, float gainY, float gainZ)
{
    accelGains[0] = gainX;
	accelGains[1] = gainY;
	accelGains[2] = gainZ;
}
void GY80IMU::getAxisGainsACCEL(double *_gains)
{
  int i;
  for(i = 0; i < 3; i++){
    _gains[i] = accelGains[i];
  }
}
void GY80IMU::setAxisOffsetACCEL(int x, int y, int z)
{
    writeToACCEL(ADXL345_OFSX, byte (x));  
    writeToACCEL(ADXL345_OFSY, byte (y));  
    writeToACCEL(ADXL345_OFSZ, byte (z));  
}
 // Gets the OFSX, OFSY and OFSZ bytes
void GY80IMU::getAxisOffsetACCEL(int* x, int* y, int*z) 
{
    byte _b;
    readFromACCEL(ADXL345_OFSX, 1, &_b);  
    *x = int (_b);
    readFromACCEL(ADXL345_OFSY, 1, &_b);  
    *y = int (_b);
    readFromACCEL(ADXL345_OFSZ, 1, &_b);  
    *z = int (_b);
}

void GY80IMU::set_BWACCEL(char bw_code)
{
    char bw = 0x00;//garantindo que o low power seja zero
	bw = bw + bw_code;
    writeToACCEL(ADXL345_BW_RATE, bw_code);
}

	  /***************************************************/
	  /*	BAROMETER   								 */
	  /***************************************************/

	
//Barometer functions	
//void GY80IMU::configBARO(int option)
//{}


