/***************************************************************************
  This is a library for the BMP085 pressure sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "Arduino.h"
#include <I2C-Master-Library/I2C.h>
#include <math.h>
#include "Adafruit_BMP085_U.h"

static bmp085_calib_data _bmp085_coeffs;   // Last read accelerometer data will be available here
static uint8_t           _bmp085Mode;

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

 /**************************************************************************/
 /*!
	 @brief  Writes an 8 bit value over I2C
 */
 /**************************************************************************/
uint8_t Adafruit_BMP085_Unified::writeCommand(uint8_t reg, uint8_t value)
{
	return _wire->write(BMP085_ADDRESS, reg, value);
}

/**************************************************************************/
/*!
	@brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_BMP085_Unified::read8(uint8_t reg, uint8_t *value)
{
	uint8_t returnStatus = _wire->read(BMP085_ADDRESS, reg, 1);
	if (returnStatus)
	{
		return returnStatus;
	}
	*value = _wire->receive();
	return 0;
}

/**************************************************************************/
/*!
	@brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_BMP085_Unified::read16(uint8_t reg, uint16_t *value)
{
	uint8_t returnStatus = _wire->read(BMP085_ADDRESS, reg, 2);
	if (returnStatus)
	{
		return returnStatus;
	}
	*value = ((uint16_t)_wire->receive() << 8) | _wire->receive();
	return 0;
}

/**************************************************************************/
/*!
	@brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_BMP085_Unified::readS16(uint8_t reg, int16_t *value)
{
	uint16_t i;
	uint8_t returnStatus = read16(reg, &i);
	if (returnStatus)
	{
		return returnStatus;
	}
	*value = (int16_t)i;
	return 0;
}

/**************************************************************************/
/*!
	@brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP085_Unified::readCoefficients(void)
{
	readS16(BMP085_REGISTER_CAL_AC1, &_bmp085_coeffs.ac1);
	readS16(BMP085_REGISTER_CAL_AC2, &_bmp085_coeffs.ac2);
	readS16(BMP085_REGISTER_CAL_AC3, &_bmp085_coeffs.ac3);
	read16(BMP085_REGISTER_CAL_AC4, &_bmp085_coeffs.ac4);
	read16(BMP085_REGISTER_CAL_AC5, &_bmp085_coeffs.ac5);
	read16(BMP085_REGISTER_CAL_AC6, &_bmp085_coeffs.ac6);
	readS16(BMP085_REGISTER_CAL_B1, &_bmp085_coeffs.b1);
	readS16(BMP085_REGISTER_CAL_B2, &_bmp085_coeffs.b2);
	readS16(BMP085_REGISTER_CAL_MB, &_bmp085_coeffs.mb);
	readS16(BMP085_REGISTER_CAL_MC, &_bmp085_coeffs.mc);
	readS16(BMP085_REGISTER_CAL_MD, &_bmp085_coeffs.md);
}

int8_t Adafruit_BMP085_Unified::isDataReady()
{
	if (readingTemperature)
	{
		if (micros() - readyStart > readyDelay)
		{
			// Temperature is ready
			if (!readTemperature(&lastTemperature))
			{
				return -1;
			}
			// Kick off the pressure reading
			if (requestPressure())
			{
				return -2;
			}
			return DATA_READY_TEMPERATURE;
		}
	}
	else if (readingPressure)
	{
		if (micros() - readyStart > readyDelay)
		{
			// Pressure is ready
			if (!readPressure(&lastPressure))
			{
				return -3;
			}
			// Kick off the temperature reading
			if (requestTemperature())
			{
				return -4;
			}
			return DATA_READY_PRESSURE;
		}
	}
	else
	{
		// We haven't started anything, so initialize
		if (requestTemperature())
		{
			return -4;
		}
	}
	// Nothing is ready
	return 0;
}

uint8_t Adafruit_BMP085_Unified::requestTemperature()
{
	uint8_t result = writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
	if (result)
	{
		return result;
	}
	readingTemperature = true;
	readyStart = micros();
	readyDelay = TEMPERATURE_READ_DELAY_MICROS;
	return 0;
}

uint8_t Adafruit_BMP085_Unified::requestPressure()
{
	uint8_t result = writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6));
	if (result)
	{
		return result;
	}
	readingPressure = true;
	readyStart = micros();
	switch (_bmp085Mode)
	{
	case BMP085_MODE_ULTRALOWPOWER:
		readyDelay = PRESSURE_ULTRALOW_READ_DELAY_MICROS;
		break;
	case BMP085_MODE_STANDARD:
		readyDelay = PRESSURE_STANDARD_READ_DELAY_MICROS;
		break;
	case BMP085_MODE_HIGHRES:
		readyDelay = PRESSURE_HIGH_READ_DELAY_MICROS;
		break;
	case BMP085_MODE_ULTRAHIGHRES:
	default:
		readyDelay = PRESSURE_ULTRAHIGH_READ_DELAY_MICROS;
		break;
	}

	return 0;
}

bool Adafruit_BMP085_Unified::readTemperature(uint16_t *temperature)
{
	bool result = !read16(BMP085_REGISTER_TEMPDATA, temperature);
	readingTemperature = false;
	readyTemperature = true;
	return result;
}

bool Adafruit_BMP085_Unified::readPressure(int32_t *pressure)
{
	uint16_t p16;
	if (read16(BMP085_REGISTER_PRESSUREDATA, &p16))
	{
		return false;
	}
	
	uint8_t p8;
	if (read8(BMP085_REGISTER_PRESSUREDATA + 2, &p8))
	{
		return false;
	}
	
	*pressure = ((uint32_t)p16 << 8 | p8) >> (8 - _bmp085Mode);
	readingPressure = false;
	readyPressure = true;
	return true;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::readRawTemperature(int32_t *temperature)
{
	uint16_t t;
	if (requestTemperature())
	{
		return false;
	}
	delayMicroseconds(readyDelay);
	if (!readTemperature(&t))
	{
		return false;
	}
	*temperature = t;
	return true;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::readRawPressure(int32_t *pressure)
{
	if (requestPressure())
	{
		return false;
	}
	// We can't simply use delayMicroseconds because it won't pause for longer
	// than 16383us, which is less than the duration required for ULTRAHIGHRES.
	delay(readyDelay / 1000);
	delayMicroseconds(readyDelay % 1000);
	if (!readPressure(pressure))
	{
		return false;
	}
	return true;
}

/**************************************************************************/
/*!
	@brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t Adafruit_BMP085_Unified::computeB5(int32_t ut) {
	int32_t X1 = (ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5) >> 15;
	int32_t X2 = ((int32_t)_bmp085_coeffs.mc << 11) / (X1 + (int32_t)_bmp085_coeffs.md);
	return X1 + X2;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

 /**************************************************************************/
 /*!
	 @brief  Setups the HW
 */
 /**************************************************************************/
bool Adafruit_BMP085_Unified::begin(bmp085_mode_t mode)
{
	// Enable I2C
	_wire->begin();

	/* Mode boundary check */
	if ((mode > BMP085_MODE_ULTRAHIGHRES) || (mode < 0))
	{
		mode = BMP085_MODE_ULTRAHIGHRES;
	}

	/* Make sure we have the right device */
	uint8_t id;
	read8(BMP085_REGISTER_CHIPID, &id);
	if (id != 0x55)
	{
		return false;
	}

	/* Set the mode indicator */
	_bmp085Mode = mode;

	/* Coefficients need to be read once */
	readCoefficients();

	return true;
}

/**************************************************************************/
/*!
	@brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::getPressure(float *pressure)
{
	int32_t  ut = 0, up = 0;

	/* Get the raw pressure and temperature values */
	if (readyPressure)
	{
		ut = lastTemperature;
		up = lastPressure;
	}
	else
	{
		if (!(readRawTemperature(&ut) && readRawPressure(&up)))
		{
			return false;
		}
		readyTemperature = false;
	}
	readyPressure = false;

	int32_t  x1, x2, b5, b6, x3, b3, p;
	uint32_t b4, b7;


	/* Temperature compensation */
	b5 = computeB5(ut);

	/* Pressure compensation */
	b6 = b5 - 4000;
	x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int32_t)_bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
	x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
	x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (_bmp085_coeffs.ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = ((uint32_t)(up - b3) * (50000 >> _bmp085Mode));

	if (b7 < 0x80000000)
	{
		p = (b7 << 1) / b4;
	}
	else
	{
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	int32_t compp = p + ((x1 + x2 + 3791) >> 4);

	/* Assign compensated pressure value */
	*pressure = compp / 100.0F;
	return true;
}

/**************************************************************************/
/*!
	@brief  Reads the temperatures in degrees Celsius
*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::getTemperature(float *temp)
{
	int32_t UT;

	if (readyTemperature)
	{
		UT = lastTemperature;
	}
	else
	{
		if (!readRawTemperature(&UT))
		{
			return false;
		}
	}
	readyTemperature = false;

	int32_t B5;
	float t;

	B5 = computeB5(UT);
	t = (B5 + 8) >> 4;
	t /= 10;

	*temp = t;
	return true;
}

/**************************************************************************/
/*!
	Calculates the altitude (in meters) from the specified atmospheric
	pressure (in hPa), and sea-level pressure (in hPa).

	@param  seaLevel      Sea-level pressure in hPa
	@param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::pressureToAltitude(float seaLevel, float atmospheric)
{
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude.  See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
	Calculates the pressure at sea level (in hPa) from the specified altitude
	(in meters), and atmospheric pressure (in hPa).

	@param  altitude      Altitude in meters
	@param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::seaLevelForAltitude(float altitude, float atmospheric)
{
	// Equation taken from BMP180 datasheet (page 17):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude.  See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}
