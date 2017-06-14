#include <OneWire.h>

/*
 * Программа управления температурой водяного теплого пола
 *
 * Использует данные с датчика температуры ds18b20 закрепленного
 * на коллекторе (температура обратки) и управляет термоэлектрическим
 * (нормально открытым) сервоприводом через реле.
 * Также для контроля меряется температура подачи (второй датчик)
 *
 * ROM коды датчиков
 * 28 FF 2B 76 84 16 5 E4 - датчик на оранжевом проводе
 * 28 FF BA 79 80 16 3 47 - датчик на синем проводе
 */

#define	SENSORS_PIN		2
#define CONTROL_PIN		4

#define MAIN_SENS_ID	0xE4
#define ADD_SENS_ID		0x47

#define DEBUG_OUTPUT	1

OneWire ds(SENSORS_PIN);  // (a 4.7K resistor is necessary)

bool	valveOpen 	= true;
float	targetTemp	= 27.0;
float	tempHyst	= 0.5;
float 	outTemp, inTemp;

void setup(void) {
  Serial.begin(9600);

  // при включении сервопривод выключен
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, HIGH);
}

void loop(void) {

	byte	sensAddr[8];
	float	temp = 0.0;

	if (!readSensors(sensAddr, &temp))
		return;

	switch (sensAddr[7])
	{
	case MAIN_SENS_ID:
#if DEBUG_OUTPUT
		Serial.print("Out temperature = ");
		Serial.println(temp);
#endif
		outTemp = temp;

		if (valveOpen && outTemp > targetTemp) {

			digitalWrite(CONTROL_PIN, LOW);	// turn on servo ('0' active level)
			valveOpen = false;

			Serial.print("Out temp (");
			Serial.print(temp);
			Serial.print("C) is under target temp (");
			Serial.print(targetTemp);
			Serial.println("C), close valve");
		}
		else if (!valveOpen && (targetTemp-outTemp) > tempHyst) {

			digitalWrite(CONTROL_PIN, HIGH); // turn off servo
			valveOpen = true;

			Serial.print("Out temp (");
			Serial.print(temp);
			Serial.print("C) is bellow target temp (");
			Serial.print(targetTemp);
			Serial.println("C), open valve");
		}
		break;
	case ADD_SENS_ID:
#if DEBUG_OUTPUT
		Serial.print("In temperature = ");
		Serial.println(temp);
#endif
		inTemp = temp;
		break;
	}
}

bool readSensors(byte* addr, float* temp)
{
#define SENSORS_DEBUG	0

	byte i;
	byte data[12];

	if ( !ds.search(addr)) {
		ds.reset_search();
		delay(250);
		return false;
	}

#if SENSORS_DEBUG
	Serial.print("ROM =");
	for( i = 0; i < 8; i++) {
		Serial.write(' ');
		Serial.print(addr[i], HEX);
	}
	Serial.println();
#endif

	if (OneWire::crc8(addr, 7) != addr[7]) {

		Serial.println("Error: CRC is not valid!");
		return false;
	}

	// the first ROM byte indicates which chip
	if (addr[0] != 0x28) {

		Serial.println("Error: Chip is not DS18B20");
		return false;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44, 1);        // start conversion, with parasite power on at the end

	delay(1000);     // maybe 750ms is enough, maybe not
	// we might do a ds.depower() here, but the reset will take care of it.

	ds.reset();
	ds.select(addr);
	ds.write(0xBE);         // Read Scratchpad

#if SENSORS_DEBUG
	Serial.print("  Data = ");
	Serial.print(present, HEX);
	Serial.print(" ");
#endif

	for ( i = 0; i < 9; i++) {           // we need 9 bytes

		data[i] = ds.read();

#if SENSORS_DEBUG
		Serial.print(data[i], HEX);
		Serial.print(" ");
#endif
	}
#if SENSORS_DEBUG
	Serial.print(" CRC=");
	Serial.print(OneWire::crc8(data, 8), HEX);
	Serial.println();
#endif

	// Convert the data to actual temperature
	// because the result is a 16 bit signed integer, it should
	// be stored to an "int16_t" type, which is always 16 bits
	// even when compiled on a 32 bit processor.
	int16_t raw = (data[1] << 8) | data[0];

	byte cfg = (data[4] & 0x60);
	// at lower res, the low bits are undefined, so let's zero them
	if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
	else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
	else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
	//// default is 12 bit resolution, 750 ms conversion time

	*temp = (float)raw / 16.0;
/*
#if DEBUG_OUTPUT
	Serial.print("  Temperature = ");
	Serial.print(*temp);
#endif
*/
	return true;
}
