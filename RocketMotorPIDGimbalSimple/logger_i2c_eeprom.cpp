#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64);   
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
 
}



void logger_I2C_eeprom::begin()
{
  Wire.begin();
  //initialize Flight structure

}

void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }
}



int logger_I2C_eeprom::readFlightList() {
  eep.read(0, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig) ;
}

int logger_I2C_eeprom::readFlight(int eeaddress) {
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::writeFlightList()
{
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}

int logger_I2C_eeprom::writeFastFlight(int eeaddress){
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return i;
}

int logger_I2C_eeprom::printFlightList()
{
  //retrieve from the eeprom
  int v_ret =  readFlightList();

  //Read the stucture
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
      break;
    Serial.print("Flight Nbr: ");
    Serial.println(i);
    Serial.print("Start: ");
    Serial.println(_FlightConfig[i].flight_start);
    Serial.print("End: ");
    Serial.println(_FlightConfig[i].flight_stop);
  }
  return i;
}

void logger_I2C_eeprom::setFlightStartAddress(int flightNbr, long startAddress)
{
  _FlightConfig[flightNbr].flight_start = startAddress;
}

void logger_I2C_eeprom::setFlightEndAddress(int flightNbr, long endAddress)
{
  _FlightConfig[flightNbr].flight_stop = endAddress;
}

void logger_I2C_eeprom::setFlightTimeData( long difftime)
{
  _FlightData.diffTime = difftime;
}
void logger_I2C_eeprom::setFlightAltitudeData( long altitude)
{
  _FlightData.altitude = altitude;
}
void logger_I2C_eeprom::setFlightTemperatureData( long temperature){
  _FlightData.temperature = temperature;
}
void logger_I2C_eeprom::setFlightPressureData( long pressure){
  _FlightData.pressure = pressure;
}
//void logger_I2C_eeprom::setFlightRocketPos(char *w, char *x, char *y, char *z )
void logger_I2C_eeprom::setFlightRocketPos(long w, long x, long y, long z )
{
    /*_FlightData.w[0] = w[0];
    _FlightData.w[1] = w[1];
    _FlightData.x[0] = x[0];
    _FlightData.x[1] = x[1];
    _FlightData.y[0] = y[0];
    _FlightData.y[1] = y[1];
    _FlightData.z[0] = z[0];
    _FlightData.z[1] = z[1];*/
    _FlightData.w =w;
    _FlightData.x = x;
    _FlightData.y = y;
    _FlightData.z = z;
}
void logger_I2C_eeprom::getFlightRocketPos(long *pos) {
  /* pos[0] = _FlightData.w;
    pos[1] = _FlightData.x;
    pos[2] = _FlightData.y;
    pos[3] = _FlightData.z;*/
}
void logger_I2C_eeprom::setFlightCorrection( long OutputX, long OutputY)
{
  _FlightData.OutputX = OutputX;
  _FlightData.OutputY = OutputY;
}
void logger_I2C_eeprom::setAcceleration(long X,long Y,long Z) {
_FlightData.accelX = X;
_FlightData.accelY = Y;
_FlightData.accelZ = Z;
}
void logger_I2C_eeprom::getFlightCorrection(long *cor) {
  /*cor[0] = _FlightData.OutputX;
    cor[1] = _FlightData.OutputY;*/
}
long logger_I2C_eeprom::getFlightStart(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_start;
}
long logger_I2C_eeprom::getFlightStop(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_stop;
}

long logger_I2C_eeprom::getFlightTimeData()
{
  return _FlightData.diffTime;
}
long logger_I2C_eeprom::getFlightAltitudeData()
{
  return _FlightData.altitude;
}

void logger_I2C_eeprom::PrintFlight(int flightNbr)
{
  long startaddress;
  long endaddress;
  long flight_type;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);
  //flight_type = getFlightType(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;
    Serial.println("StartFlight;" );
    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();

      Serial.print(String(currentTime) + "," + getFlightAltitudeData()+ "," + _FlightData.temperature+ ",");
      //Serial.println("," + _FlightData.w+ ","+ _FlightData.x + "," + _FlightData.y + "," + _FlightData.z);
      Serial.print(_FlightData.pressure);
      Serial.print(",");
      Serial.print(_FlightData.w);
      Serial.print(",");
      Serial.print(_FlightData.x);
      Serial.print(",");
      Serial.print(_FlightData.y);
      Serial.print(",");
      Serial.print(_FlightData.z);
     //Serial.println("," + _FlightData.w[0]+ _FlightData.w[1]); //","+ _FlightData.x + "," + _FlightData.y + "," + _FlightData.z);

    }
    Serial.println("EndFlight;" );
  }
  else
    Serial.println(F("No such flight\n"));
}

void logger_I2C_eeprom::printFlightData(int flightNbr)
{
  int startaddress;
  int endaddress;
  long flight_type;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();
      long pos[4];
      getFlightRocketPos(pos);
      Serial.print("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + "," + String(getFlightAltitudeData()) + ",");
      Serial.print(_FlightData.temperature);
      Serial.print(",");
      Serial.print(_FlightData.pressure);
      Serial.print(",");
      //char w[2];
      //floatToByte((float)_FlightData.w/1000,w );
      //Serial.print("XXX");
      //Serial.print(_FlightData.w);
      //Serial.print("XXX");
      //float w = (float)(_FlightData.w)/1000;
      //Serial.print("YYY");
      //Serial.print(w);
      //Serial.print("YYY");
      serialFloatPrint((float)(_FlightData.w)/1000);
      Serial.print(",");
      //char x[2];
      //floatToByte((float)_FlightData.x/1000,x );
      //Serial.print(x);
      serialFloatPrint((float)(_FlightData.x)/1000);
      Serial.print(",");
      //char y[2];
      //floatToByte((float)_FlightData.y/1000,y );
      //Serial.print(y);
      serialFloatPrint((float)(_FlightData.y)/1000);
      Serial.print(",");
      //char z[2];
      //floatToByte((float)_FlightData.z/1000,z );
      //Serial.print(z);
      serialFloatPrint((float)(_FlightData.z)/1000);
      Serial.print(",");
      Serial.print(_FlightData.OutputX);
      Serial.print(",");
      Serial.print(_FlightData.OutputY);
      Serial.print(",");
      Serial.print(_FlightData.accelX);
      Serial.print(",");
      Serial.print(_FlightData.accelY);
      Serial.print(",");
      Serial.print(_FlightData.accelZ);
      Serial.println(";");
    }
  }
}

boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight == -1)
    return true;
  // Serial.println(lastFlight);
  if (lastFlight == 24)
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("25 flights");
    //#endif
    return false;
  }
  if (getFlightStop(lastFlight) > 65500 )
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("memory is full");
    //#endif
    return false;
  }
  return true;
}
