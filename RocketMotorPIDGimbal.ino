/*

   Description: Model Rocket motor gimbal using 2 servo. This is yet again another attempt to fly a model rocket without fins using long burn motors.
                It is using the Arduino PID controler to move a rocket motor.
                Angle changes can be monitored using a USB cable or a bluetooth interface
                Inspired by various camera gimbal projects
   Author: Boris du Reau
   Date: June 2018
   Sensor used is an MPU6050 board

   You can use an Arduino Uno/Nano or stm32F103C board

   Servo Connection
   BROWN - gnd
   red - 5v
   yellow - d10 (pwm for Sero X) or PA1 for stm32
          - d11 (servo Y) or PA2 for stm32

   MPU board Connection
   VCC - 5v
   GND - GND
   SCL - A5  or pin SCL on the stm32
   SDA - A4  or pin SDA on the stm32
   INT - D2 (not used)

  Main Menu options:
        c - calibration
        b - get altimeter config
        s - write altimeter config
        d - reset altimeter config
        h - hello
        e - erase all flight
        w - start/stop recording
        r - read one flight
        n - number of flight
        l - list all flights
        a - get all flight data
        y - Telemetry on/off

  TODO:
  Build an Android interface to monitor the telemetry and configure it

*/


#include "config.h"
#include "global.h"
#include "utils.h"
#include "kalman.h"
#include "logger_i2c_eeprom.h"
logger_I2C_eeprom logger(0x50) ;
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start adress for the flights. Anything before that is the flight index
long currentMemaddress = 200;
boolean liftOff = false;
boolean landed = true;
//ground level altitude
long initialAltitude;
long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
bool canRecord;
bool recording = false;
bool rec = false;
unsigned long initialTime = 0;
unsigned long prevTime = 0;
unsigned long diffTime;
unsigned long currentTime = 0;
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
/*
   Initial setup
    do the board calibration
    if you use the calibration function do not move the board until the calibration is complete

*/
void setup()
{
  ax_offset = 1118;
  ay_offset = 513;
  az_offset = 1289;
  gx_offset = 64;
  gy_offset = -1;
  gz_offset = -33;
  ServoX.attach(10);  // attaches the X servo on PA1 for stm32 or D10 for the Arduino Uno
  ServoY.attach(11);  // attaches the Y servo on PA2 for stm32 or D11 for the Arduino Uno

  // set both servo's to 90 degree
  ServoX.write(90);
  ServoY.write(90);
  delay(500);

  Wire.begin();

  Serial.begin(38400);
  while (!Serial);      // wait for Leonardo enumeration, others continue immediately
  bmp.begin( config.altimeterResolution);
  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude(); //bmp.readAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  boolean softConfigValid = false;
  // Read altimeter softcoded configuration
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    /*Serial.println(F("Config invalid"));
    Serial.println("ax_offset =" + config.ax_offset);
    Serial.println("ay_offset =" + config.ay_offset);
    Serial.println("az_offset =" + config.az_offset);
    Serial.println("gx_offset =" + config.gx_offset);
    Serial.println("gy_offset =" + config.gy_offset);
    Serial.println("gz_offset =" + config.gz_offset);
    */
    //delay(1000);
    //calibrate();
    config.ax_offset = ax_offset;
    config.ay_offset = ay_offset;
    config.az_offset = az_offset;
    config.gx_offset = gx_offset;
    config.gy_offset = gy_offset;
    config.gz_offset = gz_offset;
    //config.cksum = 0xBA;
    writeConfigStruc();
  }
  // INPUT CALIBRATED OFFSETS HERE; SPECIFIC FOR EACH UNIT AND EACH MOUNTING CONFIGURATION!!!!
  // use the calibrate function for yours
  // you can also write down your offset and use them so that you do not have to re-run the calibration
  ax_offset = config.ax_offset;
  ay_offset = config.ay_offset;
  az_offset = config.az_offset;
  gx_offset = config.gx_offset;
  gy_offset = config.gy_offset;
  gz_offset = config.gz_offset;

  //Initialize MPU
  initialize();

  // Get flight
  int v_ret;
  v_ret = logger.readFlightList();
  //int epromsize = logger.determineSize();
  //Serial.println(epromsize);
  long lastFlightNbr = logger.getLastFlightNbr();

  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  canRecord = logger.CanRecord();
  //canRecord = true;
}

/*

   Initialize MUP6050

*/
void initialize() {
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // initialize device
  // do not use the constructor so that we can change the gyro and ACCEL RANGE
  // values accelerometer are:
  // MPU6050_ACCEL_FS_2
  // MPU6050_ACCEL_FS_4
  // MPU6050_ACCEL_FS_8
  // MPU6050_ACCEL_FS_16
  // Values for the Gyro are:
  // MPU6050_GYRO_FS_250
  // MPU6050_GYRO_FS_500
  // MPU6050_GYRO_FS_1000
  // MPU6050_GYRO_FS_2000

  Serial.println(F("Initializing MPU 6050 device..."));
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  //turn the PID on
  //servo's can go from 60 degree's to 120 degree so set the angle correction to + - 30 degrees max
  myPIDX.SetOutputLimits(-30, 30);
  myPIDY.SetOutputLimits(-30, 30);
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  SetpointX = 0;
  SetpointY = 0;
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed, 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }
}

/*

   MAIN PROGRAM LOOP

*/
void loop(void)
{
  MainMenu();
}

void Mainloop(void)
{
  long startTime = millis();
  /*Serial.print("Start main loop: ");
    Serial.println(startTime);*/
  //read current altitude
  currAltitude = (ReadAltitude() - initialAltitude);
  bool lift = false;
  if (config.liftOffDetect == 0) { //use baro detection
    if ( currAltitude > liftoffAltitude)
      lift = true;
  }
  else { // use accelero
    if (mpu.getAccelerationY() > 30000)
      lift = true;
  }

  if ((lift && !liftOff) || (recording && !liftOff))
  {
    liftOff = true;
    if (recording)
      rec = true;
    // save the time
    initialTime = millis();
    prevTime = 0;
    if (canRecord)
    {
      long lastFlightNbr = logger.getLastFlightNbr();

      if (lastFlightNbr < 0)
      {
        currentFileNbr = 0;
        currentMemaddress = 201;
      }
      else
      {
        currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
        currentFileNbr = lastFlightNbr + 1;
      }
      //Save start address
      logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
    }
    //Serial.println("We have a liftoff");
  }
  if (canRecord && liftOff)
  {
    currentTime = millis() - initialTime;
    diffTime = currentTime - prevTime;
    prevTime = currentTime;
    logger.setFlightTimeData( diffTime);
    logger.setFlightAltitudeData(currAltitude);
    logger.setFlightTemperatureData((long) bmp.readTemperature());
    logger.setFlightPressureData((long) bmp.readPressure());

    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    logger.setFlightRocketPos((long) (w * 1000), (long) (q.x * 1000), (long) (q.y * 1000), (long) (q.z * 1000));
    logger.setFlightCorrection( (long) OutputX, (long)OutputY);
    logger.setAcceleration(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
    currentMemaddress = logger.writeFastFlight(currentMemaddress);
    currentMemaddress++;
  }

  if (((canRecord && currAltitude < 10) && liftOff && !recording && !rec) || (!recording && rec))
  {
    liftOff = false;
    rec = false;
    //end loging
    //store start and end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
    // Serial.println("We have landed");
  }

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ( fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //return;
  }

  // check for correct available data length
  if (fifoCount < packetSize)
    return;

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  fifoCount -= packetSize;

  // flush buffer to prevent overflow
  // mpu.resetFIFO();

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  //mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpuPitch = ypr[PITCH] * 180 / M_PI;
  mpuRoll = ypr[ROLL] * 180 / M_PI;
  mpuYaw  = ypr[YAW] * 180 / M_PI;
  /*mpuRoll = -ypr[ROLL] * 180 / M_PI;
    mpuYaw  = -ypr[YAW] * 180 / M_PI;*/

  // flush buffer to prevent overflow
  mpu.resetFIFO();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  // flush buffer to prevent overflow
  mpu.resetFIFO();
  InputX = mpuPitch;
  myPIDX.Compute();
  InputY = mpuRoll;
  myPIDY.Compute();

  //if using PID do those
  ServoX.write(-OutputX + 90);
  ServoY.write(OutputY + 90);

  // if you do not want to use the PID
  // ServoX.write(-mpuPitch + 90);
  // ServoY.write(mpuRoll + 90);

  float q1[4];
  //mpu.dmpGetQuaternion(&q, fifoBuffer);
  q1[0] = q.w;
  q1[1] = q.x;
  q1[2] = q.y;
  q1[3] = q.z;
  //serialPrintFloatArr(q1, 4);
  //SendTelemetry(q1, 500);
  /*Serial.println("");
    Serial.print("Yaw: ") ;
    Serial.print(mpuYaw );
    Serial.print("\tPitch: " );
    Serial.print(mpuPitch );
    Serial.print("\tRoll: ");
    Serial.print(mpuRoll);*/
  /*Serial.print("gX: ");
    Serial.print(gravity.x);
    Serial.print("\tgY: ");
    Serial.print(gravity.y);
    Serial.print("\tgZ: ");
    Serial.print(gravity.z);*/
  /*Serial.print(mpuPitch);
    Serial.print("    ");
    Serial.print(mpuRoll);
    Serial.print("    ");
    Serial.print(OutputX);
    Serial.print("    ");
    Serial.println(OutputY);*/
  if (!liftOff) // && !canRecord)
    delay(10);

  // flush buffer to prevent overflow
  mpu.resetFIFO();
  /*Serial.print("End main loop: ");
    Serial.println(millis());
    long diffTime = startTime - millis();
    Serial.print("Diff time: ");
    Serial.println(diffTime);*/
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[300];


  while ( readVal != ';') {
    Mainloop();
    while (Serial.available())
    {
      readVal = Serial.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }

  interpretCommandBuffer(commandbuffer);
}

void interpretCommandBuffer(char *commandbuffer) {
  // calibrate the IMU
  if (commandbuffer[0] == 'c')
  {
    Serial.println(F("calibration\n"));
    // Do calibration suff
    state = 0;
    calibrate();
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    config.ax_offset = ax_offset;
    config.ay_offset = ay_offset;
    config.az_offset = az_offset;
    config.gx_offset = gx_offset;
    config.gy_offset = gy_offset;
    config.gz_offset = gz_offset;
    //config.cksum = 0xBA;
    config.cksum = CheckSumConf(config);
    writeConfigStruc();
    Serial.print(F("$OK;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    Serial.print(F("$start;\n"));

    SendAltiConfig();

    Serial.print(F("$end;\n"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer))
      Serial.print(F("$OK;\n"));
    else
      Serial.print(F("$KO;\n"));
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    Serial.print(F("$OK;\n"));
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
    Serial.println(F("Erase\n"));
    logger.clearFlightList();
    logger.writeFlightList();
  }
  //start or stop recording
  else if (commandbuffer[0] == 'w')
  {
    if (commandbuffer[1] == '1') {
      Serial.print(F("Start Recording\n"));
      recording = true;
    }
    else {
      Serial.print(F("Stop Recording\n"));
      recording = false;
    }
    Serial.print(F("$OK;\n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    Serial.println(F("Read flight: "));
    Serial.println( commandbuffer[1]);
    Serial.println( "\n");
    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      logger.PrintFlight(atoi(temp));
    }
    else
      Serial.println(F("not a valid flight"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    Serial.println(F("Number of flight \n"));
    Serial.print(F("n;"));
    logger.printFlightList();
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    Serial.println(F("Flight List: \n"));
    logger.printFlightList();
  }  //get all flight data
  else if (commandbuffer[0] == 'a')
  {
    Serial.print(F("$start;\n"));
    //getFlightList()
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }

    Serial.print(F("$end;\n"));
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      Serial.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      Serial.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    Serial.print(F("$OK;\n"));
  }
  else
  {
    Serial.println(F("Unknown command" ));
    Serial.println(commandbuffer[0]);
  }
}

/*

   Send telemetry to the Android device - Uncomment if needed

*/
/*void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  int pressure;
  float batVoltage;
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude();
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      Serial.print(F("$telemetry,"));
      Serial.print("RocketMotorGimbal");
      Serial.print(F(","));
      //tab 1
      //GyroX
      Serial.print(mpu.getRotationX());
      Serial.print(F(","));
      //GyroY
      Serial.print(mpu.getRotationY());
      Serial.print(F(","));
      //GyroZ
      Serial.print(mpu.getRotationZ());
      Serial.print(F(","));
      //AccelX
      Serial.print(mpu.getAccelerationX());
      Serial.print(F(","));
      //AccelY
      Serial.print(mpu.getAccelerationY());
      Serial.print(F(","));
      //AccelZ
      Serial.print(mpu.getAccelerationZ());
      Serial.print(F(","));
      //OrientX
      Serial.print(mpuYaw);
      Serial.print(F(","));
      //OrientY
      Serial.print(mpuPitch);
      Serial.print(F(","));
      //OrientZ
      Serial.print(mpuRoll);
      Serial.print(F(","));

      //tab 2
      //Altitude
      Serial.print(currAltitude);
      Serial.print(F(","));
      //temperature
      Serial.print(temperature);
      Serial.print(F(","));
      //Pressure
      Serial.print(pressure);
      Serial.print(F(","));
      //Batt voltage
      //TODO: This pin should be declared in the scope
      pinMode(1, INPUT);
      batVoltage = analogRead(1);
      Serial.print(batVoltage);
      Serial.print(F(","));
      //tab3
      serialPrintFloatArr(arr, 4);
      Serial.println(F(";"));
    }
}
*/
/*

   Send the Gimbal configuration to the Android device

*/
void SendAltiConfig() {
  bool ret = readAltiConfig();
  //if (!ret)
  //  Serial.print(F("invalid conf"));

  Serial.print(F("$alticonfig"));
  Serial.print(F(","));
  //AltimeterName
  Serial.print("RocketMotorGimbal");
  Serial.print(F(","));
  Serial.print(config.ax_offset);
  Serial.print(F(","));
  Serial.print(config.ay_offset);
  Serial.print(F(","));
  Serial.print(config.az_offset);
  Serial.print(F(","));
  Serial.print(config.gx_offset);
  Serial.print(F(","));
  Serial.print(config.gy_offset);
  Serial.print(F(","));
  Serial.print(config.gz_offset);
  Serial.print(F(","));
  Serial.print(config.KpX);
  Serial.print(F(","));
  Serial.print(config.KiX);
  Serial.print(F(","));
  Serial.print(config.KdX);
  Serial.print(F(","));
  Serial.print(config.KpY);
  Serial.print(F(","));
  Serial.print(config.KiY);
  Serial.print(F(","));
  Serial.print(config.KdY);
  Serial.print(F(","));
  Serial.print(config.ServoXMin);
  Serial.print(F(","));
  Serial.print(config.ServoXMax);
  Serial.print(F(","));
  Serial.print(config.ServoYMin);
  Serial.print(F(","));
  Serial.print(config.ServoYMax);
  Serial.print(F(","));
  Serial.print(config.connectionSpeed);
  Serial.print(F(","));
  Serial.print(config.altimeterResolution);
  Serial.print(F(","));
  Serial.print(config.eepromSize);
  Serial.print(F(","));
  //alti major version
  Serial.print(MAJOR_VERSION);
  //alti minor version
  Serial.print(F(","));
  Serial.print(MINOR_VERSION);
  Serial.print(F(","));
  Serial.print(config.unit);
  Serial.print(F(","));
  Serial.print(config.endRecordAltitude);
  Serial.print(F(","));
  Serial.print(config.beepingFrequency);
  Serial.print(F(";\n"));
}


/*


   calibration routines those will be executed each time the board is powered up.
   we might want to calibrate it for good on a flat table and save it to the microcontroler eeprom


*/
void calibrate() {
  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  //delay(1000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  //delay(1000);
  // verify connection
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  //delay(1000);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  while (1) {
    if (state == 0) {
      Serial.println("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(100);
    }

    if (state == 1) {
      Serial.println("\nCalculating offsets...");
      calibration();
      state++;
      delay(100);
    }

    if (state == 2) {
      meansensors();
      Serial.println("\nFINISHED!");
      Serial.print("\nSensor readings with offsets:\t");
      Serial.print(mean_ax);
      Serial.print("\t");
      Serial.print(mean_ay);
      Serial.print("\t");
      Serial.print(mean_az);
      Serial.print("\t");
      Serial.print(mean_gx);
      Serial.print("\t");
      Serial.print(mean_gy);
      Serial.print("\t");
      Serial.println(mean_gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset);
      Serial.print("\t");
      Serial.print(ay_offset);
      Serial.print("\t");
      Serial.print(az_offset);
      Serial.print("\t");
      Serial.print(gx_offset);
      Serial.print("\t");
      Serial.print(gy_offset);
      Serial.print("\t");
      Serial.println(gz_offset);
      Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
      //while (1);
      break;
    }
  }
}

/*
   Used by the calibration fonction

*/
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  /*ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;*/
  ax_offset = -mean_ax / 8;
  ay_offset = (16384 - mean_ay ) / 8;
  az_offset = ( - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    /*if (abs(mean_ay) <= acel_deadzone) ready++;
      else ay_offset = ay_offset - mean_ay / acel_deadzone;

      if (abs(16384 - mean_az) <= acel_deadzone) ready++;
      else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;*/
    if (abs(16384 - mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset + (16384 - mean_ay) / acel_deadzone;

    if (abs( mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + ( - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}
