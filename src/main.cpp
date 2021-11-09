
//MASTER - ARDUINO MEGA//-

#include <Arduino.h>
#define DEBUG //ainda não utilizado

//----------------------------RTC settings----------------------------------------//
  #include <DS3231.h>                     //Biblioteca para manipulação do DS3231
  #include <Wire.h>                       //Biblioteca para manipulação do protocolo I2C
  DS3231 rtc;                             //Criação do objeto do tipo DS3231
  RTCDateTime RTC_Data;                   //Criação do objeto do tipo RTCDateTime
                                          //RTC em seu endereço padrão 0x68

//----------------------------Kalman settings-------------------------------------//
  #include <Wire.h>
  #include <Kalman.h>                     // Source: https://github.com/TKJElectronics/KalmanFilter  
  #define RESTRICT_PITCH                  // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
  
  Kalman kalmanX;                         //Criação de objeto
  Kalman kalmanY;                         //Criação de objeto

//----------------------------IMU settings---------------------------------------//
  double accX, accY, accZ;
  double gyroX, gyroY, gyroZ;
  int16_t tempRaw;
  double gyroXangle, gyroYangle;          // Angle calculate using the gyro only
  double compAngleX, compAngleY;          // Calculated angle using a complementary filter
  double kalAngleX, kalAngleY;            // Calculated angle using a Kalman filter
  uint32_t timer;
  uint8_t i2cData[14];                    // Buffer for I2C data
  uint8_t dataI2c;
  int AD0 = 4;                            // Ajuste de endereço do MPU 0x69=HIGH / 0X68=LOW

//----------------------------Timerlord settings-------------------------------------//
  #include <Time.h>
  float const LONGITUDE = -43.2311486;
  float const LATITUDE = -22.8613427;
  int const TIMEZONE = -3;
  SunLight Sun_Time;

//----------------------------Data log---------------------------------------//
  
  #include <SPI.h>
  #include <SD.h>
      
  //const int chipSelect = 4;

//----------------------------I2c Comunication---------------------------------------//
//#include <I2Cyangui.h>
//comunication com;
  #include <I2C.h>
  int ArduinoSlave = 44; //i2c scan endereço do Arduino Slave

//----------------------------Watchdog---------------------------------------//
//#include <avr/wdt.h>

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://

void setup()
{
  //----------------------------I2C settings----------------------------------//
  
  Serial.begin(9600);
  I2c.begin();

   // MPU Alternative Address //
  pinMode(AD0,OUTPUT);

  // MPU Alternative Address //
  //pinMode(AD0,OUTPUT);
  //digitalWrite(AD0,HIGH);

  //---------------------------------------------------------------------------//
  //Serial.begin(9600);
  //Wire.begin();
  

  //---------------------------------RTC settings-----------------------------//
  rtc.begin();                           //Inicialização do RTC DS3231
  rtc.setDateTime(__DATE__, __TIME__);   //Configurando valores iniciais do RTC DS3231

  //---------------------------------------------------------------------------//
    #if ARDUINO >= 157
  Wire.setClock(10000UL); // Set I2C frequency to 10kHz
  #else
  TWBR = ((F_CPU / 10000UL) - 16) / 2; // Set I2C frequency to 10kHz
  #endif

//  #if ARDUINO >= 157
//  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
//  #else
//  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
//  #endif


  i2cData[0] = 7;                                   // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;                                // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;                                // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;                                // Set Accelerometer Full Scale Range to ±2g
  while (I2c.write(0x19, i2cData, 4, false));        // Write to all four registers at once
  dataI2c = 0x01;      
  while (I2c.write(0x6B, &dataI2c, 1, true));           // PLL with X axis gyroscope reference and disable sleep mode

  while (I2c.read(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {                         // Read "WHO_AM_I" register //0x69 for AD0 High default of solar sensor project
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100);                                       // Wait for sensor to stabilize
  //----------------------------------------------------------------------//
  
  //---------------------------kalman----------------------------------//
  // Set kalman and gyro starting angle //
  while (I2c.read(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;  //atan2 outputs the value of -π to π (radians. It is then converted from radians to degrees
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll);             // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  //----------------------------------------------------------------------//
  
  //---------------------------------SD Card-------------------------------------//
  
  
  while (!Serial) {// wait for serial port to connect. Needed for native USB port only
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
  if (!SD.begin(chipSelect)) {// see if the card is present and can be initialized 
    Serial.println("Card failed, or not present");
    return; // don't do anything more:
  }
  Serial.println("card initialized.");
  
  //----------------------------wATCHDOG---------------------------------------//

  //wdt_enable(WDTO_1S);
  
}

void CallRTC()                      //Pega a data e hora do RTC e imprime no monitor Serial
  {
    RTC_Data = rtc.getDateTime();     //Atribuindo valores instantâneos de data e hora à instância data e hora
  
    Serial.print(RTC_Data.day);       //Imprimindo o Dia
    Serial.print("-");
    Serial.print(RTC_Data.month);     //Imprimindo o Mês
    Serial.print("-");
    Serial.print(RTC_Data.year);      //Imprimindo o Ano
    Serial.print("  ");
    Serial.print(RTC_Data.hour);      //Imprimindo a Hora
    Serial.print(":");
    Serial.print(RTC_Data.minute);    //Imprimindo o Minuto
    Serial.print(":");
    Serial.print(RTC_Data.second);    //Imprimindo o Segundo
    Serial.print ("  ");
    //delay(1000);                    //Tempo pra atualização do valor enviado pela porta serial
  }
//::::::::::::::::::::::::::::::::::::LOOP:::::::::::::::::::::::::::::::::::::::://

void loop() {

  //----------------------------------------------------------------------//
  // Calling RTC before for record of time //
  RTC_Data = rtc.getDateTime();                                 //Atribuindo valores instantâneos de data e hora à instância data e hora
  CallRTC();                                                   //Buscando Dados RTC
  //----------------------------------------------------------------------//
 
  //-------------------------------SD_Card-------------------------------------//
  
  // Gravação de dados //
  File dataFile = SD.open("Data_SD.txt", FILE_WRITE);

  dataFile.print(String (RTC_Data.day));       
  dataFile.print("-");
  dataFile.print(String(RTC_Data.month));     
  dataFile.print("-");
  dataFile.print(String(RTC_Data.year));        
  dataFile.print("  ");  
  dataFile.print(String(RTC_Data.hour));      
  dataFile.print(":");
  dataFile.print(String(RTC_Data.minute));    
  dataFile.print(":");
  dataFile.print(String(RTC_Data.second));    
  dataFile.print ("  ");
  
  //----------------------------------------------------------------------//
  // Calculating Sun parameters  //
   
  Sun_Time.Sun_Range (LONGITUDE, LATITUDE, TIMEZONE);
  double Sun_Setpoint = Sun_Time.Sun_Position (RTC_Data.second, RTC_Data.minute, RTC_Data.hour, RTC_Data.day, RTC_Data.month, RTC_Data.year);
  Serial.print("Angulo Desejado: ");
  Serial.print(Sun_Setpoint);
  Serial.print("\t");

  dataFile.print("Setpoint: ");
  dataFile.print(Sun_Setpoint);
  dataFile.print("\t");
  
    
  //---------------------------------MPU_Read-------------------------------------//
  
  while (I2c.read(0x3B, i2cData, 14)); // Update all the values //
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000;         // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);        // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;                           // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;


  if (gyroXangle < -180 || gyroXangle > 180) // Reset the gyro angle when it has drifted too much
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  
  
  Serial.print("Roll_K: "); Serial.print(kalAngleX); Serial.print("\t");
  Serial.print("\t");
  
  dataFile.print("Angle: "); dataFile.print(kalAngleX); dataFile.println("\t");
  dataFile.close();
 
//----------------------------------------------------------------------//
  

  delay(2);  //Verificar necessidade desse delay

  int Angle_Erro = Sun_Setpoint - kalAngleX;  //Calculo do erro ((double) Setpoint - (double)variavel do sistema)
  Serial.print("Erro = "); Serial.print(Angle_Erro);
  
  //---------------------------Comunicação com Arduino Slave-------------------------------//
  
  
  byte byte2 = int(Sun_Setpoint);        //Pegando os 8 primeiros bits
  byte byte1 = int(Sun_Setpoint) >> 8;   //Pegando os próximos 8 bits
  byte byte4 = int(kalAngleX);
  byte byte3 = int(kalAngleX) >> 8;
  
  //I2c.begin(44); //endereço do arduino slave
  I2c.write(ArduinoSlave, byte1);
  I2c.write(ArduinoSlave, byte2);
  I2c.write(ArduinoSlave, byte3);
  I2c.write(ArduinoSlave, byte4);  
  I2c.end();       //Termina a transmissão


  
  //-------------------------------------------------------------------------//

  Serial.println("");
  
}


  //-----------------------------------Watchdog reset----------------------------------//
  
  
  
  //--------------------------------------------------------------------------------//
