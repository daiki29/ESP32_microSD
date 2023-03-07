// ===========================================
#define usCiclo 10000  // Ciclo de ejecucion de software en microsegundos
// ==========================================
#include <SoftWire.h>
#include <SdFat.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h> 
#include "GFButton.h"
#include <LiquidCrystal_I2C.h>//Libreria para LCD I2C
LiquidCrystal_I2C lcd(0x26, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define MPU6050_address 0x68
#define DataM 1000
#define DS1307_I2C_ADDRESS 0x68

GFButton btn1 = GFButton(22,E_GFBUTTON_PULLDOWN);
GFButton btn2 = GFButton (23,E_GFBUTTON_PULLDOWN);
int ledState = HIGH;     // the current state of LED
bool disp=true;
const int LED_A   = 6; // Arduino pin connected to LED's pin
const int LED_G   = 7; // Arduino pin connected to LED's pin
char TmeStrng[] = "0000-00-00 00:00:00";     //19 ascii characters 

float angulo_pitch, angulo_roll, yawGyro, pitchGyro, rollGyro, angle_pitch_acc, angle_roll_acc, temperature, gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int gx, gy, gz, cal_int;
float acc_total_vector, ax, ay, az;
bool set_gyro_angles, accCalibOK  = false, act1=true;
float angle_yaw_acc_cal, angle_roll_acc_cal, angle_pitch_acc_cal;

int sdaPinA = 44;
int sclPinA = 45;
long tiempo_ejecucion, loop_timer;

SoftWire sw1(sdaPinA, sclPinA);
char swTxBuffer[32];
char swRxBuffer[32];

char nombrefichero[16];
int i = 1,j=0;
int exist = LOW ;
bool run_=false;

RTC_DS1307 RTC;
SdFat SD;
DateTime now;
File dataFile;


void setup() {
  lcd.begin(20, 4);//Inicializar la LCD 20x4
  lcd.backlight();//Encender la luz de fondo
  // PREPARAR COMUNICACION SERIAL
  //Serial.begin(115200);
  pinMode(LED_A, OUTPUT);
  pinMode(LED_G, OUTPUT);
  btn1.setHoldHandler(button1_on_hold);
  btn2.setHoldHandler(button2_on_hold);
  btn1.setPressHandler(button_on_press);
  btn1.setDebounceTime(50);
  btn2.setDebounceTime(50);

  for(i=0;i<20;i++)
    {
      lcd.setCursor ( 0, 0 ); 
      lcd.print(F("Waiting... "));
      ledState = !ledState;
    digitalWrite(LED_G, ledState);
    digitalWrite(LED_A, !ledState);
    delay(250);
    }
    i=0;

  sw1.begin();
  pinMode(LED_A, OUTPUT); // Led azul
  pinMode(LED_G, OUTPUT);
  init_gyro();         // Inicializar MPU

  digitalWrite(A0, HIGH); // Encendemos un Led durante la calibracion
  //==== Calibrar giroscopio
  lcd.setCursor ( 0, 0 ); 
  lcd.print(F("calibrating    "));
  lcd.setCursor ( 0, 1 ); 
  lcd.print(F("gyroscope    "));

  for (cal_int = 0; cal_int < DataM ; cal_int ++) {
    MPU_6050();            // Leemos los datos del MPU6050 3000 veces y sacamos el valor medio para obtener los offset del giroscopio
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    ledState = !ledState;
    
    if(cal_int%50){
    ledState = !ledState;
    digitalWrite(LED_G, ledState);  }
    delayMicroseconds(1000);
  }
  digitalWrite(LED_G, LOW);
  gyro_X_cal = gyro_X_cal / DataM;       // valor medio de las DataM00 muestras
  gyro_Y_cal = gyro_Y_cal / DataM;
  gyro_Z_cal = gyro_Z_cal / DataM;

  //==== Calibrar acelerometro
    lcd.setCursor ( 0, 0 ); 
  lcd.print(F("calibrating    "));
  lcd.setCursor ( 0, 1 ); 
  lcd.print(F("accelerometer  "));
  for (cal_int = 0; cal_int < DataM ; cal_int ++) {
    MPU_6050();
    angle_pitch_acc_cal += ax;
    angle_roll_acc_cal  += ay;
    angle_yaw_acc_cal   += az;
    if(cal_int%50){
    ledState = !ledState;
    digitalWrite(LED_A, ledState); }
  }
  digitalWrite(LED_A, LOW);
  angle_pitch_acc_cal = angle_pitch_acc_cal / DataM;
  angle_roll_acc_cal  = angle_roll_acc_cal / DataM;
  angle_yaw_acc_cal   = angle_yaw_acc_cal / DataM;
  accCalibOK = true;
  lcd.print(F("calibration    "));
  lcd.setCursor ( 0, 1 ); 
  lcd.print(F("ready      "));
  
  Wire.begin();                        // start the i2c interface
  RTC.begin();                         // start the RTC 
  RTC.adjust(DateTime((__DATE__), (__TIME__)));    //sets the RTC to the computer time when the sketch is loaded

  if (!SD.begin(10)) {
    lcd.clear();
    digitalWrite(LED_G, HIGH);
    lcd.setCursor ( 0, 0 ); 
    lcd.print(F("Error SD no found"));
    lcd.setCursor ( 0, 1 ); 
    lcd.print(F("Restart device"));
    act1=false;
    delay(2000);
    return;
  }

  while (exist == LOW) { 
  sprintf(nombrefichero, "datalog%000d.csv",i); //genera el archivo 1.txt, 2.txt ..... xxxxxxxx.txt 
  if (SD.exists(nombrefichero)) 
  { 
    i = i+1; 
  } 
  else 
  { 
    exist = HIGH; 
    }
  }
  exist = LOW;
   dataFile = SD.open(nombrefichero, FILE_WRITE);
   lcd.clear();
   lcd.setCursor ( 0, 0 ); 
   lcd.print(nombrefichero);
  if (dataFile) {                     // if the file is available, write to it:
    dataFile.println("Date,Ax,Ay,Az,Gx,Gy,Gz,AngPitch,AngRoll");
    dataFile.close();
  }
  else {
    for(i=0;i<20;i++)
    {
      ledState = !ledState;
    digitalWrite(LED_G, ledState);
    digitalWrite(LED_A, !ledState);
    if(i%2){
    lcd.setCursor ( 0, 2 ); 
    lcd.print(i%2);}
    delay(500);
    }
    return;
  }
    lcd.setCursor ( 0, 2 ); 
    lcd.print("Ready");
    loop_timer = micros();

}
 
void loop() {
  
  btn1.process();
  btn2.process();
  if(run_){
  while (micros() - loop_timer < usCiclo);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  MPU_6050();    // Leer MPU
  ProcesMPU();   // Procesar datos MPU
  now = RTC.now();              //this reads the time from the RTC
  sprintf(TmeStrng, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second()); // [added seconds]

  String dataString = "";                  //this erases the previous string
    dataString += TmeStrng;
    dataString += ",";      
    dataString += String(ax/8192.0);
    dataString += ",";   
    dataString += String(ay/8192.0);
    dataString += ",";    
    dataString += String(az/8192.0);
    dataString += ",";    
    dataString += String(pitchGyro);
    dataString += ",";    
    dataString += String(rollGyro);
    dataString += ",";    
    dataString += String(yawGyro);
    dataString += ",";    
    dataString += String(angulo_roll);
    dataString += ",";    
    dataString += String(angulo_pitch);
    if(disp){
    lcd.setCursor ( 0, 0 ); 
    lcd.print(F("ax: "));
    lcd.setCursor ( 0, 1 ); 
    lcd.print(ax/8192.0,4);
    lcd.print(F(" "));
    lcd.setCursor ( 10, 0 ); 
    lcd.print(F("ay: "));
    lcd.setCursor ( 10, 1 ); 
    lcd.print(ay/8192,4);
    lcd.setCursor ( 0, 2 ); 
    lcd.print(F("az"));
    lcd.setCursor ( 0, 3 ); 
    lcd.print(az/8192,4);
    lcd.print(F(" "));
    lcd.setCursor ( 10, 2 ); 
    lcd.print(F("roll: "));
    lcd.setCursor ( 10, 3 ); 
    lcd.print(angulo_pitch,4);
    lcd.print(F(" "));}
    else
    {
      lcd.setCursor ( 0, 0 ); 
    lcd.print(F("gx: "));
    lcd.setCursor ( 0, 1 ); 
    lcd.print(pitchGyro,4);
    lcd.print(F(" "));
    lcd.setCursor ( 10, 0 ); 
    lcd.print(F("gy: "));
    lcd.setCursor ( 10, 1 ); 
    lcd.print(rollGyro,4);
    lcd.setCursor ( 0, 2 ); 
    lcd.print(F("gz"));
    lcd.setCursor ( 0, 3 ); 
    lcd.print(yawGyro,4);
    lcd.print(F(" "));
    lcd.setCursor ( 10, 2 ); 
    lcd.print(F("pitch: "));
    lcd.setCursor ( 10, 3 ); 
    lcd.print(angulo_roll,4);
    lcd.print(F(" "));
      }
    
    dataFile = SD.open(nombrefichero, FILE_WRITE);  // if the file is available, write to it:
    if (dataFile) 
    {
      dataFile.println(dataString);
      dataFile.close();
    }
    else
    {
      lcd.setCursor ( 0, 0 ); 
      lcd.print(F("Error write SD"));
      lcd.setCursor ( 0, 1 ); 
      lcd.print(F("Restart device"));
      delay(2000);
      act1=false;
      return;
      digitalWrite(LED_G, HIGH);
      return;
      }
    j++;
    if(j%5)
    {
    ledState = !ledState;
    digitalWrite(LED_A, ledState);
      }
    if(j>19)
    {j=0;}
  }
  else
  {
    lcd.setCursor ( 10, 3 ); 
    lcd.print(F("Standby"));
    digitalWrite(LED_A, LOW);
    digitalWrite(LED_G, LOW);
    loop_timer = micros();
    }
}
void button_on_press(GFButton & btn)
{  // Use of the press count field
  disp=!disp;
}
 
// MANEJADOR DE EVENTOS DEL BOTON DE ENCENDIDO
void button1_on_hold(GFButton & button)
{
  if (button.isFirstHold())
  {
    lcd.clear();
    while (exist == LOW) { 
    sprintf(nombrefichero, "datalog%000d.csv",i); //genera el archivo 1.txt, 2.txt ..... xxxxxxxx.txt 
  if (SD.exists(nombrefichero)) 
  { 
    i = i+1; 
  } 
  else 
  { 
    exist = HIGH; 
    }
  }
  exist = LOW;
  
   dataFile = SD.open(nombrefichero, FILE_WRITE);
   lcd.setCursor ( 0, 0 ); 
   lcd.print(nombrefichero);
  if (dataFile) {                     // if the file is available, write to it:
    dataFile.println("Date,Ax,Ay,Az,Gx,Gy,Gz,AngPitch,AngRoll");
    dataFile.close();
  }
   else {
    for(i=0;i<15;i++)
    {
      ledState = !ledState;
    digitalWrite(LED_G, ledState);
    digitalWrite(LED_A, !ledState);
    delay(250);
    }
    return;
  }
    digitalWrite(LED_A, LOW);
  } else {
        ledState = !ledState;
    digitalWrite(LED_A, ledState); 
  }
}
 
// MANEJADOR DE EVENTOS DEL BOTON DE APAGADO
void button2_on_hold(GFButton & button)
{
  if (button.isFirstHold())
  {
    run_= !run_;
    if(run_)
    {
      lcd.clear();
      }
    digitalWrite(LED_G, LOW);
  } else {
    for(i=0;i<15;i++)
    {
    ledState = !ledState;
    digitalWrite(LED_G, ledState);
    delay(250); 
    }
    digitalWrite(LED_G, LOW); 
  }
}


//================= Leer MPU6050
void MPU_6050() {
  sw1.beginTransmission(MPU6050_address);
  sw1.write(0x3B); // Access the first register
  sw1.endTransmission();

  uint8_t registers[14]; // There are 41 registers we need to read from to get the date and time.
  int numBytes = sw1.requestFrom(MPU6050_address, (uint8_t)14);
  for (int i = 0; i < numBytes; ++i) {
    registers[i] = sw1.read();
  }
  if (numBytes != 14) {
    return;
  }

// Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  ax=registers[0]<<8|registers[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay=registers[2]<<8|registers[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=registers[4]<<8|registers[5];  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature=registers[6]<<8|registers[7];  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=registers[8]<<8|registers[9];  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=registers[10]<<8|registers[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=registers[12]<<8|registers[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

//================= Calculo de w y angulo
void ProcesMPU() {
  
  // ================================================================================================== Calibracion
  ax -= angle_pitch_acc_cal;
  ay -= angle_roll_acc_cal;
  az -= angle_yaw_acc_cal;
  az = az + 8192;

  // ================================================================================================== Calculo w
  pitchGyro = (gx - gyro_X_cal) / 131; // 131: si leo 131 en raw, significa que gira a 1º/s
  rollGyro  = (gy - gyro_Y_cal) / 131;
  yawGyro   = (gz - gyro_Z_cal) / 131;

  // ================================================================================================== Calculo ang
  angulo_pitch += pitchGyro * tiempo_ejecucion / 1000 ;
  angulo_roll  += rollGyro * tiempo_ejecucion / 1000 ;
  angulo_pitch += angulo_roll * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);      // tiempo_ejecucion/1000 /131 * PI/180
  angulo_roll  -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);

  acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angle_pitch_acc  = asin((float)ay / acc_total_vector) * 57.2958;     // 57.2958 = Conversion de radianes a grados 180/PI
  angle_roll_acc   = asin((float)ax / acc_total_vector) * -57.2958;

  angulo_pitch = angulo_pitch * 0.99 + angle_pitch_acc * 0.01;      // Corregimos en drift con filtro complementario
  angulo_roll  = angulo_roll * 0.99 + angle_roll_acc * 0.01;
}

//================= Subrutina inicilialización (solo se ejecuta una vez al iniciar el programa)
void init_gyro() {
  sw1.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw1.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw1.beginTransmission(MPU6050_address);
  sw1.write(0x6B);                           // PWR_MGMT_1 registro 6B hex
  sw1.write(0x00);                           // 00000000 para activar
  sw1.endTransmission();
  sw1.beginTransmission(MPU6050_address);
  sw1.write(0x1B);                           // GYRO_CONFIG registro 1B hex
  sw1.write(0x00);                           // 00001000: 500dps  //250dps 131
  sw1.endTransmission();
  sw1.beginTransmission(MPU6050_address);
  sw1.write(0x1C);                          // ACCEL_CONFIG registro 1C hex
  sw1.write(0x08);                          // 00010000: +/- 8g //+/-4g 8192
  sw1.endTransmission();
  sw1.beginTransmission(MPU6050_address);
  sw1.write(0x1A);                        // LPF registro 1A hex
  sw1.write(0x04);                        // 256Hz(0ms):0x00 - 188Hz(2ms):0x01 - 98Hz(3ms):0x02 - 42Hz(4.9ms):0x03 - 20Hz(8.5ms):0x04 - 10Hz(13.8ms):0x05 - 5Hz(19ms):0x06
  sw1.endTransmission();
}
