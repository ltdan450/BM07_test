#include <Arduino_LSM6DS3.h>
//#include <LSM6DS3.h>

#define CG_DESCENT_IMPLEMENTATION
#include <cg_descent.h>

// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

//B
// 0 : nothing
// 1 : nothing
// 2 : MMC5603NJ
// 3 : QMC5883L
// 4 : MMC5603NJ
// 5 : QMC5883L
// 6 : MMC5603NJ
// 7 : QMC5883L
// 8 : nothing

//small magnet is 25.40 x 1.50 x 6.34mm

#include <Wire.h>

//#include <filters.h>
//#include "Biquad.h"

#include <math.h>
#include <assert.h>
#define CG_FLOAT float
#define CG_INT int
//#include <EEPROM.h>
// Variables supplied by customer
const char * networkName = "jimmy";
const char * networkPswd = "meatball";
const float b_earth_mag = 50.8;

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * udpAddress = "192.168.0.148";
const int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//The udp library class
//WiFiUDP udp;

#define RESET_MC 27
#define SW1 4
#define SW2 5
#define SW3 21
#define MMC5603NJ 0b00110000  // =0x30
#define QMC5883P 0b00101100 // =0x2C
#define LSM6DS3TR 0b01101010 // =0x6A

#define Control_Register0 0x1B
#define Device_Status1 0x18

//MMC5603NJ constants
#define MMC5603_CR0 0x1B
#define MMC5603_CR1 0x1C
#define MMC5603_CR2 0x1D
#define MMC5603_ODR 0x1A
#define MMC5603_Stat1 0x18

//QMC5883P constants
#define QMC5883_CR1 0x0A
#define QMC5883_CR2 0x0B
#define QMC5883_sign 0x29

//Biquad bqz2 = Biquad();//int type, double Fc, double Q, double peakGainDB);
//bqz2->setFC(0.1);


// Setup QMC5883L
uint8_t QMC5883_CR1_config = 0b00000110;
uint8_t QMC5883_CR2_config = 0b01001100;
uint8_t QMC5883_sign_config = 0x06;

int X0, X1, X2, Y0, Y1, Y2, Z0, Z1, Z2, X_out, Y_out, Z_out;
unsigned long X_byt, Y_byt, Z_byt;

float filt_block_xy[8][3][6] = {0}; //filt_block[i][j][k], i is sensor num, j is xyz, k=0  is x0 (mult by b) k=4 is y2 (mult by a)
float bas[1][5];

const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
const float sampling_time = 1.0 / 50.0; //Sampling time in seconds.

float vals[8][3] = {0.0};   // raw values
float mfs [8][3] = {0.0};   // fitered values
float a = 0.78;

float mag_cals[8][9] = {0.0}; 

int run_mode = 0; // 0 = ready for anything, 1 = enter cal mode, 2 = cal mode running



/***  Calibration variables, global ******/
uint8_t sensor_under_cal = 64;
int pts = 512;
int o_step = 1;
float scratch [8 * 3 * 512] = {0};
int scratch_user = 0;
  float eps = 100.0;

/**** TCA9548APWR active I2c bus *****/
uint8_t bus_number = 0;


void setup_filter(void) {
  //bas[i][j], i is filter number, j=0 is b0, j=1 is b1, j=2, b2, j=3 is a0, j=4 is a1
  float b_0[] = {0.00024132, 0.00048264, 0.00024132};
  float a_0[] = {1.95558189, -0.95654717};

  bas[0][0] = b_0[0];
  bas[0][1] = b_0[1];
  bas[0][2] = b_0[2];
  bas[0][3] = a_0[0];
  bas[0][4] = a_0[1];
}

void rot_matrix(float ang_zyx[3], float R[3][3] ){
  float psi = ang_zyx[0];
  float theta = ang_zyx[1];
  float phi = ang_zyx[2];

  float spsi = sinf(psi);
  float cpsi = cosf(psi);
  float stheta = sinf(theta);
  float ctheta = cosf(theta);
  float sphi = sinf(phi);
  float cphi = cosf(phi);
  R[0][0] = cpsi*ctheta;  R[0][1] = cpsi*stheta*sphi-spsi*cphi; R[0][2] = cpsi*stheta*cphi+spsi*sphi;
  R[1][0] = spsi*ctheta;  R[1][1] = spsi*stheta*sphi+cpsi*cphi; R[1][2] = spsi*stheta*cphi+cpsi*spsi;
  R[2][0] = -stheta;      R[2][1] = ctheta*sphi;                R[2][2] = ctheta*cphi;

  if(1) {
    Serial.printf("\n Rotation matrix based on ang_zyx %f %f %f:\n", ang_zyx[0], ang_zyx[1], ang_zyx[2]);
    for (int c = 0; c <3; c++) {
      for (int r = 0; r<3; r++) {
        Serial.printf("%f ",R[r][c]);
      }
      Serial.print("\n");
    }



  }


}

void trans_matrix(float matIn[3][3], float tmat[3][3]){
  tmat[0][0] = matIn[0][0];   tmat[0][1] = matIn[1][0];   tmat[0][2] = matIn [2][0];
  tmat[1][0] = matIn[0][1];   tmat[1][1] = matIn[1][1];   tmat[1][2] = matIn [2][1];
  tmat[2][0] = matIn[0][2];   tmat[2][1] = matIn[1][2];   tmat[2][2] = matIn [2][2];
}

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void config_sensors(void) {
  // Setup MMC5603NJs
  bus_number = 3;
  TCA9548A(bus_number);
  //Wire.beginTransmission(QMC5883P);
  //Wire.write(QMC5883_CR1);
  //Wire.write(QMC5883_CR1_config); //Set signal
  //Wire.endTransmission();

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_CR2);
  Wire.write(QMC5883_CR2_config); //Set signal
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_sign);
  Wire.write(QMC5883_sign_config); //Set signal
  Wire.endTransmission();

  bus_number = 5;
  TCA9548A(bus_number);
  //Wire.beginTransmission(QMC5883P);
  //Wire.write(QMC5883_CR1);
  //Wire.write(QMC5883_CR1_config); //Set signal
  //Wire.endTransmission();


  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_CR2);
  Wire.write(QMC5883_CR2_config); //Set signal
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_sign);
  Wire.write(QMC5883_sign_config); //Set signal
  Wire.endTransmission();

  bus_number = 7;
  TCA9548A(bus_number);
  //Wire.beginTransmission(QMC5883P);
  //Wire.write(QMC5883_CR1);
  //Wire.write(QMC5883_CR1_config); //Set signal
  //Wire.endTransmission();


  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_CR2);
  Wire.write(QMC5883_CR2_config); //Set signal
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_sign);
  Wire.write(QMC5883_sign_config); //Set signal
  Wire.endTransmission();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

void get_lsm6ds3_mx(void) {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  float x, y, z;
  float acc[3];

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

  float gx, gy, gz;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.println(gz);
  }
}

void scani2c (void) {
  uint8_t inbuff [6];
  byte error;
  bus_number = 3;
  TCA9548A(bus_number);
  for (uint8_t j = 0; j <= 8; j++) {

    TCA9548A(j);
    for (uint8_t i = 1; i < 127; i++) {
      Wire.beginTransmission(i);
      error = Wire.endTransmission();
      if (error == 0) {
        Serial.printf("\n no error at addr:%d, bus:%d", i, j);
      }
    }
  }
  Serial.println("error:");
  Serial.println(error);



}

void get_mx_QMC (void) {
  uint8_t inbuff [6];
  for (int i = 0; i < 6; i++) {
    inbuff[i] = 0;
  }
  byte error;
  bus_number = 3;
  TCA9548A(bus_number);

  Wire.beginTransmission(QMC5883P);
  Wire.write(0x00);
  error = Wire.endTransmission();
  Wire.requestFrom(QMC5883P, 1);
  Serial.printf("\n err:%d", (int)error);
  inbuff[0] = Wire.read();

  Serial.println("buffer bytes:");
  for (int i = 0; i < 6; i++) {
    Serial.println(inbuff[i]);
  }

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_CR2);
  Wire.write(QMC5883_CR2_config); //Set signal
  error = Wire.endTransmission();
  Serial.printf("\n err:%d", (int)error);

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_sign);
  Wire.write(QMC5883_sign_config); //Set signal
  error = Wire.endTransmission();
  Serial.printf("\n err:%d", (int)error);

  Wire.beginTransmission(QMC5883P);
  Wire.write(QMC5883_CR1);
  Wire.write(QMC5883_CR1_config); //Set signal
  error = Wire.endTransmission();
  Serial.printf("\n err:%d", (int)error);

  Wire.beginTransmission(QMC5883P);
  Wire.write(0x09);
  error = Wire.endTransmission();
  Serial.printf("\n err:%d", (int)error);
  Wire.requestFrom(QMC5883P, 1);
  inbuff[0] = Wire.read();

  Serial.println("buffer bytes:");
  for (int i = 0; i < 6; i++) {
    Serial.println(inbuff[i]);
  }

  Wire.beginTransmission(QMC5883P);
  Wire.write(0x01);
  error = Wire.endTransmission();
  Serial.printf("\n err:%d", (int)error);
  Wire.requestFrom(QMC5883P, 6);
  for (int i = 0; i < 6; i++) {
    inbuff[i] = Wire.read();
  }

  Serial.println("buffer bytes:");
  for (int i = 0; i < 6; i++) {
    Serial.println(inbuff[i]);
  }

  float x = (float)(((inbuff[1] << 8) | (inbuff[0] << 0)) - 32768) * 0.000066666666666666666f;
  float y = (float)(((inbuff[3] << 8) | (inbuff[2] << 0)) - 32768) * 0.000066666666666666666f;
  float z = (float)(((inbuff[5] << 8) | (inbuff[4] << 0)) - 32768) * 0.000066666666666666666f;

  Serial.printf("%f %f %f\n", x, y, z);






}

void get_mx_MMC (void) {
  uint8_t inbuff [9];
  for (int i = 0; i < 9; i++) {
    inbuff[i] = 0;
  }
  byte error;
  bus_number = 2;
  TCA9548A(bus_number);

  Wire.beginTransmission(MMC5603NJ);
  Wire.write(0x39);
  error = Wire.endTransmission();
  Wire.requestFrom(MMC5603NJ, 1);
  Serial.printf("\n err:%d", (int)error);
  inbuff[0] = Wire.read();

  Serial.println("buffer bytes:");
  for (int i = 0; i < 9; i++) {
    Serial.println(inbuff[i]);
  }

  Wire.beginTransmission(MMC5603NJ);
  Wire.write(Control_Register0);
  Wire.write(0x21); // m measuring enable with auto-reset (0010 0001)
  error = Wire.endTransmission();
  Serial.printf("\n err:%d", (int)error);

  Wire.beginTransmission(MMC5603NJ);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MMC5603NJ, 9, true);
  for (int i = 0; i < 9; i++) {
    inbuff[i] = Wire.read();
  }

  Serial.println("buffer bytes:");
  for (int i = 0; i < 9; i++) {
    Serial.println(inbuff[i]);
  }

  float x = (float)(((inbuff[0] << 12) | (inbuff[1] << 4) | (inbuff[6] >> 4)) - 524288) * 0.00006103515625f * 100.0f;
  float y = (float)(((inbuff[2] << 12) | (inbuff[3] << 4) | (inbuff[7] >> 4)) - 524288) * 0.00006103515625f * 100.0f;
  float z = (float)(((inbuff[4] << 12) | (inbuff[5] << 4) | (inbuff[8] >> 4)) - 524288) * 0.00006103515625f * 100.0f;

  Serial.printf("%f %f %f\n", x, y, z);






}

void measure_mag (int n, float*xval, float*yval, float*zval) {
  uint8_t inbuff [9];
  if (n == 2 || n == 4 || n == 6) {
    // It is a MMC5603
    byte error;
    bus_number = (uint8_t)n;
    TCA9548A(bus_number);
    Wire.beginTransmission(MMC5603NJ);
    Wire.write(Control_Register0);
    Wire.write(0x21); // m measuring enable with auto-reset (0010 0001)
    error = Wire.endTransmission();

    Wire.beginTransmission(MMC5603NJ);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(MMC5603NJ, 9, true);
    for (int i = 0; i < 9; i++) {
      inbuff[i] = Wire.read();
    }

    //float x = (float)(((inbuff[0]<<12) | (inbuff[1]<<4)| (inbuff[6]>>4)) - 524288)*0.00006103515625f*100.0f;
    //float y = (float)(((inbuff[2]<<12) | (inbuff[3]<<4)| (inbuff[7]>>4)) - 524288)*0.00006103515625f*100.0f;
    //float z = (float)(((inbuff[4]<<12) | (inbuff[5]<<4)| (inbuff[8]>>4)) - 524288)*0.00006103515625f*100.0f;
    //Serial.printf("%f %f %f\n",x,y,z);
    *xval = (float)(((inbuff[0] << 12) | (inbuff[1] << 4) | (inbuff[6] >> 4)) - 524288) * 0.00006103515625f * 100.0f;
    *yval = (float)(((inbuff[2] << 12) | (inbuff[3] << 4) | (inbuff[7] >> 4)) - 524288) * 0.00006103515625f * 100.0f;
    *zval = (float)(((inbuff[4] << 12) | (inbuff[5] << 4) | (inbuff[8] >> 4)) - 524288) * 0.00006103515625f * 100.0f;


  }
  else if (n == 3 || n == 5 || n == 7) {
    // It is a QMC5883
    byte error;
    bus_number = (uint8_t)n;
    TCA9548A(bus_number);

    Wire.beginTransmission(QMC5883P);
    Wire.write(QMC5883_CR1);
    Wire.write(QMC5883_CR1_config); //Set signal
    error = Wire.endTransmission();

    Wire.beginTransmission(QMC5883P);
    Wire.write(0x01);
    error = Wire.endTransmission();

    Wire.requestFrom(QMC5883P, 6);
    for (int i = 0; i < 6; i++) {
      inbuff[i] = Wire.read();
    }

    //float x = (float)(((inbuff[1]<<8) | (inbuff[0]<<0)) - 32768)*0.000066666666666666666f*100.0f;
    //float y = (float)(((inbuff[3]<<8) | (inbuff[2]<<0)) - 32768)*0.000066666666666666666f*100.0f;
    //float z = (float)(((inbuff[5]<<8) | (inbuff[4]<<0)) - 32768)*0.000066666666666666666f*100.0f;
    //Serial.printf("%f %f %f\n",x,y,z);
    *xval = (float)(((inbuff[1] << 8) | (inbuff[0] << 0)) - 32768) * 0.000066666666666666666f * 100.0f;
    *yval = (float)(((inbuff[3] << 8) | (inbuff[2] << 0)) - 32768) * 0.000066666666666666666f * 100.0f;
    *zval = (float)(((inbuff[5] << 8) | (inbuff[4] << 0)) - 32768) * 0.000066666666666666666f * 100.0f;
  }
}

void get_sensor_vals(void) {
  int tm0 = millis();
  for (int i = 2; i < 7; i++) {
    measure_mag(i, &vals[i - 2][0], &vals[i - 2][1], &vals[i - 2][2]);
    //Serial.printf("\nmeasurment :%d millis:%d\n",i,millis()-t0);
  }

  //Serial.printf("\nmeasurment millis:%d\n",millis()-t0);
  IMU.readAcceleration(vals[6][0], vals[6][1], vals[6][2]);
  //Serial.printf("\nmeasurment millis:%d\n",millis()-t0);
  IMU.readGyroscope(vals[7][0], vals[7][1], vals[7][2]);
  //Serial.printf("\nmeasurment millis:%d\n",millis()-t0);
  int t1 = millis();
  if (0) Serial.printf("\nmeasurment loop millis:%d\n", t1 - tm0);
}

void filter_sensor_vals(void) {
  for (int i = 0; i < 8; i += 1) {
    for (int j = 0; j < 3; j++) {
      mfs[i][j] = ((1.0 - a) * vals[i][j] + a * mfs[i][j]) ;
      //Serial.printf("\n raw%d_%d: %f filt%d_d: %f  cal:%f\n",i,j,vals[i][j],j,mfs[i][j], mag_cals[i][j]);
    }
  }
}

void get_measurements(float mx[8][3]) {
  int raw_data_mmc5603 [3][3][3];
  int raw_data_qmc5883 [3][3][2];
  int delaytime = 10;

  for (uint8_t i = 2; i <= 7; i += 2) {
    bus_number = i;
    TCA9548A(bus_number);

    //Removed writing 0x08 to Control_Register0

    Wire.beginTransmission(MMC5603NJ);
    Wire.write(Control_Register0);
    Wire.write(0x21); // m measuring enable with auto-reset (0010 0001)
    Wire.endTransmission();
    delay(delaytime);
  }

  for (uint8_t i = 3; i <= 8; i += 2) {
    bus_number = i;
    TCA9548A(bus_number);
    Wire.beginTransmission(QMC5883P);
    Wire.write(QMC5883_CR1);
    Wire.write(QMC5883_CR1_config); //Set signal
    Wire.endTransmission();
    //Serial.printf("\n err:%d",(int)error);
    delay(delaytime);
  }

  IMU.readAcceleration(mx[6][0], mx[6][1], mx[6][2]);
  IMU.readGyroscope(mx[7][0], mx[7][1], mx[7][2]);

  for (uint8_t i = 2; i <= 7; i += 2) {
    bus_number = i;
    TCA9548A(bus_number);
    Wire.beginTransmission(MMC5603NJ);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(MMC5603NJ, 9, true);
    //Serial.printf("\nMMC i:%d ref:%d\n",i,i/2-1);
    raw_data_mmc5603[i / 2 - 1][0][0] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][0][1] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][1][0] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][1][1] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][2][0] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][2][1] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][0][2] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][1][2] = Wire.read();
    raw_data_mmc5603[i / 2 - 1][2][2] = Wire.read();
    delay(delaytime);
  }

  for (uint8_t i = 3; i <= 8; i += 2) {
    bus_number = i;
    TCA9548A(bus_number);
    Wire.beginTransmission(QMC5883P);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(QMC5883P, 6, true);
    //Serial.printf("\nQMC i:%d ref:%d\n",i,(i-1)/2-1);
    raw_data_qmc5883[(i - 1) / 2 + 2][0][0] = Wire.read();
    raw_data_qmc5883[(i - 1) / 2 + 2][0][1] = Wire.read();
    raw_data_qmc5883[(i - 1) / 2 + 2][1][0] = Wire.read();
    raw_data_qmc5883[(i - 1) / 2 + 2][1][1] = Wire.read();
    raw_data_qmc5883[(i - 1) / 2 + 2][2][0] = Wire.read();
    raw_data_qmc5883[(i - 1) / 2 + 2][2][1] = Wire.read();
    delay(delaytime);
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mx[i][j] = (float)(((raw_data_mmc5603[i][j][0] << 12) | (raw_data_mmc5603[i][j][1] << 4) | (raw_data_mmc5603[i][j][2] >> 4)) - 524288) * 0.00006103515625f;
    }
  }

  for (int i = 3; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      mx[i][j] = (float)(((raw_data_qmc5883[i - 3][j][1] << 8) | (raw_data_qmc5883[i - 3][j][0] << 0)) - 32768) * 0.000066666666666666666f;
    }
  }

  //float x = (float)(((inbuff[1]<<8) | (inbuff[0]<<0)) - 32768)*0.000066666666666666666f;
  //float y = (float)(((inbuff[3]<<8) | (inbuff[2]<<0)) - 32768)*0.000066666666666666666f;
  //float z = (float)(((inbuff[5]<<8) | (inbuff[4]<<0)) - 32768)*0.000066666666666666666f;
  if (true) {
    Serial.println("new MX:\n");
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 8; i++) {
        float val = mx [i][j];
        if (val >= 0.0) Serial.print(" ");
        Serial.printf(" %f ", val);
      }
      Serial.print("\n");
    }
  }
}

void filt (int n, float x, float y, float z) {
  // float xyz are incoming unfiltered sensor data
  Serial.println("hello!");
  Serial.printf("\nfilt input n:%d x:%f y:%f z:%f\n", n, x, y, z);

  float incoming[3] = {x, y, z};
  //filt_block_xy[i][j][k], i is sensor num, j is xyz, k=0  is x0 (mult by b) k=5 is y2 (mult by a)
  //k0:x0, k1:x1, k2:x2, k3:y0, k4:y1, k5:y2

  //bas[i][j], i is filter number, j=0 is b0, j=1 is b1, j=2, b2, j=3 is a0, j=4 is a1
  //  0   1   2   3   4   5
  //  x0  x1  x2  y0  y1  y2
  //  b0  b1  b2  a0  a1

  Serial.printf("b0:%f b1:%f b2:%f a0:%f a1:%f", bas[0][0], bas[0][1], bas[0][2], bas[0][3], bas[0][4]);

  //X axis first
  for (int i = 0; i < 3; i++) {
    if (1) {
      Serial.print("\n starting filt block \n");
      for (int h = 0; h < 6; h++) {
        Serial.printf(" %f ", filt_block_xy[n][i][h]);
        Serial.print("\n");
      }
    }

    filt_block_xy[n][i][0]  = incoming[i];
    //          y[0]        =   a[0]    *           y[1]          + a[1]      * y[2]                    +  b[0]       *   x[0]                    + b[1]      * x[1]                    + b[2]      * x[2];
    filt_block_xy[n][i][3]  = bas[0][3] * filt_block_xy[n][i][4]  + bas[0][4] * filt_block_xy[n][i][5]  +  bas[0][0]  *   filt_block_xy[n][i][0]  + bas[0][1] * filt_block_xy[n][i][1]  + bas[0][2] * filt_block_xy[n][i][2];

    if (1) {
      Serial.print("\n ending filt block \n");
      for (int h = 0; h < 6; h++) {
        Serial.printf(" %f ", filt_block_xy[n][i][h]);
        Serial.print("\n");
      }
    }

    for (int j = 1; j >= 0; j--) {
      //x[j+1] = x[j]; // store xi
      filt_block_xy[n][i][j + 1] = filt_block_xy[n][i][j];
      //y[j+1] = y[j]; // store yi
      filt_block_xy[n][i][j + 4] = filt_block_xy[n][i][j + 3];
    }
    if (1) {
      Serial.print("\n shifted filt block \n");
      for (int h = 0; h < 6; h++) {
        Serial.printf(" %f ", filt_block_xy[n][i][h]);
        Serial.print("\n");
      }
    }

  }
  Serial.printf("\nfilt output n:%d x:%f y:%f z:%f\n", n, filt_block_xy[n][0][3], filt_block_xy[n][1][3], filt_block_xy[n][2][3]);




  //  y[0] = a[0]*y[1] + a[1]*y[2] + b[0]*x[0] + b[1]*x[1] + b[2]*x[2];


  /*

       float t = micros()/1.0e6;
    x[0] = sin(2*PI*2*t) + 0.5*sin(2*PI*25*t);

    // Compute the filtered signal
    // (second order Butterworth example)
    float b[] = {0.00024132, 0.00048264, 0.00024132};
    float a[] = {1.95558189, -0.95654717};
    y[0] = a[0]*y[1] + a[1]*y[2] +
               b[0]*x[0] + b[1]*x[1] + b[2]*x[2];

    if(k % 3 ==0)
    {
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency

    // For the serial monitor
    Serial.print(2*x[0]);
    Serial.print(" ");
    Serial.println(2*y[0]);
    }

    delay(1); // Wait 1ms
    for(int i = 1; i >= 0; i--){
    x[i+1] = x[i]; // store xi
    y[i+1] = y[i]; // store yi
    }
  */



}

void set_ellipse_grad(float *g, float *p_in, int n){
  //float eps = 0.001;
  float * x_buff = (float*) malloc(n * sizeof(float));
  float err_zero = get_ellipse_error(p_in,n);
  for (int i = 0; i<n; i++){
    for (int j = 0; j<n; j++) { 
      if (j==i) x_buff[j] = p_in[j]+eps;
      else x_buff[j] = p_in[j];
      //Serial.printf("i:%d j:%d pin[j]:%f xbuff[j]:%f",i,j,p_in[j],x_buff[j]);
    }
    g[i] = (-err_zero + get_ellipse_error(x_buff,n))/eps;
  }

  Serial.printf("\nerr_z:% g:%f %f %f %f %f %f x:%f %f %f %f %f %f\n",err_zero, g[0], g[1], g[2], g[3], g[4], g[5], x_buff[0],x_buff[1], x_buff[2], x_buff[3], x_buff[4], x_buff[5]);


  free(x_buff);
}

float get_ellipse_error(float *p_in, int n) {
  float a = p_in [0];
  float b = p_in [1];
  float c = p_in [2];
  a,b,c = b_earth_mag;
  if (a == 0.0) a = 0.000001;
  if (b == 0.0) b = 0.000001;
  if (c == 0.0) c = 0.000001;
  float x0 = p_in [3];
  float y0 = p_in [4];
  float z0 = p_in [5];
  //Serial.printf("\nellipse error call, a:%f, b:%f, c:%f, x0:%f, y0:%f, z0:%f ",a,b,c,x0,y0,z0);
  float error_sum = 0.0;
  int terms = 0;
  for (int i = 0; i<pts; i+=o_step) {
    terms++;
    float x = scratch[pts*sensor_under_cal*3+i*3+0];
    float y = scratch[pts*sensor_under_cal*3+i*3+1];
    float z = scratch[pts*sensor_under_cal*3+i*3+2];

    //float x_calc = x0 - (a*sqrt(b*b*c*c - b*b*z*z + 2 *b*b*z*z0 - b*b*z0*z0 - c*c*y*y + 2*c*c*y*y0 - c*c*y0*y0))/(b*c);
    //float y_calc = y0 - (b*sqrt(a*a*c*c - a*a*z*z + 2 *a*a*z*z0 - a*a*z0*z0 - c*c*x*x + 2*c*c*x*x0 - c*c*x0*x0))/(a*c);
    //float z_calc = z0 - (c*sqrt(a*a*b*b - a*a*y*y + 2 *a*a*y*y0 - a*a*y0*y0 - b*b*x*x + 2*b*b*x*x0 - b*b*x0*x0))/(a*b);



    float err = ((x-x0)*(x-x0)/(a*a) + (y-y0)*(y-y0)/(b*b) + (z-z0)*(z-z0)/(c*c));
    //float err = (x-x_calc)*(x-x_calc) + (y-y_calc)*(y-y_calc) + (z-z_calc)*(z-z_calc);
    if (i == 5) Serial.printf("\n x5:%f, y5=%f, z5=%f, f=%f \n",x,y,z,err);
    
    error_sum += (err*err); 
    
  }

  //Serial.printf(" error_sum:%f \n",error_sum);

  return (error_sum/((float)terms)+fabs(a-b_earth_mag)+fabs(b-b_earth_mag)+fabs(c-b_earth_mag));
  //return (error_sum/((float)terms));
}

void set_sphere_grad(float *g, float *p_in, int n){

  float * x_buff = (float*) malloc(n * sizeof(float));
  float err_zero = get_sphere_error(p_in,n);

  Serial.printf("\neps: %f  x:%f %f %f ez:%f    ", eps,  x_buff[0],x_buff[1], x_buff[2], err_zero );

  for (int i = 0; i<n; i++){
    for (int j = 0; j<n; j++) { 
      if (j==i) x_buff[j] = p_in[j]+eps;
      else x_buff[j] = p_in[j];
      //Serial.printf("i:%d j:%d pin[j]:%f xbuff[j]:%f",i,j,p_in[j],x_buff[j]);
    }
    float err_x = get_sphere_error(x_buff,n);
    Serial.printf(" f%d:%f ",i,err_x);
    g[i] = (-err_zero + err_x)/eps;
  }

  //Serial.printf("\nerr_z:%f  G: %f %f %f  x:%f %f %f      %f    %f %f %f    %f\n",err_zero, g[0], g[1], g[2], x_buff[0],x_buff[1], x_buff[2], err_zero, err_x[0], );
  Serial.printf(" G: %f %f %f \n", g[0], g[1], g[2]);

  free(x_buff);
}


float get_sphere_error(float *p_in, int n) {
  // N=3 --> just set offset
  float error_sum = 0.0;
  if (n==3) {
    float x0 = p_in [0];
    float y0 = p_in [1];
    float z0 = p_in [2];
    float r = b_earth_mag;

    int terms = 0;
    for (int i = 0; i<pts; i+=o_step) {
    terms++;
      float x = scratch[pts*sensor_under_cal*3+i*3+0];
      float y = scratch[pts*sensor_under_cal*3+i*3+1];
      float z = scratch[pts*sensor_under_cal*3+i*3+2];
      float r_calc = sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0) + (z-z0)*(z-z0));
      float err = (r_calc-r)*(r_calc-r);
      if (i == -5) Serial.printf("\n x5:%f, y5=%f, z5=%f, f=%f \n",x,y,z,err);
      error_sum += (err*err); 
    }
    return (sqrt(error_sum)/((float)terms));
  }
  
  /*** N=4 --> offsets plus radius  ***/
  else if (n==4) {
    float x0 = p_in [0];
    float y0 = p_in [1];
    float z0 = p_in [2];
    float r = p_in[3];

    int terms = 0;
    for (int i = 0; i<pts; i+=o_step) {
    terms++;
      float x = scratch[pts*sensor_under_cal+i*3+0];
      float y = scratch[pts*sensor_under_cal+i*3+1];
      float z = scratch[pts*sensor_under_cal+i*3+2];
      float r_calc = sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0) + (z-z0)*(z-z0));
      float err = (r_calc-r)*(r_calc-r);
      if (i == -5) Serial.printf("\n x5:%f, y5=%f, z5=%f, f=%f \n",x,y,z,err);
      error_sum += (err*err); 
    }
    return (error_sum/((float)terms));
  }



}

void cal_sensors (void) {
  Serial.println("enter cal routine");
  int n_sensors = 6;

  //reset mag cal values
  for (int i=0; i<8; i++) {
    for (int j = 0; j<9; j++){
      mag_cals[i][j] = 0.0;
    }
  }

/****   Get sensor values, filter them, store in scratch  ****/
  Serial.println("cal values");
  for (int i = 0; i < pts; i++) {

    get_sensor_vals();
    filter_sensor_vals();
    // i 8 j 3 mfs[i][j] = ((1.0 - a) * vals[i][j] + a * mfs[i][j]) ;
    for (int j = 0; j < n_sensors; j++) {
      //float x = mfs[0][0];
      //float y = mfs[0][1];
      //float z = mfs[0][2];
      //Serial.printf("\nj:%d i:%d x:%f y:%f z:%f",j,i,x,y,z);
      //memcpy(&s1+i*4, &(mfs[0][0]),12);
      scratch[j * pts * 3 + i * 3 + 0] = mfs[j][0];
      scratch[j * pts * 3 + i * 3 + 1] = mfs[j][1];
      scratch[j * pts * 3 + i * 3 + 2] = mfs[j][2];
      //Serial.printf("\nj:%d i:%d x:%f y:%f z:%f", j, i, mfs[j][0], mfs[j][1], mfs[j][2]);
      delay(1);
      Serial.printf("%f %f %f ", mfs[j][0], mfs[j][1], mfs[j][2]);
    }
    Serial.print("\n");



  }

  // Go through magnetometers and calibrate
  int n = 3;
  float * ellipse_vals = (float*)malloc(n*sizeof(float));
  float * ellipse_grad = (float*)malloc(n*sizeof(float));
  int n_e = 3;
  float f_buff;
  float * sphere_vals = (float*)malloc(n_e*sizeof(float));
  cg_stats stats;
  for (int i = 0; i <n; i++) ellipse_vals[i] = 1.0;
  for (int i = 0; i <n; i++) ellipse_grad[i] = 2.0;
  for (int i = 0; i <n; i++) sphere_vals[i] = 1.0;
  
  for (uint8_t j = 0; j < n_sensors; j++) {
    f_buff = 1000.0;
    sensor_under_cal = j;
    //float test_error = get_ellipse_error(ellipse_vals,n);
    //set_ellipse_grad(ellipse_grad,ellipse_vals,6);
    for (int i=0; i<5; i++){
      //Serial.printf("\ngradi:%d val:%f",i,ellipse_grad[i]);
      f_buff = 100.0;
      eps = .1;
      }
      
      if (0) {
        cg_descent(ellipse_vals,6,&stats,NULL,1.e-7,get_ellipse_error, set_ellipse_grad, NULL,NULL);
        Serial.printf("\nEllipse values found:\nx0:%f,y0:%f,z0:%f,a:%f,b:%f,c:%f",ellipse_vals[3],ellipse_vals[4],ellipse_vals[5],ellipse_vals[0],ellipse_vals[1],ellipse_vals[2]);
        Serial.printf("\n:fval:%f eps:%f",stats.f, eps);
        
      }
      if (1) {
        while(f_buff>30.0) {

          float xmin, xmax, ymin, ymax, zmin, zmax = 0.0;
          for (int i = 0; i<pts; i+=o_step) {
            float x = scratch[pts*sensor_under_cal*3+i*3+0];
            float y = scratch[pts*sensor_under_cal*3+i*3+1];
            float z = scratch[pts*sensor_under_cal*3+i*3+2];
            if(x<xmin)xmin=x;
            if(x>xmax)xmax=x;
            if(y<ymin)ymin=y;
            if(y>ymax)ymax=y;
            if(z<zmin)zmin=z;
            if(z>zmax)zmax=z;
          }
            //float x_est = (xmin + xmax)/2.0;
            //float y_est = (ymin + ymax)/2.0;
            //float z_est = (zmin + zmax)/2.0;
            
            //Serial.printf("xmin:%f xmax:%f ymin:%f ymax:%f zmax:%f zmin:%f x_est:%f, y_est:%f, z_est:%f",xmin,xmax,ymin, ymax, zmin, zmax, x_est, y_est, z_est);

          //sphere_vals[0] = x_est;
          //sphere_vals[1] = y_est;
          //sphere_vals[2] = z_est;


          cg_descent(sphere_vals,3,&stats,NULL,1.e-7,get_sphere_error, set_sphere_grad, NULL,NULL);
          Serial.printf("\n sphere values found for sensor:%d \nx0:%f,y0:%f,z0:%f",j,sphere_vals[0],sphere_vals[1],sphere_vals[2]);
          Serial.printf("\n:fval:%f eps:%f",stats.f, eps);
          f_buff = stats.f;
          eps *= 0.1;
        }
        //EEPROM.put(j*n_sensors*3*4+0*4,sphere_vals[0]);
        //EEPROM.put(j*n_sensors*3*4+1*4,sphere_vals[1]);
        //EEPROM.put(j*n_sensors*3*4+2*4,sphere_vals[2]);

      mag_cals[j][0] = sphere_vals[0];
      mag_cals[j][1] = sphere_vals[1];
      mag_cals[j][2] = sphere_vals[2];
      }

      if (0) {
        mag_cals[j][0] = mfs[j][0];
        mag_cals[j][1] = mfs[j][1];
        mag_cals[j][2] = mfs[j][2];
      }




    //float x0 [6] = {1.0,1.0,1.0,10.0,10.0,10.0};
    //ellipse_error(x0,pts);

    
    Serial.print("\n");
  }
  free(ellipse_vals);
  free(ellipse_grad);


  delay(5000);

}





void setup()
{
  Serial.println("Setup started");
  pinMode(RESET_MC, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  digitalWrite(SW1, LOW);
  digitalWrite(SW2, LOW);
  digitalWrite(SW3, LOW);
  digitalWrite(RESET_MC, HIGH);

  Wire.begin();

  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
  uint16_t a13 = analogRead(A13);
  Serial.printf("\n reading: %d \n", a13);

  config_sensors();
  //get_lsm6ds3_mx();
  setup_filter();

  //test_opt();
  delay(5000);
  //connectToWiFi(networkName, networkPswd);

}

int oldmillis = 0;
void loop()
{

  int t0 = millis();
  int sw1_val;
  int sw2_val;
  int sw3_val;

  sw1_val = digitalRead(SW1);
  sw2_val = digitalRead(SW2);
  sw3_val = digitalRead(SW3);

  if (0) Serial.printf("\nSW1:%d SW2:%d SW3:%d\n", sw1_val, sw2_val, sw3_val);

  if (0) {
    if (sw1_val) {
      a = a + 0.001;
      Serial.printf("\na:%f\n", a * 100.0);
    }
    else if (sw3_val) {
      a = a - 0.001;
      Serial.printf("\na:%f\n", a * 100.0);
    }
    else {
      //Serial.print("\na:0.0\n");
    }
  }
  //get_mx_MMC();
  //get_measurements(john);
  //get_mx_QMC();


  // Get raw sensor values
  if (1) {
    get_sensor_vals();
  }

  // print raw measured values
  if (false) {
    Serial.println("new MX:\n");
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 8; i++) {
        float val = vals [i][j];
        if (val >= 0.0) Serial.print(" ");
        if (fabs(val) < 100) Serial.print(" ");
        if (fabs(val) < 10) Serial.print(" ");
        Serial.printf(" %f ", val);
      }
      Serial.print("\n");
    }
  }

  // check for calibration run input
  if (sw1_val && (run_mode == 0)) {
    run_mode = 1;
  }

  if (run_mode == 1) {
    cal_sensors();
    run_mode = 0;
  }



  // Filter values with simple IIR filter
  if (true) {
    filter_sensor_vals();
    //mfs[i][j] = (1.0-a) * vals[i][j] + a * mfs[i][j];

    //Serial.printf("\nin:%f out: %f     diff:%f    alpha:%f \n  ", vals[i][j],mfs[i][j], vals[i][j]-mfs[i][j], a);
    //ser
    //Serial.println

  }

  // Filter with filter library
  if (false) {
    /*
    float filtx = f2x.filterIn(vals[2][0]);
    float filty = f2x.filterIn(vals[2][1]);
    float filtz = f2x.filterIn(vals[2][2]);

    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);

    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);

    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
    filtx = f2x.filterIn(vals[2][0]);
    filty = f2x.filterIn(vals[2][1]);
    filtz = f2x.filterIn(vals[2][2]);
*/
    //Serial.printf("\nraw   x:%f    y:%f    z:%f \n filt: x:%f    y:%f    z:%f\n", vals[2][0], vals[2][1], vals[2][2], filtx, filty, filtz);
    /*
      for (int k=2; k<9; k++){
        filt(k, vals[k][0], vals[k][1], vals[k][2]);
        if (k>7) break;
        vals[k][0] = filt_block_xy[k][0][3];
        vals[k][1] = filt_block_xy[k][1][3];
        vals[k][2] = filt_block_xy[k][2][3];
      }
    */



    /*
        vals[3][0] = f3x.filterIn(vals[3][0]);
        vals[3][1] = f3x.filterIn(vals[3][1]);
        vals[3][2] = f3x.filterIn(vals[3][2]);

        vals[4][0] = f4x.filterIn(vals[4][0]);
        vals[4][1] = f4x.filterIn(vals[4][1]);
        vals[4][2] = f4x.filterIn(vals[4][2]);

        vals[5][0] = f5x.filterIn(vals[5][0]);
        vals[5][1] = f5x.filterIn(vals[5][1]);
        vals[5][2] = f5x.filterIn(vals[5][2]);

        vals[6][0] = f6x.filterIn(vals[6][0]);
        vals[6][1] = f6x.filterIn(vals[6][1]);
        vals[6][2] = f6x.filterIn(vals[6][2]);

        vals[7][0] = f7x.filterIn(vals[7][0]);
        vals[7][1] = f7x.filterIn(vals[7][1]);
        vals[7][2] = f7x.filterIn(vals[7][2]);
    */
  }

  // print filtered sensor values for all sensors
  if (false) {
    Serial.println("filtered vals:\n");
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 8; i++) {
        float val = mfs [i][j];
        if (val >= 0.0) Serial.print(" ");
        if (fabs(val) < 100) Serial.print(" ");
        if (fabs(val) < 10) Serial.print(" ");
        Serial.printf(" %f ", val);
      }
      Serial.print("\n");
    }
  }

  // print single sensor values
  if (true) {
    //Serial.println("filtered vals:\n");
    for (int i = 0; i<8; i++) {
    for (int j = 0; j < 3; j++) {
       //int i = 0;
        float val = mfs [i][j] - mag_cals[i][j];
        if (val >= 0.0) Serial.print("");
        if (fabs(val) < 100) Serial.print("");
        if (fabs(val) < 10) Serial.print("");
        Serial.printf(" %f ", val);

      //Serial.print("\n");
    }
    }
    Serial.print("\n");
  
  }



// x5:-131.968872, y5=160.097122, z5=-132.967499, f=1717246208.000000 
  // print loop run time
  if (false) {
    int newmillis = millis();
    Serial.printf("\nrun loop millis:%d\n", newmillis - oldmillis);
    oldmillis = newmillis;
  }

  delay(40);
}
