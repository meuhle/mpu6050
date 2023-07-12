#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float norm_g;
float acceleration[3];
int32_t rotatio[3];
int32_t rotation[3];
bool first = true;
float RwEst[3];
float RwGyro[3];
long int old_t = 0;
float Awz[2];
int wGyro = 10;

void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); 

    
   calculate_IMU_error();
  delay(20);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
   Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  //
  long int current_t = millis();
  long int t = current_t-old_t;
  old_t = current_t;
  // Calculating Roll and Pitch from the accelerometer data
  norm_g =  sqrt(AccX*AccX + AccY*AccY+AccZ*AccZ);
  acceleration[0] = AccX / norm_g;
  acceleration[1]= AccY/norm_g;
  acceleration[2]=  AccZ/norm_g;
  //accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  //accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
 Serial.print("ERROR X: " + String(GyroErrorX));
 Serial.print("ERROR Y: " +String(GyroErrorY));
 Serial.print("ERROR Z: " +String(GyroErrorZ));
  // Correct the outputs with the calculated error values
  /*rotation[0] = GyroX +GyroErrorX; // GyroErrorX ~(-0.56)
  rotation[1] = GyroY + GyroErrorY; // GyroErrorY ~(2)
  rotation[2] = GyroZ +GyroErrorZ; // GyroErrorZ ~ (-0.8)*/
  rotation[0] = GyroX; // GyroErrorX ~(-0.56)
  rotation[1] = GyroY ; // GyroErrorY ~(2)
  rotation[2] = GyroZ;

    if (first){
    for(int w=0;w<=2;w++) RwEst[w] = acceleration[w]; 
    old_t = millis();
    first = false;
  }
  else
  {
    if(abs(RwEst[2]) < 0.01){
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for(int w=0;w<=2;w++) RwGyro[w] = RwEst[w];
    }
     else
    {
      //Serial.print("is doing");
      Awz[0] = atan2(RwEst[0],RwEst[2]) * 180 / PI;   //get angle and convert to degrees        
      Awz[0] += ((rotation[1])*(t/1000) );                     //get updated angle according to gyro movement

      Awz[1] = atan2(RwEst[1],RwEst[2]) * 180 / PI;   //get angle and convert to degrees        
      Awz[1] += ((rotation[0])*(t/1000));                     //get updated angle according to gyro movement

    }

  float signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
  RwGyro[0] = sin(Awz[0] * PI / 180);
    RwGyro[0] /= sqrt( 1 + pow(cos(Awz[0] * PI / 180),2) * pow((tan(Awz[1] * PI / 180)),2 ));
    RwGyro[1] = sin(Awz[1] * PI / 180);
    RwGyro[1] /= sqrt( 1 + pow(cos(Awz[1] * PI / 180),2) * pow((tan(Awz[0] * PI / 180)),2 ));

    RwGyro[2] = signRzGyro * sqrt(1 - pow(RwGyro[0],2) - pow(RwGyro[1],2));
    //combinazione acc + gyro
    for(int w=0;w<=2;w++) RwEst[w] = (acceleration[w]+RwGyro[w]*wGyro)/(1+wGyro);
    //normalizzazione
    float R = sqrt(RwEst[0]*RwEst[0]+RwEst[1]*RwEst[1]+RwEst[2]*RwEst[2]);
    for(int w=0;w<=2;w++) RwEst[w] /= R;
    
  }
  Serial.print("\nRxEst : ");Serial.print(RwEst[0]);Serial.print("\n");
  Serial.print("RyEst : ");Serial.print(RwEst[1]*-1);Serial.print("\n");
  }
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  //gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  //gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  //yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  //roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  //pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
  /*Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);*/



void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
