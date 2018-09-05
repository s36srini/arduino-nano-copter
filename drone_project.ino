
#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int16_t GyX_offset, GyY_offset, GyZ_offset;

const int SAMPLING_NUM = 100; // Number of samples to seek average offset in order to correct gyro offsets.

// Setup I2C using 0x68 and starting read register.
void configure_read_imu() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
}

void read_raw_values() {
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void setup_offsets() {
  int16_t GyX_sum = 0;
  int16_t GyY_sum = 0;
  int16_t GyZ_sum = 0;

  for(int i = 0; i < SAMPLING_NUM; ++i) {
    configure_read_imu();
    read_raw_values();
    
    GyX_sum += GyX;
    GyY_sum += GyY;
    GyZ_sum += GyZ;

  }

  GyX_offset = (GyX_sum / SAMPLING_NUM);
  GyY_offset = (GyY_sum / SAMPLING_NUM);
  GyZ_offset = (GyZ_sum / SAMPLING_NUM);
}

void setup() {
  // Setup serial interface.
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  setup_offsets();
  
}

void loop() {

  configure_read_imu();
  read_raw_values();
  
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX - GyX_offset);
  Serial.print(" | GyY = "); Serial.print(GyY - GyY_offset);
  Serial.print(" | GyZ = "); Serial.println(GyZ - GyZ_offset);
  delay(333);
  
}
