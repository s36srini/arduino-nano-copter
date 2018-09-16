#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int Tmp,GyX_vel,GyY_vel,GyZ_vel;

long GyX_offset, GyY_offset, GyZ_offset;
long AcX,AcY,AcZ;

float angle_pitch, angle_roll, angle_pitch_output, angle_roll_output;

long loop_timer; //Timer to keep track of loop time (4000us)

boolean set_gyro_angles = false; //Don't set the gyro angles from the gyro upon the first iteration, 
                              //set it from the calculated angles from the accelerometer.

const int SAMPLING_NUM = 1000; // Number of samples to seek average offset in order to correct gyro offsets.


void read_raw_values() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,14);  // request a total of 14 registers
  while(Wire.available() < 14); // Wait until all bytes are received
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX_vel=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY_vel=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ_vel=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void setup_offsets() {
  for(int i = 0; i < SAMPLING_NUM; ++i) {
    read_raw_values();
    
    GyX_offset += GyX_vel;
    GyY_offset += GyY_vel;
    GyZ_offset += GyZ_vel;
    delay(3);

  }

  GyX_offset /= SAMPLING_NUM;
  GyY_offset /= SAMPLING_NUM;
  GyZ_offset /= SAMPLING_NUM;
}

void setup_mpu_6050_registers() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission();

  // Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU_addr);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();
                                               
  // Configure the gyro (500dps full scale)
  Wire.beginTransmission(MPU_addr);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
  
}

void setup() {
  // Setup serial interface.
  Wire.begin();

  setup_mpu_6050_registers();
  setup_offsets();
  
  Serial.begin(115200);
  loop_timer = micros();
}

void loop() {
  read_raw_values();
  // Subtract the offset from the raw gyro velocity values
  GyX_vel -= GyX_offset;
  GyY_vel -= GyY_offset;
  GyZ_vel -= GyZ_offset;

  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += GyX_vel * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += GyY_vel * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(GyZ_vel * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_roll -= angle_pitch * sin(GyZ_vel * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angle

  //Accelerometer angle calculations
  long acc_total_vector = sqrt((AcX*AcX)+(AcY*AcY)+(AcZ*AcZ));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  float angle_pitch_acc = asin((float)AcY/acc_total_vector)* 57.296;       //Calculate the pitch angle from the acceleration
  float angle_roll_acc = asin((float)AcX/acc_total_vector)* -57.296;       //Calculate the roll angle from the acceleration

  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  Serial.print(" Angle Pitch  = "); Serial.print(angle_pitch_output);
  Serial.print(" | Angle Roll = "); Serial.println(angle_roll_output);

  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();//Reset the loop timer

  
}
