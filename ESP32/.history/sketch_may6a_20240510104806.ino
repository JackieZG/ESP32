
#include "ICM42688.h"
#include "ArduinoLog.h"
// an ICM42688 object with the ICM42688 sensor on SPI bus 0 and chip select pin 10
ICM42688 IMU(SPI, 1);


void filter() {
  int status;
  status = IMU.setFilters(true, true);
  if (status > 0) {
    Serial.println("set filter success");
  }
}

void read_calibration() {
  float gxb;
  float gyb;
  float gzb;
  float axb;
  float ayb;
  float azb;

  gxb = IMU.getGyroBiasX();
  gyb = IMU.getGyroBiasY();
  gzb = IMU.getGyroBiasZ();
  axb = IMU.getAccelBiasX_mss();
  ayb = IMU.getAccelBiasX_mss();
  azb = IMU.getAccelBiasX_mss();

  Serial.print(",bais_gx:");
  Serial.print(gxb, 6);
  Serial.print(",bais_gy:");
  Serial.print(gyb, 6);
  Serial.print(",bais_gz:");
  Serial.print(gzb, 6);

  Serial.print("\n");
  IMU.setGyroBiasX(gxb);
  IMU.setGyroBiasY(gyb);
  IMU.setGyroBiasZ(gzb);
  Serial.print(",bais_gxed:");
  Serial.print(gxb, 6);
  Serial.print(",bais_gyed:");
  Serial.print(gyb, 6);
  Serial.print(",bais_gzed:");
  Serial.print(gzb, 6);

  Serial.println("\n set the bias of gyro successfully\n");
  read_data();
}

void execute_callibration() {
  int gb_status;
  int ab_status;
  gb_status = IMU.calibrateGyro();
  ab_status = IMU.calibrateAccel();
  Serial.print(",result of calibrate gyro: ");
  Serial.print(gb_status, 6);
  Serial.print(",result of calibrate acc: ");
  Serial.print(ab_status, 6);
}

void read_data() {
  Serial.print(",ax:");
  Serial.print(IMU.accX(), 6);
  Serial.print(",ay:");
  Serial.print(IMU.accY(), 6);
  Serial.print(",az:");
  Serial.print(IMU.accZ(), 6);
  Serial.print(",gx:");
  Serial.print(IMU.gyrX(), 6);
  Serial.print(",gy:");
  Serial.print(IMU.gyrY(), 6);
  Serial.print(",gz:");
  Serial.print(IMU.gyrZ(), 6);
  Serial.print(",tm:");
  Serial.println(IMU.temp(), 6);
}

void read_init_data() {
  g_info.acc_x = IMU.accX();
  g_info.acc_y = IMU.accY();
  g_info.acc_z = IMU.accZ();
  gy_info.gyro_x = IMU.gyrX();
  gy_info.gyro_y = IMU.gyrY();
  gy_info.gyro_z = IMU.gyrZ();
}






/*************************************************ino函数的使用******************************************************************************/

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
  Serial.print("\n");

  filter();
}

void loop() {
  // read the sensor
  IMU.getAGT();
  // display the data
  read_calibration();
  read_init_data();
  flow_data_process();

  delay(100);
}
