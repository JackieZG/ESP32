
#include "ICM42688.h"
#include "ArduinoLog.h"
// an ICM42688 object with the ICM42688 sensor on SPI bus 0 and chip select pin 10
ICM42688 IMU(SPI, 1);

/********************************************全局变量声明区**************************************************************************************/
#define FLOW_DATA_BUFFER_SIZE 10  //
typedef struct
{
  float Q_bias;   /*系统中在roll方向上的陀螺仪产生的漂移的过程噪声，可以通过调参获得最优值 */
  float Q_angle;  /*系统中在roll方向上的，陀螺仪在该时刻产生的过程噪声，可以通过调参获得最优值*/
  float Q_gyro;   /*系统中在roll方向上的陀螺仪产生的角度的过程噪声，可以通过调参获得最优值*/
  float R_angle;  /*系统中在roll方向上的陀螺仪产生的角度的测量噪声，用过0.3*/
  float dt;       /*卡尔曼滤波过程中计算间隔时间*/
  float PP[2][2]; /*卡尔曼滤波过程中，系统的协方差矩阵*/
  float K_0;      /*卡尔纳增益*/
  float K_1;      /*卡尔纳增益*/
  float angle;    /*角度*/
} KALMAN_FILTER_PARA;
KALMAN_FILTER_PARA KKF_pitch;

typedef struct
{
  int tremos_sensitivity;
  int measure_range;
  int sample_frequecy;
  int reading_count;
  int interval;
} GSENSOR_DRIVER_PARA;

typedef struct
{
  float acc_x;
  float acc_y;
  float acc_z;
} G_INFO;
G_INFO g_info;
G_INFO kg_info;
G_INFO kkg_info;
typedef struct
{
  float gyro_x;
  float gyro_y;
  float gyro_z;
} GY_INFO;
GY_INFO gy_info;
GY_INFO kgy_info;
typedef struct {
  float data[FLOW_DATA_BUFFER_SIZE];
  int start;  // 环形缓冲区的起始位置
  int end;    // 环形缓冲区的结束位置（下一个插入点的位置）
} CircularBuffer;

/*由于需要对数据进行二次处理，所以将其放进数据流里面进行处理*/
CircularBuffer acc_x_flow;
CircularBuffer acc_y_flow;
CircularBuffer acc_z_flow;
static float last_filtered_acc_x = 0;
static float last_filtered_acc_y = 0;
static float last_filtered_acc_z = 0;

#define ORDER 4
#define STRAIGHT_ANGLE 180

double b[ORDER + 1] = { 0.01020948, 0.04083792, 0.06125688, 0.04083792, 0.01020948 };
double a[ORDER + 1] = { 1.00000000, -1.96842779, 1.73586071, -0.72447083, 0.1203896 };

static double w_accx[ORDER + 1] = { 0 };  // Internal state array
static double w_accy[ORDER + 1] = { 0 };  // Internal state array
static double w_accz[ORDER + 1] = { 0 };  // Internal state array]

float gyro_x_before = 0;
float gyro_y_before = 0;
float gyro_z_before = 0;
float acc_x_before = 0;
float acc_y_before = 0;
float acc_z_before = 0;
static float angle_pitch_k;
static float acc_xyz_filtered;
static float angle_pitch_mmm;

/********************************************结束**************************************************************************************/
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



/**************************************************数据处理函数区域***********************************************************************/
KALMAN_FILTER_PARA kalman_filtering(float angle_m, float v, KALMAN_FILTER_PARA kfp) {
  /*如果没有角速度，便不进行先验估计了*/
  if (0 == v) {
    /*不做处理*/
  } else {
    /*1.先验估计的roll*/
    kfp.angle += (v - kfp.Q_bias) * kfp.dt;
    //log_d("%.4f,%.4f,%.4f,%.4f\n",kfp.angle,v,kfp.Q_bias,kfp.dt);
    /*2.更新系统误差矩阵*/
    kfp.PP[0][0] += (kfp.Q_angle - (kfp.PP[0][1] - kfp.PP[1][0])) * kfp.dt;
    kfp.PP[0][1] += -kfp.PP[1][1] * kfp.dt;
    kfp.PP[1][0] += -kfp.PP[1][1] * kfp.dt;
    kfp.PP[1][1] += kfp.Q_gyro;
    /*3.更新卡尔曼增益*/
    kfp.K_0 = kfp.PP[0][0] / kfp.PP[0][0] + kfp.R_angle;
    kfp.K_1 = kfp.PP[1][0] / kfp.PP[1][0] + kfp.R_angle;
  }

  /*4.后验估计*/
  kfp.angle += kfp.K_0 * (angle_m - kfp.angle);
  kfp.Q_bias += kfp.K_1 * (angle_m - kfp.angle);
  kfp.PP[0][0] -= kfp.K_0 * (angle_m - kfp.angle);
  kfp.PP[0][1] -= kfp.K_0 * (angle_m - kfp.angle);
  kfp.PP[1][0] -= kfp.K_1 * (angle_m - kfp.angle);
  kfp.PP[1][1] -= kfp.K_1 * (angle_m - kfp.angle);

  return kfp;
}

void init_flow_buffer_data(CircularBuffer *buffer) {
  buffer->start = 0;
  buffer->end = 0;
}

void init_flow_buffer() {
  init_flow_buffer_data(&acc_x_flow);
  init_flow_buffer_data(&acc_y_flow);
  init_flow_buffer_data(&acc_z_flow);
  KKF_pitch.dt = 0.009;       /*根据log，83hz，0.001,0.008,0.02(收敛快)*/
  KKF_pitch.Q_angle = 0.001;  //0.001
  KKF_pitch.Q_bias = 1.409976;
  KKF_pitch.Q_gyro = 0.025;
  KKF_pitch.R_angle = 0.000001332; /*测试过的数据有：0.3;gyro 3segma:0.000001332*/
  KKF_pitch.K_0 = 1.0f;
  KKF_pitch.K_1 = 1.0f;
  KKF_pitch.angle = 0.0f;
  KKF_pitch.PP[0][0] = 1.0f;
  KKF_pitch.PP[0][1] = 0.0f;
  KKF_pitch.PP[1][0] = 0.0f;
  KKF_pitch.PP[1][1] = 1.0f;
}

CircularBuffer add_to_buffer(CircularBuffer *buffer, float new_acc_x) {
  buffer->data[buffer->end] = new_acc_x;
  buffer->end = (buffer->end + 1) % FLOW_DATA_BUFFER_SIZE;
  if (buffer->end == buffer->start) {
    buffer->start = (buffer->start + 1) % FLOW_DATA_BUFFER_SIZE;  // 丢弃最老的数据点
  }
  return *buffer;
}

void read_data_flow(float acc_x, float acc_y, float acc_z) {
  acc_x_flow = add_to_buffer(&acc_x_flow, acc_x);
  acc_y_flow = add_to_buffer(&acc_y_flow, acc_y);
  acc_z_flow = add_to_buffer(&acc_z_flow, acc_z);
}

void processChunk(float *input, float *output, double *w) {

  int i, j;
  for (i = 0; i < FLOW_DATA_BUFFER_SIZE; i++) {
    double outputSample = 0;

    for (j = 0; j <= ORDER; j++) {
      if (i - j >= 0) {
        outputSample += b[j] * input[i - j];
        if (j > 0) {
          outputSample -= a[j] * w[j - 1];
        }
      } else {
        outputSample += b[j] * 0;
        if (j > 0) {
          outputSample -= a[j] * 0;
        }
      }
    }
    for (j = ORDER; j > 0; j--) {
      w[j] = w[j - 1];
    }
    w[0] = outputSample;
    output[i] = outputSample;  // Store the filtered sample in the output array
  }
}

void flow_data_process() {

  read_data_flow(kg_info.acc_x, kg_info.acc_y, kg_info.acc_z);

  float filtered_acc_x[FLOW_DATA_BUFFER_SIZE];
  float filtered_acc_y[FLOW_DATA_BUFFER_SIZE];
  float filtered_acc_z[FLOW_DATA_BUFFER_SIZE];

  processChunk(acc_x_flow.data, filtered_acc_x, w_accx);
  processChunk(acc_y_flow.data, filtered_acc_y, w_accy);
  processChunk(acc_z_flow.data, filtered_acc_z, w_accz);

  last_filtered_acc_x = filtered_acc_x[FLOW_DATA_BUFFER_SIZE - 1];
  last_filtered_acc_y = filtered_acc_y[FLOW_DATA_BUFFER_SIZE - 1];
  last_filtered_acc_z = filtered_acc_z[FLOW_DATA_BUFFER_SIZE - 1];
  kkg_info.acc_x = last_filtered_acc_x;
  kkg_info.acc_y = last_filtered_acc_y;
  kkg_info.acc_z = last_filtered_acc_z;

  /*姿态解算*/
  float acc_xy = sqrt(kkg_info.acc_y * kkg_info.acc_y + kkg_info.acc_z * kkg_info.acc_z);
  kkg_info.acc_x = 0 - kkg_info.acc_x;
  float angle_pitch_mmm = atan2(kkg_info.acc_x, acc_xy) * STRAIGHT_ANGLE / PI;
  kkg_info.acc_x = 0 - kkg_info.acc_x;

  KKF_pitch = kalman_filtering(angle_pitch_mmm, kgy_info.gyro_y, KKF_pitch);

  angle_pitch_k = -KKF_pitch.angle;

  if (angle_pitch_k > 180) {
    angle_pitch_k -= 360;
  } else if (angle_pitch_k < -180) {
    angle_pitch_k += 360;
  } else {
    /*数值计算错误*/
  }
}



int get_angle_pitch_k(void) {
  int angle = para_float_to_int_with_two_decimals(angle_pitch_k * 0.01);
  return angle;
}

int get_last_filtered_acc_x() {
  int acc = para_float_to_int_with_two_decimals(last_filtered_acc_x);
  return acc;
}
int get_last_filtered_acc_y() {
  int acc = para_float_to_int_with_two_decimals(last_filtered_acc_y);
  return acc;
}
int get_last_filtered_acc_z() {
  int acc = para_float_to_int_with_two_decimals(last_filtered_acc_z);
  return acc;
}
int get_filtered_accxyz() {
  int acc = acc_xyz_filtered;
  return acc;
}

int get_before_gyro_x() {
  int gyro = para_float_to_int_with_two_decimals(gyro_x_before);
  return gyro;
}

int get_before_gyro_y() {
  int gyro = para_float_to_int_with_two_decimals(gyro_y_before);
  return gyro;
}

int get_before_gyro_z() {
  int gyro = para_float_to_int_with_two_decimals(gyro_z_before);
  return gyro;
}

int get_before_acc_x() {
  int acc = para_float_to_int_with_two_decimals(acc_x_before);
  return acc;
}

int get_before_acc_y() {
  int acc = para_float_to_int_with_two_decimals(acc_y_before);
  return acc;
}

int get_before_acc_z() {
  int acc = para_float_to_int_with_two_decimals(acc_z_before);
  return acc;
}

int get_before_acc_xyz() {
  int acc = acc_x_before * acc_x_before + acc_y_before * acc_y_before + acc_z_before * acc_z_before;
  return acc;
}
static int para_float_to_int_with_two_decimals(float f) {
  f = f * 10000;      // 将浮点数乘以10000，扩大四个数量级，保留小数点后四位小数
  f = (int)round(f);  // 使用round函数进行四舍五入，然后转换为整数
  return f;
}

/************************************************************************************************************************************/



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
