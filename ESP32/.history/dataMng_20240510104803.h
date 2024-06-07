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