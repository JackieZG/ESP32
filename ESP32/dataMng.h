#ifndef DATA_MNG_H
#define DATA_MNG_H

#include <Arduino.h>
#include "math.h"
#include <ICM42688.h>
#include <Wire.h>

/********************************************全局变量声明区**************************************************************************************/
#define FLOW_DATA_BUFFER_SIZE 10  //
#define ORDER 4
#define STRAIGHT_ANGLE 180
#define PI 3.14

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

typedef struct
{
  float gyro_x;
  float gyro_y;
  float gyro_z;
} GY_INFO;

typedef struct {
  float data[FLOW_DATA_BUFFER_SIZE];
  int start;  // 环形缓冲区的起始位置
  int end;    // 环形缓冲区的结束位置（下一个插入点的位置）
} CircularBuffer;

extern ICM42688 IMU;  // 声明，不定义
extern KALMAN_FILTER_PARA KKF_pitch;  // 声明，不定义
/********************************************结束**************************************************************************************/
KALMAN_FILTER_PARA kalman_filtering(float angle_m, float v, KALMAN_FILTER_PARA kfp) ;
void init_flow_buffer_data(CircularBuffer *buffer) ;
void init_flow_buffer() ;
CircularBuffer add_to_buffer(CircularBuffer *buffer, float new_acc_x) ;
void read_data_flow(float acc_x, float acc_y, float acc_z);
void processChunk(float *input, float *output, double *w);
void flow_data_process() ;
int get_angle_pitch_k(void) ;
int get_last_filtered_acc_x();
int get_last_filtered_acc_y() ;
int get_last_filtered_acc_z() ;
int get_filtered_accxyz() ;
int get_before_gyro_x() ;
int get_before_gyro_y() ;
int get_before_gyro_z();
int get_before_acc_x();
int get_before_acc_y() ;
int get_before_acc_z();
int get_before_acc_xyz() ;
void read_init_data();
void log_kf_para();
#endif
