#include "dataMng.h"
#include <cmath>
#include <vector>
#include <iostream>

class SensorDataProcessor {
public:
    SensorDataProcessor(int size) : acc_x_flow(size), acc_y_flow(size), acc_z_flow(size) {}

    void readInitData(ICM42688& imu) {
        g_info.acc_x = imu.accX();
        g_info.acc_y = imu.accY();
        g_info.acc_z = imu.accZ();
        gy_info.gyro_x = imu.gyrX();
        gy_info.gyro_y = imu.gyrY();
        gy_info.gyro_z = imu.gyrZ();
    }

    void processData();

private:
    G_INFO g_info;
    G_INFO kg_info;
    G_INFO kkg_info;

    GY_INFO gy_info;
    GY_INFO kgy_info;

    std::vector<float> acc_x_flow;
    std::vector<float> acc_y_flow;
    std::vector<float> acc_z_flow;

    float last_filtered_acc_x = 0;
    float last_filtered_acc_y = 0;
    float last_filtered_acc_z = 0;

    double b[5] = {0.01020948, 0.04083792, 0.06125688, 0.04083792, 0.01020948};
    double a[5] = {1.00000000, -1.96842779, 1.73586071, -0.72447083, 0.1203896};

    double w_accx[5] = {0};  // Internal state array
    double w_accy[5] = {0};
    double w_accz[5] = {0};

    float gyro_x_before = 0;
    float gyro_y_before = 0;
    float gyro_z_before = 0;
    float acc_x_before = 0;
    float acc_y_before = 0;
    float acc_z_before = 0;
    float angle_pitch_k;
    float acc_xyz_filtered;
    float angle_pitch_mmm;

    void processChunk(float *input, float *output, double *w, int size);
};

void SensorDataProcessor::processChunk(float *input, float *output, double *w, int size) {
    for (int i = 0; i < size; ++i) {
        double outputSample = 0;
        for (int j = 0; j <= 4; ++j) {
            if (i - j >= 0) {
                outputSample += b[j] * input[i - j];
                if (j > 0) outputSample -= a[j] * w[j - 1];
            }
        }
        for (int j = 4; j > 0; --j) w[j] = w[j - 1];
        w[0] = outputSample;
        output[i] = outputSample;  // Store the filtered sample in the output array
    }
}

void SensorDataProcessor::processData() {
    // Example of data processing method
}

// Usage
int calculat_data() {
    ICM42688 imu(SPI, 5);
    SensorDataProcessor processor(100);  // Assume buffer size is 100
    processor.readInitData(imu);
    processor.processData();
    return 0;
}
