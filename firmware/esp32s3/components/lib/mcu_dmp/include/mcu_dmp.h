#ifndef MCU_DMP_H
#define MCU_DMP_H

#include <math.h>
#include <stdint.h>
#include "stdio.h"

/* ���峣�� */
#define DEG2RAD		0.017453293f
#define RAD2DEG		57.29578f
#define ABS(x) 		(((x) < 0) ? (-x) : (x))

/* ����3�����ݵĽṹ�� */
typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

/* ����ŷ���ǵĽṹ�� */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

/* ���忨�����˲����ṹ�� */
typedef struct {
    float q;     // ��������
    float r;     // ��������
    float x;     // ״̬����
    float p;     // �������Э����
    float k;     // ����������
    uint8_t init;// ��ʼ����־
} KalmanFilter;

typedef struct {
    float x;     // ״̬����
    float p;     // �������Э����
    float q;     // ��������
    float r;     // ��������
    float k;     // ����������
    float v;     // ��������
    float s;     // ����Э����
    float r0;    // ������������
    float window[10]; // ���´���
    int window_idx;   // ��������
    int init;    // ��ʼ����־
} AdaptiveKalmanFilter;

/* �������� */
void imu_init(void);
void imu_update(Axis3f acc, Axis3f gyro, float dt);
EulerAngles imu_get_euler_angles(Axis3f gyro);
float invSqrt(float x);
float kalman_filter(KalmanFilter* kf, float measurement, float gyro_rate, float dt);


#endif 

