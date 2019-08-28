#ifndef filtro_h
#define filtro_h

#include <cmath>
#include <stdio.h>
#include <memory>

#ifndef _INERTIAL_SENSOR_H
#define _INERTIAL_SENSOR_H

class InertialSensor {
public:
    virtual bool initialize() = 0;
    virtual bool probe() = 0;
    virtual void update() = 0;

    float read_temperature() {return temperature;};
    void read_accelerometer(float *ax, float *ay, float *az) {*ax = _ax; *ay = _ay; *az = _az;};
    void read_gyroscope(float *gx, float *gy, float *gz) {*gx = _gx; *gy = _gy; *gz = _gz;};
    void read_magnetometer(float *mx, float *my, float *mz) {*mx = _mx; *my = _my; *mz = _mz;};

protected:
    float temperature;
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _mx, _my, _mz;
};

#endif //_INERTIAL_SENSOR_H

class AHRS{
private:
    float q0, q1, q2, q3;
    float gyroOffset[3];
    float twoKi;
    float twoKp;
    float integralFBx, integralFBy, integralFBz;
public:
    AHRS();

    void sensorinit(void);
    void update(float dt);
    void updateIMU(float dt);
    void setGyroOffset();
    void getEuler(float* roll, float* pitch, float* yaw);

    float invSqrt(float x);
    float getW();
    float getX();
    float getY();
    float getZ();
};

#endif 