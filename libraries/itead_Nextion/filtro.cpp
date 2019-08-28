#include <stdio.h>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>

/*
Written by Qiyong Mu (kylongmu@msn.com)
Adapted for Raspberry Pi by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
*/

#ifndef _MPU9250_H
#define _MPU9250_H

/*
SPIDev driver code is placed under the BSD license.
Written by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
Copyright (c) 2014, Emlid Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _SPIDEV_H_
#define _SPIDEV_H_

//#define _XOPEN_SOURCE 600
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

class SPIdev {
public:
    SPIdev()
    {
    }

    static int transfer(const char *spidev,
                        unsigned char *tx,
                        unsigned char *rx,
                        unsigned int length,
                        unsigned int speed_hz = 1000000,
                        unsigned char bits_per_word = 8,
                        unsigned short delay_usecs = 0)
    {
        spi_ioc_transfer spi_transfer;
        
        memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

        spi_transfer.tx_buf = (unsigned long)tx;
        spi_transfer.rx_buf = (unsigned long)rx;
        spi_transfer.len = length;
        spi_transfer.speed_hz = speed_hz;
        spi_transfer.bits_per_word = bits_per_word;
        spi_transfer.delay_usecs = delay_usecs;

        int spi_fd = ::open(spidev, O_RDWR);

        if (spi_fd < 0 ) {
            printf("Error: Can not open SPI device\n");
            
            return -1;
        }

        int status = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

        ::close(spi_fd);

        // Debug information
        /*
        printf("Tx: ");
        for (int i = 0; i < length; i++)
            printf("%x ", tx[i]);
        printf("\n");

        printf("Rx: ");
        for (int i = 0; i < length; i++)
            printf("%x ", rx[i]);
        printf("\n");
        */

        return status;
    }
};

#endif //_SPIDEV_H_

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



class MPU9250 : public InertialSensor
{
public:
    MPU9250();

    bool initialize();
    bool probe();
    void update();

private:
    unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData);
    unsigned int ReadReg(uint8_t ReadAddr);
    void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes);

    unsigned int set_gyro_scale(int scale);
    unsigned int set_acc_scale(int scale);

    void calib_acc();
    void calib_mag();

    float acc_divider;
    float gyro_divider;

    int calib_data[3];
    float magnetometer_ASA[3];
};

#endif //_MPU9250_H

// MPU9250 registers
#define MPUREG_XG_OFFS_TC          0x00
#define MPUREG_YG_OFFS_TC          0x01
#define MPUREG_ZG_OFFS_TC          0x02
#define MPUREG_X_FINE_GAIN         0x03
#define MPUREG_Y_FINE_GAIN         0x04
#define MPUREG_Z_FINE_GAIN         0x05
#define MPUREG_XA_OFFS_H           0x06
#define MPUREG_XA_OFFS_L           0x07
#define MPUREG_YA_OFFS_H           0x08
#define MPUREG_YA_OFFS_L           0x09
#define MPUREG_ZA_OFFS_H           0x0A
#define MPUREG_ZA_OFFS_L           0x0B
#define MPUREG_PRODUCT_ID          0x0C
#define MPUREG_SELF_TEST_X         0x0D
#define MPUREG_SELF_TEST_Y         0x0E
#define MPUREG_SELF_TEST_Z         0x0F
#define MPUREG_SELF_TEST_A         0x10
#define MPUREG_XG_OFFS_USRH        0x13
#define MPUREG_XG_OFFS_USRL        0x14
#define MPUREG_YG_OFFS_USRH        0x15
#define MPUREG_YG_OFFS_USRL        0x16
#define MPUREG_ZG_OFFS_USRH        0x17
#define MPUREG_ZG_OFFS_USRL        0x18
#define MPUREG_SMPLRT_DIV          0x19
#define MPUREG_CONFIG              0x1A
#define MPUREG_GYRO_CONFIG         0x1B
#define MPUREG_ACCEL_CONFIG        0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG         0x37
#define MPUREG_INT_ENABLE          0x38
#define MPUREG_ACCEL_XOUT_H        0x3B
#define MPUREG_ACCEL_XOUT_L        0x3C
#define MPUREG_ACCEL_YOUT_H        0x3D
#define MPUREG_ACCEL_YOUT_L        0x3E
#define MPUREG_ACCEL_ZOUT_H        0x3F
#define MPUREG_ACCEL_ZOUT_L        0x40
#define MPUREG_TEMP_OUT_H          0x41
#define MPUREG_TEMP_OUT_L          0x42
#define MPUREG_GYRO_XOUT_H         0x43
#define MPUREG_GYRO_XOUT_L         0x44
#define MPUREG_GYRO_YOUT_H         0x45
#define MPUREG_GYRO_YOUT_L         0x46
#define MPUREG_GYRO_ZOUT_H         0x47
#define MPUREG_GYRO_ZOUT_L         0x48
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_EXT_SENS_DATA_01    0x4A
#define MPUREG_EXT_SENS_DATA_02    0x4B
#define MPUREG_EXT_SENS_DATA_03    0x4C
#define MPUREG_EXT_SENS_DATA_04    0x4D
#define MPUREG_EXT_SENS_DATA_05    0x4E
#define MPUREG_EXT_SENS_DATA_06    0x4F
#define MPUREG_EXT_SENS_DATA_07    0x50
#define MPUREG_EXT_SENS_DATA_08    0x51
#define MPUREG_EXT_SENS_DATA_09    0x52
#define MPUREG_EXT_SENS_DATA_10    0x53
#define MPUREG_EXT_SENS_DATA_11    0x54
#define MPUREG_EXT_SENS_DATA_12    0x55
#define MPUREG_EXT_SENS_DATA_13    0x56
#define MPUREG_EXT_SENS_DATA_14    0x57
#define MPUREG_EXT_SENS_DATA_15    0x58
#define MPUREG_EXT_SENS_DATA_16    0x59
#define MPUREG_EXT_SENS_DATA_17    0x5A
#define MPUREG_EXT_SENS_DATA_18    0x5B
#define MPUREG_EXT_SENS_DATA_19    0x5C
#define MPUREG_EXT_SENS_DATA_20    0x5D
#define MPUREG_EXT_SENS_DATA_21    0x5E
#define MPUREG_EXT_SENS_DATA_22    0x5F
#define MPUREG_EXT_SENS_DATA_23    0x60
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL           0x6A
#define MPUREG_PWR_MGMT_1          0x6B
#define MPUREG_PWR_MGMT_2          0x6C
#define MPUREG_BANK_SEL            0x6D
#define MPUREG_MEM_START_ADDR      0x6E
#define MPUREG_MEM_R_W             0x6F
#define MPUREG_DMP_CFG_1           0x70
#define MPUREG_DMP_CFG_2           0x71
#define MPUREG_FIFO_COUNTH         0x72
#define MPUREG_FIFO_COUNTL         0x73
#define MPUREG_FIFO_R_W            0x74
#define MPUREG_WHOAMI              0x75
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             0x0c  // should return 0x18
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09

// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F

// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

// Configuration bits MPU9250
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG                   0x80

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB

#define Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)


/*
  Written by Alexey Bulatov (alexey.bulatov@emlid.com) for Raspberry Pi
*/

#ifndef _LSM9DS1_H
#define _LSM9DS1_H

#include <stdint.h>

/*
SPIDev driver code is placed under the BSD license.
Written by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
Copyright (c) 2014, Emlid Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _SPIDEV_H_
#define _SPIDEV_H_

//#define _XOPEN_SOURCE 600
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

class SPIdev {
public:
    SPIdev()
    {
    }

    static int transfer(const char *spidev,
                        unsigned char *tx,
                        unsigned char *rx,
                        unsigned int length,
                        unsigned int speed_hz = 1000000,
                        unsigned char bits_per_word = 8,
                        unsigned short delay_usecs = 0)
    {
        spi_ioc_transfer spi_transfer;
        
        memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

        spi_transfer.tx_buf = (unsigned long)tx;
        spi_transfer.rx_buf = (unsigned long)rx;
        spi_transfer.len = length;
        spi_transfer.speed_hz = speed_hz;
        spi_transfer.bits_per_word = bits_per_word;
        spi_transfer.delay_usecs = delay_usecs;

        int spi_fd = ::open(spidev, O_RDWR);

        if (spi_fd < 0 ) {
            printf("Error: Can not open SPI device\n");
            
            return -1;
        }

        int status = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

        ::close(spi_fd);

        // Debug information
        /*
        printf("Tx: ");
        for (int i = 0; i < length; i++)
            printf("%x ", tx[i]);
        printf("\n");

        printf("Rx: ");
        for (int i = 0; i < length; i++)
            printf("%x ", rx[i]);
        printf("\n");
        */

        return status;
    }
};

#endif //_SPIDEV_H_


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



class LSM9DS1 : public InertialSensor
{
public:
    LSM9DS1();

    bool initialize();
    bool probe();
    void update();

private:
    unsigned int WriteReg(const char *dev, uint8_t WriteAddr, uint8_t WriteData);
    unsigned int ReadReg(const char *dev, uint8_t ReadAddr);
    void ReadRegs(const char *dev, uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes);

    void set_gyro_scale(int scale);
    void set_acc_scale(int scale);
    void set_mag_scale(int scale);

    void rotate();

    float gyro_scale;
    float acc_scale;
    float mag_scale;
};

#endif //_LSM9DS1_H

// who am I values
#define WHO_AM_I_ACC_GYRO           0x68
#define WHO_AM_I_MAG                0x3D

// Accelerometer and Gyroscope registers
#define LSM9DS1XG_ACT_THS           0x04
#define LSM9DS1XG_ACT_DUR           0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37

// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should return 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

// Configuration bits Accelerometer and Gyroscope
#define BITS_XEN_G                  0x08
#define BITS_YEN_G                  0x10
#define BITS_ZEN_G                  0x20
#define BITS_XEN_XL                 0x08
#define BITS_YEN_XL                 0x10
#define BITS_ZEN_XL                 0x20
#define BITS_ODR_G_14900mHZ         0x20
#define BITS_ODR_G_59500mHZ         0x40
#define BITS_ODR_G_119HZ            0x60
#define BITS_ODR_G_238HZ            0x80
#define BITS_ODR_G_476HZ            0xA0
#define BITS_ODR_G_952HZ            0xC0
#define BITS_ODR_XL_10HZ            0x20
#define BITS_ODR_XL_50HZ            0x40
#define BITS_ODR_XL_119HZ           0x60
#define BITS_ODR_XL_238HZ           0x80
#define BITS_ODR_XL_476HZ           0xA0
#define BITS_ODR_XL_952HZ           0xC0
#define BITS_FS_G_MASK              0xE3
#define BITS_FS_G_245DPS            0x00
#define BITS_FS_G_500DPS            0x08
#define BITS_FS_G_2000DPS           0x18
#define BITS_FS_XL_MASK             0xE7
#define BITS_FS_XL_2G               0x00
#define BITS_FS_XL_4G               0x10
#define BITS_FS_XL_8G               0x18
#define BITS_FS_XL_16G              0x08

// Configuration bits Magnetometer
#define BITS_TEMP_COMP              0x80
#define BITS_OM_LOW                 0x00
#define BITS_OM_MEDIUM              0x20
#define BITS_OM_HIGH                0x40
#define BITS_OM_ULTRA_HIGH          0x60
#define BITS_ODR_M_625mHZ           0x00
#define BITS_ODR_M_1250mHZ          0x04
#define BITS_ODR_M_250mHZ           0x08
#define BITS_ODR_M_5HZ              0x0C
#define BITS_ODR_M_10HZ             0x10
#define BITS_ODR_M_20HZ             0x14
#define BITS_ODR_M_40HZ             0x18
#define BITS_ODR_M_80HZ             0x1C
#define BITS_FS_M_MASK              0x0C
#define BITS_FS_M_4Gs               0x00
#define BITS_FS_M_8Gs               0x20
#define BITS_FS_M_12Gs              0x40
#define BITS_FS_M_16Gs              0x60
#define BITS_MD_CONTINUOUS          0x00
#define BITS_MD_SINGLE              0x01
#define BITS_MD_POWERDOWN           0x02
#define BITS_OMZ_LOW                0x00
#define BITS_OMZ_MEDIUM             0x04
#define BITS_OMZ_HIGH               0x08
#define BITS_OMZ_ULTRA_HIGH         0x0C

#pragma once

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#define NAVIO2 3
#define NAVIO 1

int write_file(const char *path, const char *fmt, ...);
int read_file(const char *path, const char *fmt, ...);
bool check_apm();
int get_navio_version();


#include "filtro.h"

#define G_SI 9.80665
#define PI   3.14159

MPU9250 mpu;
LSM9DS1 lsm;



AHRS::AHRS()
{
    q0 = 1; q1 = 0; q2 = 0, q3 = 0; twoKi = 0; twoKp =2;
}

void AHRS::sensorinit(void)
{
	mpu.initialize();
}

void AHRS::update(float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    mpu.update();
    mpu.read_accelerometer(&ax, &ay, &az);
    mpu.read_gyroscope(&gx, &gy, &gz);
    mpu.read_magnetometer(&mx, &my, &mz);


    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);		// pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRS::updateIMU(float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float ax, ay, az;
    float gx, gy, gz;

    // Accel + gyro.
    mpu.update();
    mpu.read_accelerometer(&ax, &ay, &az);
    mpu.read_gyroscope(&gx, &gy, &gz);

    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx *= (180 / PI) * 0.0175;
    gy *= (180 / PI) * 0.0175;
    gz *= (180 / PI) * 0.0175;

    gx -= gyroOffset[0];
    gy -= gyroOffset[1];
    gz -= gyroOffset[2];

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);		// pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRS::setGyroOffset()
{
    //---------------------- Calculate the offset -----------------------------

    float offset[3] = {0.0, 0.0, 0.0};
    float gx, gy, gz;

    //----------------------- MPU initialization ------------------------------

    mpu.initialize();

    //-------------------------------------------------------------------------

    //printf("Beginning Gyro calibration...\n");
    for(int i = 0; i<100; i++)
    {
        mpu.update();
        mpu.read_gyroscope(&gx, &gy, &gz);

        gx *= 180 / PI;
        gy *= 180 / PI;
        gz *= 180 / PI;

        offset[0] += gx*0.0175;
        offset[1] += gy*0.0175;
        offset[2] += gz*0.0175;

        usleep(10000);
    }
    offset[0]/=100.0;
    offset[1]/=100.0;
    offset[2]/=100.0;

    //printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);

    gyroOffset[0] = offset[0];
    gyroOffset[1] = offset[1];
    gyroOffset[2] = offset[2];
}

void AHRS::getEuler(float* roll, float* pitch, float* yaw)
{
   *roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * 180.0/M_PI;
   *pitch = asin(2*(q0*q2-q3*q1)) * 180.0/M_PI;
   *yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * 180.0/M_PI;
}

float AHRS::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float AHRS::getW()
{
    return  q0;
}

float AHRS::getX()
{
    return  q1;
}

float AHRS::getY()
{
    return  q2;
}

float AHRS::getZ()
{
    return  q3;
}

