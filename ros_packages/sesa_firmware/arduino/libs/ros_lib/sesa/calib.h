#ifndef _ROS_sesa_calib_h
#define _ROS_sesa_calib_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sesa
{

  class calib : public ros::Msg
  {
    public:
      typedef uint8_t _ID_type;
      _ID_type ID;
      typedef uint8_t _sys_type;
      _sys_type sys;
      typedef uint8_t _acc_type;
      _acc_type acc;
      typedef uint8_t _gyro_type;
      _gyro_type gyro;
      typedef uint8_t _mag_type;
      _mag_type mag;
      typedef int16_t _off_accX_type;
      _off_accX_type off_accX;
      typedef int16_t _off_accY_type;
      _off_accY_type off_accY;
      typedef int16_t _off_accZ_type;
      _off_accZ_type off_accZ;
      typedef int16_t _off_magX_type;
      _off_magX_type off_magX;
      typedef int16_t _off_magY_type;
      _off_magY_type off_magY;
      typedef int16_t _off_magZ_type;
      _off_magZ_type off_magZ;
      typedef int16_t _off_gyroX_type;
      _off_gyroX_type off_gyroX;
      typedef int16_t _off_gyroY_type;
      _off_gyroY_type off_gyroY;
      typedef int16_t _off_gyroZ_type;
      _off_gyroZ_type off_gyroZ;
      typedef int16_t _rad_acc_type;
      _rad_acc_type rad_acc;
      typedef int16_t _rad_mag_type;
      _rad_mag_type rad_mag;

    calib():
      ID(0),
      sys(0),
      acc(0),
      gyro(0),
      mag(0),
      off_accX(0),
      off_accY(0),
      off_accZ(0),
      off_magX(0),
      off_magY(0),
      off_magZ(0),
      off_gyroX(0),
      off_gyroY(0),
      off_gyroZ(0),
      rad_acc(0),
      rad_mag(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ID);
      *(outbuffer + offset + 0) = (this->sys >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sys);
      *(outbuffer + offset + 0) = (this->acc >> (8 * 0)) & 0xFF;
      offset += sizeof(this->acc);
      *(outbuffer + offset + 0) = (this->gyro >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gyro);
      *(outbuffer + offset + 0) = (this->mag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag);
      union {
        int16_t real;
        uint16_t base;
      } u_off_accX;
      u_off_accX.real = this->off_accX;
      *(outbuffer + offset + 0) = (u_off_accX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_accX.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_accX);
      union {
        int16_t real;
        uint16_t base;
      } u_off_accY;
      u_off_accY.real = this->off_accY;
      *(outbuffer + offset + 0) = (u_off_accY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_accY.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_accY);
      union {
        int16_t real;
        uint16_t base;
      } u_off_accZ;
      u_off_accZ.real = this->off_accZ;
      *(outbuffer + offset + 0) = (u_off_accZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_accZ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_accZ);
      union {
        int16_t real;
        uint16_t base;
      } u_off_magX;
      u_off_magX.real = this->off_magX;
      *(outbuffer + offset + 0) = (u_off_magX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_magX.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_magX);
      union {
        int16_t real;
        uint16_t base;
      } u_off_magY;
      u_off_magY.real = this->off_magY;
      *(outbuffer + offset + 0) = (u_off_magY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_magY.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_magY);
      union {
        int16_t real;
        uint16_t base;
      } u_off_magZ;
      u_off_magZ.real = this->off_magZ;
      *(outbuffer + offset + 0) = (u_off_magZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_magZ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_magZ);
      union {
        int16_t real;
        uint16_t base;
      } u_off_gyroX;
      u_off_gyroX.real = this->off_gyroX;
      *(outbuffer + offset + 0) = (u_off_gyroX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_gyroX.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_gyroX);
      union {
        int16_t real;
        uint16_t base;
      } u_off_gyroY;
      u_off_gyroY.real = this->off_gyroY;
      *(outbuffer + offset + 0) = (u_off_gyroY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_gyroY.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_gyroY);
      union {
        int16_t real;
        uint16_t base;
      } u_off_gyroZ;
      u_off_gyroZ.real = this->off_gyroZ;
      *(outbuffer + offset + 0) = (u_off_gyroZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_off_gyroZ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->off_gyroZ);
      union {
        int16_t real;
        uint16_t base;
      } u_rad_acc;
      u_rad_acc.real = this->rad_acc;
      *(outbuffer + offset + 0) = (u_rad_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rad_acc.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rad_acc);
      union {
        int16_t real;
        uint16_t base;
      } u_rad_mag;
      u_rad_mag.real = this->rad_mag;
      *(outbuffer + offset + 0) = (u_rad_mag.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rad_mag.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rad_mag);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->ID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ID);
      this->sys =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sys);
      this->acc =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->acc);
      this->gyro =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gyro);
      this->mag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mag);
      union {
        int16_t real;
        uint16_t base;
      } u_off_accX;
      u_off_accX.base = 0;
      u_off_accX.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_accX.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_accX = u_off_accX.real;
      offset += sizeof(this->off_accX);
      union {
        int16_t real;
        uint16_t base;
      } u_off_accY;
      u_off_accY.base = 0;
      u_off_accY.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_accY.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_accY = u_off_accY.real;
      offset += sizeof(this->off_accY);
      union {
        int16_t real;
        uint16_t base;
      } u_off_accZ;
      u_off_accZ.base = 0;
      u_off_accZ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_accZ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_accZ = u_off_accZ.real;
      offset += sizeof(this->off_accZ);
      union {
        int16_t real;
        uint16_t base;
      } u_off_magX;
      u_off_magX.base = 0;
      u_off_magX.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_magX.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_magX = u_off_magX.real;
      offset += sizeof(this->off_magX);
      union {
        int16_t real;
        uint16_t base;
      } u_off_magY;
      u_off_magY.base = 0;
      u_off_magY.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_magY.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_magY = u_off_magY.real;
      offset += sizeof(this->off_magY);
      union {
        int16_t real;
        uint16_t base;
      } u_off_magZ;
      u_off_magZ.base = 0;
      u_off_magZ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_magZ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_magZ = u_off_magZ.real;
      offset += sizeof(this->off_magZ);
      union {
        int16_t real;
        uint16_t base;
      } u_off_gyroX;
      u_off_gyroX.base = 0;
      u_off_gyroX.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_gyroX.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_gyroX = u_off_gyroX.real;
      offset += sizeof(this->off_gyroX);
      union {
        int16_t real;
        uint16_t base;
      } u_off_gyroY;
      u_off_gyroY.base = 0;
      u_off_gyroY.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_gyroY.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_gyroY = u_off_gyroY.real;
      offset += sizeof(this->off_gyroY);
      union {
        int16_t real;
        uint16_t base;
      } u_off_gyroZ;
      u_off_gyroZ.base = 0;
      u_off_gyroZ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_off_gyroZ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->off_gyroZ = u_off_gyroZ.real;
      offset += sizeof(this->off_gyroZ);
      union {
        int16_t real;
        uint16_t base;
      } u_rad_acc;
      u_rad_acc.base = 0;
      u_rad_acc.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rad_acc.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rad_acc = u_rad_acc.real;
      offset += sizeof(this->rad_acc);
      union {
        int16_t real;
        uint16_t base;
      } u_rad_mag;
      u_rad_mag.base = 0;
      u_rad_mag.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rad_mag.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rad_mag = u_rad_mag.real;
      offset += sizeof(this->rad_mag);
     return offset;
    }

    virtual const char * getType() override { return "sesa/calib"; };
    virtual const char * getMD5() override { return "59992e462b9023081725eaac64aa3d5c"; };

  };

}
#endif
