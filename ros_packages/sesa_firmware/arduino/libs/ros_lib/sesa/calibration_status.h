#ifndef _ROS_sesa_calibration_status_h
#define _ROS_sesa_calibration_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sesa
{

  class calibration_status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _imu_1_type;
      _imu_1_type imu_1;
      typedef int8_t _imu_2_type;
      _imu_2_type imu_2;
      typedef int8_t _imu_3_type;
      _imu_3_type imu_3;
      typedef int8_t _imu_4_type;
      _imu_4_type imu_4;
      typedef int8_t _imu_5_type;
      _imu_5_type imu_5;
      typedef int8_t _imu_6_type;
      _imu_6_type imu_6;
      typedef int8_t _imu_7_type;
      _imu_7_type imu_7;
      typedef int8_t _imu_8_type;
      _imu_8_type imu_8;

    calibration_status():
      header(),
      imu_1(0),
      imu_2(0),
      imu_3(0),
      imu_4(0),
      imu_5(0),
      imu_6(0),
      imu_7(0),
      imu_8(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_1;
      u_imu_1.real = this->imu_1;
      *(outbuffer + offset + 0) = (u_imu_1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_1);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_2;
      u_imu_2.real = this->imu_2;
      *(outbuffer + offset + 0) = (u_imu_2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_2);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_3;
      u_imu_3.real = this->imu_3;
      *(outbuffer + offset + 0) = (u_imu_3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_3);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_4;
      u_imu_4.real = this->imu_4;
      *(outbuffer + offset + 0) = (u_imu_4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_4);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_5;
      u_imu_5.real = this->imu_5;
      *(outbuffer + offset + 0) = (u_imu_5.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_5);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_6;
      u_imu_6.real = this->imu_6;
      *(outbuffer + offset + 0) = (u_imu_6.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_6);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_7;
      u_imu_7.real = this->imu_7;
      *(outbuffer + offset + 0) = (u_imu_7.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_7);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_8;
      u_imu_8.real = this->imu_8;
      *(outbuffer + offset + 0) = (u_imu_8.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_8);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_1;
      u_imu_1.base = 0;
      u_imu_1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_1 = u_imu_1.real;
      offset += sizeof(this->imu_1);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_2;
      u_imu_2.base = 0;
      u_imu_2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_2 = u_imu_2.real;
      offset += sizeof(this->imu_2);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_3;
      u_imu_3.base = 0;
      u_imu_3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_3 = u_imu_3.real;
      offset += sizeof(this->imu_3);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_4;
      u_imu_4.base = 0;
      u_imu_4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_4 = u_imu_4.real;
      offset += sizeof(this->imu_4);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_5;
      u_imu_5.base = 0;
      u_imu_5.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_5 = u_imu_5.real;
      offset += sizeof(this->imu_5);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_6;
      u_imu_6.base = 0;
      u_imu_6.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_6 = u_imu_6.real;
      offset += sizeof(this->imu_6);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_7;
      u_imu_7.base = 0;
      u_imu_7.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_7 = u_imu_7.real;
      offset += sizeof(this->imu_7);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_8;
      u_imu_8.base = 0;
      u_imu_8.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_8 = u_imu_8.real;
      offset += sizeof(this->imu_8);
     return offset;
    }

    virtual const char * getType() override { return "sesa/calibration_status"; };
    virtual const char * getMD5() override { return "b67307df0657976be0c06dd203742685"; };

  };

}
#endif
