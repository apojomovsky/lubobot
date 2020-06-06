#ifndef _ROS_ca_msgs_Bumper_h
#define _ROS_ca_msgs_Bumper_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ca_msgs
{

  class Bumper : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _is_left_pressed_type;
      _is_left_pressed_type is_left_pressed;
      typedef bool _is_right_pressed_type;
      _is_right_pressed_type is_right_pressed;
      typedef bool _is_light_left_type;
      _is_light_left_type is_light_left;
      typedef bool _is_light_front_left_type;
      _is_light_front_left_type is_light_front_left;
      typedef bool _is_light_center_left_type;
      _is_light_center_left_type is_light_center_left;
      typedef bool _is_light_center_right_type;
      _is_light_center_right_type is_light_center_right;
      typedef bool _is_light_front_right_type;
      _is_light_front_right_type is_light_front_right;
      typedef bool _is_light_right_type;
      _is_light_right_type is_light_right;
      typedef uint16_t _light_signal_left_type;
      _light_signal_left_type light_signal_left;
      typedef uint16_t _light_signal_front_left_type;
      _light_signal_front_left_type light_signal_front_left;
      typedef uint16_t _light_signal_center_left_type;
      _light_signal_center_left_type light_signal_center_left;
      typedef uint16_t _light_signal_center_right_type;
      _light_signal_center_right_type light_signal_center_right;
      typedef uint16_t _light_signal_front_right_type;
      _light_signal_front_right_type light_signal_front_right;
      typedef uint16_t _light_signal_right_type;
      _light_signal_right_type light_signal_right;

    Bumper():
      header(),
      is_left_pressed(0),
      is_right_pressed(0),
      is_light_left(0),
      is_light_front_left(0),
      is_light_center_left(0),
      is_light_center_right(0),
      is_light_front_right(0),
      is_light_right(0),
      light_signal_left(0),
      light_signal_front_left(0),
      light_signal_center_left(0),
      light_signal_center_right(0),
      light_signal_front_right(0),
      light_signal_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_left_pressed;
      u_is_left_pressed.real = this->is_left_pressed;
      *(outbuffer + offset + 0) = (u_is_left_pressed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_left_pressed);
      union {
        bool real;
        uint8_t base;
      } u_is_right_pressed;
      u_is_right_pressed.real = this->is_right_pressed;
      *(outbuffer + offset + 0) = (u_is_right_pressed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_right_pressed);
      union {
        bool real;
        uint8_t base;
      } u_is_light_left;
      u_is_light_left.real = this->is_light_left;
      *(outbuffer + offset + 0) = (u_is_light_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_light_left);
      union {
        bool real;
        uint8_t base;
      } u_is_light_front_left;
      u_is_light_front_left.real = this->is_light_front_left;
      *(outbuffer + offset + 0) = (u_is_light_front_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_light_front_left);
      union {
        bool real;
        uint8_t base;
      } u_is_light_center_left;
      u_is_light_center_left.real = this->is_light_center_left;
      *(outbuffer + offset + 0) = (u_is_light_center_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_light_center_left);
      union {
        bool real;
        uint8_t base;
      } u_is_light_center_right;
      u_is_light_center_right.real = this->is_light_center_right;
      *(outbuffer + offset + 0) = (u_is_light_center_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_light_center_right);
      union {
        bool real;
        uint8_t base;
      } u_is_light_front_right;
      u_is_light_front_right.real = this->is_light_front_right;
      *(outbuffer + offset + 0) = (u_is_light_front_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_light_front_right);
      union {
        bool real;
        uint8_t base;
      } u_is_light_right;
      u_is_light_right.real = this->is_light_right;
      *(outbuffer + offset + 0) = (u_is_light_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_light_right);
      *(outbuffer + offset + 0) = (this->light_signal_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_signal_left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light_signal_left);
      *(outbuffer + offset + 0) = (this->light_signal_front_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_signal_front_left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light_signal_front_left);
      *(outbuffer + offset + 0) = (this->light_signal_center_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_signal_center_left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light_signal_center_left);
      *(outbuffer + offset + 0) = (this->light_signal_center_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_signal_center_right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light_signal_center_right);
      *(outbuffer + offset + 0) = (this->light_signal_front_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_signal_front_right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light_signal_front_right);
      *(outbuffer + offset + 0) = (this->light_signal_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_signal_right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light_signal_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_left_pressed;
      u_is_left_pressed.base = 0;
      u_is_left_pressed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_left_pressed = u_is_left_pressed.real;
      offset += sizeof(this->is_left_pressed);
      union {
        bool real;
        uint8_t base;
      } u_is_right_pressed;
      u_is_right_pressed.base = 0;
      u_is_right_pressed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_right_pressed = u_is_right_pressed.real;
      offset += sizeof(this->is_right_pressed);
      union {
        bool real;
        uint8_t base;
      } u_is_light_left;
      u_is_light_left.base = 0;
      u_is_light_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_light_left = u_is_light_left.real;
      offset += sizeof(this->is_light_left);
      union {
        bool real;
        uint8_t base;
      } u_is_light_front_left;
      u_is_light_front_left.base = 0;
      u_is_light_front_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_light_front_left = u_is_light_front_left.real;
      offset += sizeof(this->is_light_front_left);
      union {
        bool real;
        uint8_t base;
      } u_is_light_center_left;
      u_is_light_center_left.base = 0;
      u_is_light_center_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_light_center_left = u_is_light_center_left.real;
      offset += sizeof(this->is_light_center_left);
      union {
        bool real;
        uint8_t base;
      } u_is_light_center_right;
      u_is_light_center_right.base = 0;
      u_is_light_center_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_light_center_right = u_is_light_center_right.real;
      offset += sizeof(this->is_light_center_right);
      union {
        bool real;
        uint8_t base;
      } u_is_light_front_right;
      u_is_light_front_right.base = 0;
      u_is_light_front_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_light_front_right = u_is_light_front_right.real;
      offset += sizeof(this->is_light_front_right);
      union {
        bool real;
        uint8_t base;
      } u_is_light_right;
      u_is_light_right.base = 0;
      u_is_light_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_light_right = u_is_light_right.real;
      offset += sizeof(this->is_light_right);
      this->light_signal_left =  ((uint16_t) (*(inbuffer + offset)));
      this->light_signal_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light_signal_left);
      this->light_signal_front_left =  ((uint16_t) (*(inbuffer + offset)));
      this->light_signal_front_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light_signal_front_left);
      this->light_signal_center_left =  ((uint16_t) (*(inbuffer + offset)));
      this->light_signal_center_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light_signal_center_left);
      this->light_signal_center_right =  ((uint16_t) (*(inbuffer + offset)));
      this->light_signal_center_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light_signal_center_right);
      this->light_signal_front_right =  ((uint16_t) (*(inbuffer + offset)));
      this->light_signal_front_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light_signal_front_right);
      this->light_signal_right =  ((uint16_t) (*(inbuffer + offset)));
      this->light_signal_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light_signal_right);
     return offset;
    }

    const char * getType(){ return "ca_msgs/Bumper"; };
    const char * getMD5(){ return "18fe5b1d31e6a8db180b924157ac665e"; };

  };

}
#endif
