#ifndef _ROS_lubobot_msgs_LuboEncoders_h
#define _ROS_lubobot_msgs_LuboEncoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lubobot_msgs
{

  class LuboEncoders : public ros::Msg
  {
    public:
      typedef uint16_t _left_type;
      _left_type left;
      typedef uint16_t _right_type;
      _right_type right;

    LuboEncoders():
      left(0),
      right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left);
      *(outbuffer + offset + 0) = (this->right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->left =  ((uint16_t) (*(inbuffer + offset)));
      this->left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->left);
      this->right =  ((uint16_t) (*(inbuffer + offset)));
      this->right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->right);
     return offset;
    }

    const char * getType(){ return "lubobot_msgs/LuboEncoders"; };
    const char * getMD5(){ return "52aa20a03a2be2568dcbb88f1a7e85e5"; };

  };

}
#endif