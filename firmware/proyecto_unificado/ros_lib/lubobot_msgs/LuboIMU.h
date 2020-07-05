#ifndef _ROS_lubobot_msgs_LuboIMU_h
#define _ROS_lubobot_msgs_LuboIMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "lubobot_msgs/LuboVector3.h"

namespace lubobot_msgs
{

  class LuboIMU : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef lubobot_msgs::LuboVector3 _accel_type;
      _accel_type accel;
      typedef lubobot_msgs::LuboVector3 _gyro_type;
      _gyro_type gyro;

    LuboIMU():
      header(),
      accel(),
      gyro()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      offset += this->gyro.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
      offset += this->gyro.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "lubobot_msgs/LuboIMU"; };
    const char * getMD5(){ return "53582bc8b7315f3bc7728d82df98bb24"; };

  };

}
#endif