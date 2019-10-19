#ifndef _ROS_tiny_msgs_tinyIMU_h
#define _ROS_tiny_msgs_tinyIMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "tiny_msgs/tinyVector.h"

namespace tiny_msgs
{

  class tinyIMU : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef tiny_msgs::tinyVector _accel_type;
      _accel_type accel;
      typedef tiny_msgs::tinyVector _gyro_type;
      _gyro_type gyro;

    tinyIMU():
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

    const char * getType(){ return "tiny_msgs/tinyIMU"; };
    const char * getMD5(){ return "53582bc8b7315f3bc7728d82df98bb24"; };

  };

}
#endif
