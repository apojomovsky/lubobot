#ifndef _ROS_ca_msgs_Mode_h
#define _ROS_ca_msgs_Mode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ca_msgs
{

  class Mode : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _mode_type;
      _mode_type mode;
      enum { MODE_OFF = 0 };
      enum { MODE_PASSIVE = 1 };
      enum { MODE_SAFE = 2 };
      enum { MODE_FULL = 3 };

    Mode():
      header(),
      mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
     return offset;
    }

    const char * getType(){ return "ca_msgs/Mode"; };
    const char * getMD5(){ return "b4faf4b68b6555d4656417971bee31a0"; };

  };

}
#endif
