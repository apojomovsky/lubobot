#ifndef _ROS_ca_msgs_ChargingState_h
#define _ROS_ca_msgs_ChargingState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ca_msgs
{

  class ChargingState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _state_type;
      _state_type state;
      enum { CHARGE_NONE = 0 };
      enum { CHARGE_RECONDITION = 1 };
      enum { CHARGE_FULL = 2 };
      enum { CHARGE_TRICKLE = 3 };
      enum { CHARGE_WAITING = 4 };
      enum { CHARGE_FAULT = 5 };

    ChargingState():
      header(),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "ca_msgs/ChargingState"; };
    const char * getMD5(){ return "e4da1ef814a2def80691224a5e5883ea"; };

  };

}
#endif
