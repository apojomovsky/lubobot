#ifndef _ROS_ca_msgs_PlaySong_h
#define _ROS_ca_msgs_PlaySong_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ca_msgs
{

  class PlaySong : public ros::Msg
  {
    public:
      typedef uint8_t _song_type;
      _song_type song;

    PlaySong():
      song(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->song >> (8 * 0)) & 0xFF;
      offset += sizeof(this->song);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->song =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->song);
     return offset;
    }

    const char * getType(){ return "ca_msgs/PlaySong"; };
    const char * getMD5(){ return "eb55a5c354f06b6572d1ec46a28b6e6a"; };

  };

}
#endif
