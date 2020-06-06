#ifndef _ROS_ca_msgs_DefineSong_h
#define _ROS_ca_msgs_DefineSong_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ca_msgs
{

  class DefineSong : public ros::Msg
  {
    public:
      typedef uint8_t _song_type;
      _song_type song;
      typedef uint8_t _length_type;
      _length_type length;
      uint32_t notes_length;
      typedef uint8_t _notes_type;
      _notes_type st_notes;
      _notes_type * notes;
      uint32_t durations_length;
      typedef float _durations_type;
      _durations_type st_durations;
      _durations_type * durations;

    DefineSong():
      song(0),
      length(0),
      notes_length(0), notes(NULL),
      durations_length(0), durations(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->song >> (8 * 0)) & 0xFF;
      offset += sizeof(this->song);
      *(outbuffer + offset + 0) = (this->length >> (8 * 0)) & 0xFF;
      offset += sizeof(this->length);
      *(outbuffer + offset + 0) = (this->notes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->notes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->notes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->notes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->notes_length);
      for( uint32_t i = 0; i < notes_length; i++){
      *(outbuffer + offset + 0) = (this->notes[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->notes[i]);
      }
      *(outbuffer + offset + 0) = (this->durations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->durations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->durations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->durations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->durations_length);
      for( uint32_t i = 0; i < durations_length; i++){
      union {
        float real;
        uint32_t base;
      } u_durationsi;
      u_durationsi.real = this->durations[i];
      *(outbuffer + offset + 0) = (u_durationsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_durationsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_durationsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_durationsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->durations[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->song =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->song);
      this->length =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->length);
      uint32_t notes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      notes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      notes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      notes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->notes_length);
      if(notes_lengthT > notes_length)
        this->notes = (uint8_t*)realloc(this->notes, notes_lengthT * sizeof(uint8_t));
      notes_length = notes_lengthT;
      for( uint32_t i = 0; i < notes_length; i++){
      this->st_notes =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_notes);
        memcpy( &(this->notes[i]), &(this->st_notes), sizeof(uint8_t));
      }
      uint32_t durations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      durations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      durations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      durations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->durations_length);
      if(durations_lengthT > durations_length)
        this->durations = (float*)realloc(this->durations, durations_lengthT * sizeof(float));
      durations_length = durations_lengthT;
      for( uint32_t i = 0; i < durations_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_durations;
      u_st_durations.base = 0;
      u_st_durations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_durations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_durations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_durations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_durations = u_st_durations.real;
      offset += sizeof(this->st_durations);
        memcpy( &(this->durations[i]), &(this->st_durations), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "ca_msgs/DefineSong"; };
    const char * getMD5(){ return "ae0ef1f2fad74bf546a9e5f037c27a5d"; };

  };

}
#endif
