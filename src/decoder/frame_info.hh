#ifndef FRAME_INFO_HH
#define FRAME_INFO_HH

#include <set>
#include <string>
#include <fstream>

#include "decoder.hh"
#include "dependency_tracking.hh"

struct FrameName
{
  SourceHash source;
  TargetHash target;

  std::string str() const;

  FrameName( const std::string & name );
  FrameName( const SourceHash & source, const TargetHash & target );

  friend size_t hash_value( const FrameName & name );

  bool operator==( const FrameName & other ) const;
};

class SerializedFrame
{
private:
  FrameName name_;
  std::vector<uint8_t> serialized_frame_;

public:
  SerializedFrame( const FrameName & name, const std::vector<uint8_t> & data );

  FrameName name() const { return name_; }
  const Chunk chunk() const;
};

class FrameInfo
{
private:
  size_t offset_;
  size_t length_;
  FrameName name_;
  size_t frame_id_;

public:
  std::string frame_name() const { return name_.str(); }
  size_t offset() const { return offset_; }
  size_t length() const { return length_; }
  const size_t & frame_id() const { return frame_id_; }
  const FrameName name() const { return name_; }
  const SourceHash & source_hash() const { return name_.source; }
  const TargetHash & target_hash() const { return name_.target; }

  void set_source_hash( const SourceHash & source_hash ) { name_.source = source_hash; }
  void set_target_hash( const TargetHash & target_hash ) { name_.target = target_hash; }
  void set_offset( const size_t & offset ) { offset_ = offset; }
  void set_length( const size_t & length ) { length_ = length; }
  void set_frame_id( const size_t & frame_id ) { frame_id_ = frame_id; }

  FrameInfo( const FrameName & name, const size_t & offset, const size_t & length );

  bool validate_source( const DecoderHash & decoder_hash ) const;
  bool validate_target( const DecoderHash & decoder_hash ) const;
  bool shown() const;

  bool is_keyframe() const;
};

#endif /* FRAME_INFO_HH */
