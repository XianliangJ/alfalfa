#ifndef CONTINUATION_PLAYER_HH
#define CONTINUATION_PLAYER_HH

#include "player.hh"
#include "frame.hh"

class SourcePlayer;

struct PreContinuation
{
  bool shown;
  MissingTracker missing;
  SourceHash source_hash;
  TargetHash target_hash;

  std::string continuation_name( void ) const;
};

class ContinuationPlayer : public FramePlayer
{
private:
  References prev_references_;
  bool shown_;
  RasterHandle cur_output_;
  UpdateTracker cur_updates_;
  bool on_key_frame_;
  Optional<KeyFrame> key_frame_;
  Optional<InterFrame> inter_frame_;

  template<class FrameType>
  Optional<RasterHandle> track_continuation_info( const FrameType & frame );

public:
  ContinuationPlayer( const uint16_t width, const uint16_t height );

  ContinuationPlayer( ContinuationPlayer && ) = default;
  ContinuationPlayer & operator=( ContinuationPlayer && ) = default;

  Optional<RasterHandle> decode( const SerializedFrame & frame );

  void apply_changes( Decoder & other ) const;

  bool need_gen_continuation() const;

  PreContinuation make_pre_continuation( const SourcePlayer & source ) const;

  SerializedFrame make_continuation( const PreContinuation & pre,
                                     const SourcePlayer & source ) const;
  SerializedFrame operator-( const SourcePlayer & source ) const;
};

class SourcePlayer : public FramePlayer
{
friend class ContinuationPlayer;
private:
  bool need_continuation_, first_continuation_;

public:
  SourcePlayer( const ContinuationPlayer & original )
    : FramePlayer( original ),
      need_continuation_( true ), // Assume we're making a continuation frame unless we've shown otherwise
      first_continuation_( true )
  {}

  void set_need_continuation( bool b )
  {
    need_continuation_ = b;
  }

  bool need_continuation() const
  {
    return need_continuation_;
  }

  bool first_continuation() const
  {
    return first_continuation_;
  }

  void unset_first_continuation()
  {
    first_continuation_ = false;
  }

  Optional<RasterHandle> decode( const SerializedFrame & frame )
  {
    return FramePlayer::decode( frame );
  }

  void sync_changes( const ContinuationPlayer & target_player )
  {
    target_player.apply_changes( decoder_ );
  }

  bool operator==( const SourcePlayer & other) const
  {
    return FramePlayer::operator==( other ) and need_continuation_ == other.need_continuation_ and
      first_continuation_ == other.first_continuation_;
  }
};

#endif
