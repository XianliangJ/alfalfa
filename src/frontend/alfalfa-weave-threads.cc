#include <iostream>
#include <fstream>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include "continuation_player.hh"
#include "alfalfa_video.hh"

using namespace std;

class SwitchPlayer : public SourcePlayer
{
private:
  struct Switch {
    TrackDBIterator switch_start;
    vector<FrameInfo> frames;
  };

  vector<Switch> switches_;

public:
  SwitchPlayer( const TrackDBIterator & start, const ContinuationPlayer & original )
    : SourcePlayer( original ),
      switches_ { Switch { start, {} } }
  {}

  void add_switch_frame( const FrameInfo & frame )
  {
    for ( Switch & switch_ : switches_ ) {
      switch_.frames.push_back( frame );
    }
  }

  void merge( const SwitchPlayer & other )
  {
    for ( const Switch & sw : other.switches_ ) {
      switches_.emplace_back( move( sw ) );
    }
  }

  void write_switches( WritableAlfalfaVideo & alf, const TrackDBIterator & switch_end ) const
  {
    for ( const Switch & switch_ : switches_ ) {
      alf.insert_switch_frames( switch_.switch_start, switch_.frames, switch_end );
    }
  }
};

// Streams keep track of all the players converging onto them,
// because they need to be able to update the states of those
// frames with their continuation frames
class StreamState
{
private:
  ContinuationPlayer stream_player_;

  vector<FrameInfo> cur_frames_ {};
  // List of players we are generating continuations from
  vector<SwitchPlayer> switch_players_ {};

  TrackDBIterator track_frame_, track_end_;
  
  const PlayableAlfalfaVideo & source_alf_;

public:
  StreamState( const pair<TrackDBIterator, TrackDBIterator> & track,
               const PlayableAlfalfaVideo & source_alf )
    : stream_player_( source_alf.get_info().width, source_alf.get_info().height ),
      track_frame_( track.first ),
      track_end_( track.second ),
      source_alf_( source_alf )
  {}

  void update_switch_players( WritableAlfalfaVideo & alf )
  {
    vector<SwitchPlayer> new_players;
    new_players.reserve( switch_players_.size() );

    for ( SwitchPlayer & player : switch_players_ ) {
      for ( const FrameInfo & frame : cur_frames_ ) {
        if ( player.can_decode( frame ) ) {
          player.safe_decode( frame, source_alf_.get_chunk( frame ) );
          player.add_switch_frame( frame );
        } else {
          player.set_need_continuation( true );
          break;
        }
      }

      if ( player == stream_player_ ) {
        assert( not player.need_continuation() );
        /* Check if we're converged on the track after decoding the current frames */
        player.write_switches( alf, ++TrackDBIterator( track_frame_ ) );
      } else {
        /* Remove duplicate converging players */
        auto iter = find( new_players.begin(), new_players.end(), player ); 
        if ( iter != new_players.end() ) {
          /* If two players are equal but have yet to converge
           * we only need to track one copy, but merge together their switch
           * information */
          iter->merge( player );
        } else {
          new_players.emplace_back( move( player ) );
        }
      }
    }

    switch_players_ = move( new_players );
  }


  void advance_until_shown()
  {
    cur_frames_.clear();
    // Serialize and write until next displayed frame
    while ( not eos() ) {
      const FrameInfo & frame = *track_frame_;

      stream_player_.decode( source_alf_.get_chunk( frame ) );
      cur_frames_.push_back( frame );

      if ( frame.shown() ) {
        return;
      }

      track_frame_++;
    }
    throw Unsupported( "Undisplayed frames at end of video unsupported" );
  }

  void advance_past_shown()
  {
    track_frame_++;
  }

  void new_source( StreamState & other )
  {
    switch_players_.emplace_back( other.track_frame_, other.stream_player_ );
  }

  // Create all the necessary continuation frames for this point in the track,
  // and save them into new_alf
  void make_continuations( WritableAlfalfaVideo & new_alf )
  {
    for ( SwitchPlayer & switch_player : switch_players_ ) {
      // Check if the source needs a continuation at this point
      if ( switch_player.need_continuation() ) {
        for ( const SerializedFrame & frame : stream_player_.make_reference_updates( switch_player ) ) {
          FrameInfo info = new_alf.import_serialized_frame( frame );
          switch_player.add_switch_frame( info );
          switch_player.safe_decode( info, frame.chunk() );
        }

        if ( stream_player_.need_state_update( switch_player ) ) {
          SerializedFrame state_update = stream_player_.make_state_update( switch_player );
          FrameInfo info = new_alf.import_serialized_frame( state_update );
          switch_player.add_switch_frame( info );
          switch_player.safe_decode( info, state_update.chunk() );
        }

        SerializedFrame inter = stream_player_.rewrite_inter_frame( switch_player ); 
        FrameInfo info = new_alf.import_serialized_frame( inter );
        switch_player.add_switch_frame( info );
        
        // Even if we didn't need to make a new continuation frame, we still need to sync the most recent
        // changes from stream player (the same effect as decoding stream_player's last frame)
        //stream_player_.apply_changes( switch_player );
        switch_player.safe_decode( info, inter.chunk() );

        // Set player as not needing continuation, will be reevaluated next frame
        switch_player.set_need_continuation( false );
      }
    }
  }

  void write_final_switches( WritableAlfalfaVideo & new_alf )
  {
    for ( const SwitchPlayer & player : switch_players_ ) {
      player.write_switches( new_alf, track_frame_ );
    }
  }

  bool on_keyframe( void ) const
  {
    return track_frame_->is_keyframe();
  }

  bool eos( void ) const
  {
    return track_frame_ == track_end_; 
  }
  
  unsigned num_converging() const
  {
    return switch_players_.size();
  }
};

class ContinuationGenerator
{
private: 
  PlayableAlfalfaVideo orig_alf_;
  WritableAlfalfaVideo new_alf_;
  vector<StreamState> streams_ {};

public:
  ContinuationGenerator( const string & source_alf_path, const string & new_alf_path,
                         unsigned long dri,
                         vector<size_t> track_ids )
    : orig_alf_( source_alf_path ), new_alf_( new_alf_path, orig_alf_.get_info().width, orig_alf_.get_info().height )
  {
    /* Copy in the source video */
    combine( new_alf_, orig_alf_ );

    for ( size_t track_id : track_ids ) {
      size_t begin_frame = orig_alf_.get_dri_to_frame_index_mapping( track_id, dri )[ 0 ];

      auto track = orig_alf_.get_track_range( track_id, begin_frame, orig_alf_.get_track_size( track_id ) );

      while ( track.first != track.second and not track.first->is_keyframe() ) {
        track.first++;
      }

      streams_.emplace_back( track, orig_alf_ );
    }
  }

  void write_continuations( unsigned long weave_length )
  {
    unsigned weaved_frames = 0;
    bool one_keyframe_reached = false;
    bool two_keyframe_reached = false;

    while ( ( weaved_frames < weave_length or not one_keyframe_reached or not two_keyframe_reached or streams_[ 0 ].num_converging() > 0 or streams_[ 1 ].num_converging() > 0 ) and not streams_[ 0 ].eos() ) {
      for ( StreamState & stream : streams_ ) {
        stream.advance_until_shown();
      }

      // Add new starting points for continuations
      StreamState & source_stream = streams_[ 0 ];
      StreamState & target_stream = streams_[ 1 ];

      // Insert new copies of switch_player and target_players for continuations starting
      // at this frame

      if ( not ( weaved_frames >= weave_length and one_keyframe_reached and two_keyframe_reached ) )
      {
        target_stream.new_source( source_stream );
        source_stream.new_source( target_stream );
      }
      

      for ( unsigned i = 0; i < streams_.size(); i++ ) {
        StreamState & stream = streams_[ i ];
        stream.update_switch_players( new_alf_ );
        stream.make_continuations( new_alf_ );
        stream.advance_past_shown();
      }

      weaved_frames++;

      if ( weaved_frames >= weave_length and source_stream.on_keyframe() ) {
        one_keyframe_reached = true;
      }

      if ( weaved_frames >= weave_length and target_stream.on_keyframe() ) {
        two_keyframe_reached = true;
      }
    }

    for ( StreamState & stream : streams_ ) {
      stream.write_final_switches( new_alf_ );
    }

    new_alf_.save();
  }
};

int main( int argc, const char * const argv[] )
{
  if ( argc < 7 ) {
    cerr << "Usage: " << argv[ 0 ] << " <input-alf> <output-alf> <dri> <weave-length> <trajectoryA> <trajectoryB>" << endl;
    return EXIT_FAILURE;
  }

  unsigned long dri = stoul( argv[ 3 ] );
  unsigned long weave_length = stoul( argv[ 4 ] );

  vector<size_t> track_ids;
  for ( int i = 5; i < 7; i++ ) {
    track_ids.push_back( stoul( argv[ i ] ) );
  }

  ContinuationGenerator generator( argv[ 1 ], argv[ 2 ], dri, track_ids );

  generator.write_continuations( weave_length );
}
