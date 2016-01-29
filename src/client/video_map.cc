#include <algorithm>
#include <limits>
#include <chrono>

#include "video_map.hh"

using namespace std;
using namespace grpc;
using namespace chrono;

unsigned int find_max( vector<size_t> track_ids )
{
  if ( track_ids.empty() ) {
    throw runtime_error( "video has no tracks" );
  }
  
  sort( track_ids.begin(), track_ids.end() );

  for ( unsigned int i = 0; i < track_ids.size(); i++ ) {
    if ( i != track_ids[ i ] ) {
      throw runtime_error( "video does not have contiguous track_ids starting with 0" );
    }
  }

  return track_ids.back();
}

vector<size_t> get_track_lengths( const AlfalfaVideoClient & video )
{
  unsigned int max_track_id = find_max( video.get_track_ids() );
  vector<size_t> ret;
  
  for ( unsigned int i = 0; i <= max_track_id; i++ ) {
    ret.push_back( video.get_track_size( i ) );
  }

  return ret;
}

VideoMap::VideoMap( const string & server_address, const unsigned int raster_count )
  : video_( server_address ),
    track_lengths_ ( get_track_lengths( video_ ) ),
    tracks_(),
    shown_frame_counts_(),
    total_shown_frame_count_( raster_count )
{
  for ( unsigned int i = 0; i < track_lengths_.size(); i++ ) {
    tracks_.emplace_back();
    shown_frame_counts_.emplace_back();
    fetchers_.emplace_back( [&] ( unsigned int track_id ) { fetch_track( track_id ); }, i );
    fetchers_.back().detach();
  }
}

void VideoMap::fetch_track( unsigned int track_id )
{
  /* start fetching the track details */
  unique_lock<mutex> lock { mutex_ };
  grpc::ClientContext client_context;
  const unsigned int tracklen = track_lengths_.at( track_id );
  auto track_infos = video_.get_abridged_frames( client_context, track_id, 0, tracklen );

  lock.unlock();
  
  AlfalfaProtobufs::AbridgedFrameInfo frame;

  auto next_frame_read = steady_clock::now();

  while ( track_infos->Read( &frame ) ) {
    lock.lock();
    const uint64_t cumulative_length = frame.length() + (tracks_.at( track_id ).empty()
							 ? 0
							 : tracks_.at( track_id ).back().cumulative_length);
    tracks_.at( track_id ).emplace_back( frame,
					 shown_frame_counts_.at( track_id ),
					 track_id,
					 tracks_.at( track_id ).size(),
					 cumulative_length );
    if ( frame.key() ) {
      const auto & new_frame = tracks_.at( track_id ).back();
      keyframe_switches_.emplace( new_frame.timestamp,
				  make_pair( new_frame.track_id, new_frame.track_index ) );
    }

    if ( frame.shown() ) {
      shown_frame_counts_.at( track_id )++;
    }

    lock.unlock();

    /* throttle download of frames to 1000 fps per track to allow video to start playing */
    next_frame_read += milliseconds( 1 );
    this_thread::sleep_until( next_frame_read );
  }

  cerr << "all done with track " << track_id << endl;
  
  /* confirm all finished okay */
  lock.lock();
  RPC( "ClientReader::Finish", track_infos->Finish() );
  if ( tracks_.at( track_id ).size() != tracklen ) {
    throw runtime_error( "on track " + to_string( track_id ) + ", got "
			 + to_string( tracks_.at( track_id ).size() )
			 + " frames, expected " + to_string( tracklen ) );
  }
}

unsigned int VideoMap::track_length_full( const unsigned int track_id ) const
{
  return track_lengths_.at( track_id );
}

unsigned int VideoMap::track_length_now( const unsigned int track_id ) const
{
  unique_lock<mutex> lock { mutex_ };
  return tracks_.at( track_id ).size();
}

/* initialization state of the "dumb" decoder */
const AnnotatedFrameInfo & VideoMap::no_frame()
{
  static AnnotatedFrameInfo init_frame ( AlfalfaProtobufs::AbridgedFrameInfo {}, -1, -1, -1, -1 );
  return init_frame;
}

Optional<AnnotatedFrameInfo> VideoMap::successor( const AnnotatedFrameInfo & fi ) const
{
  if ( fi.track_id == no_frame().track_id ) {
    if ( tracks_.empty() or tracks_.at( 0 ).empty() ) {
      return {};
    } else {
      return { true, tracks_.at( 0 ).at( 0 ) };
    }
  }


  if ( fi.track_index + 1 >= tracks_.at( fi.track_id ).size() ) {
    return {};
  }

  return { true, tracks_.at( fi.track_id ).at( fi.track_index + 1 ) };
}

deque<AnnotatedFrameInfo> VideoMap::best_plan( const AnnotatedFrameInfo & last_frame,
					       const bool playing,
					       bool seeking ) const
{
  deque<AnnotatedFrameInfo> ret;
  unique_lock<mutex> lock { mutex_ };

  double time_margin_available = playing ? 0 : 3;

  //  cerr << "best_plan( " << track_id << ", " << track_index << " )\n";

  const auto previous_frame = [&] () {
    return ret.empty() ? last_frame : ret.back();
  };

  while ( true ) {
    vector<AnnotatedFrameInfo> eligible_next_frames;

    if ( seeking ) {
      const unsigned int intended_timestamp = last_frame.track_index;
      for ( int i = 0; eligible_next_frames.size() <= 4; i++ ) {
	auto range = keyframe_switches_.equal_range( intended_timestamp + i );
	for ( auto sw = range.first; sw != range.second; sw++ ) {
	  if ( sw->second.first > 5 ) {
	    continue;
	  }
	  eligible_next_frames.push_back( tracks_.at( sw->second.first ).at( sw->second.second ) );
	}

	range = keyframe_switches_.equal_range( intended_timestamp - i );
	for ( auto sw = range.first; sw != range.second; sw++ ) {
	  if ( sw->second.first > 5 ) {
	    continue;
	  }
	  eligible_next_frames.push_back( tracks_.at( sw->second.first ).at( sw->second.second ) );
	}
      }
      seeking = false;
    } else {
      int timestamp = 0;

      /* add the "normal" path option */
      auto normal_next_frame = successor( previous_frame() );
      if ( normal_next_frame.initialized() ) {
	/* the decoder is in a particular state already */
	timestamp = normal_next_frame.get().timestamp;
	eligible_next_frames.push_back( normal_next_frame.get() );
      }

      /* are there available keyframe options in other tracks? */
      auto range = keyframe_switches_.equal_range( timestamp );
      for ( auto sw = range.first; sw != range.second; sw++ ) {
	if ( sw->second.first > 5 ) {
	  continue;
	}
	eligible_next_frames.push_back( tracks_.at( sw->second.first ).at( sw->second.second ) );
      }
    }

    /* find best option */
    AnnotatedFrameInfo best_option = eligible_next_frames.front();
    for ( const auto & alternative : eligible_next_frames ) {
      /* penalize not-here frames with a constant offset, representing request-response latency */
      double best_margin_required = best_option.time_margin_required + (best_option.time_to_fetch ? 2.0 : 0);
      double alt_margin_required = alternative.time_margin_required + (alternative.time_to_fetch ? 2.0 : 0);

      const bool current_option_is_playable = best_margin_required <= time_margin_available;
      const bool alternative_is_playable = alt_margin_required <= time_margin_available;

      /* case 1: neither is playable. pick the one that has some reasonable compromise */
      if ( (not current_option_is_playable) and (not alternative_is_playable) ) {
	const double best_score = best_option.suffix_figure_of_merit() * 100 - best_margin_required;
	const double alt_score = alternative.suffix_figure_of_merit() * 100 - alt_margin_required;

	if ( alt_score > best_score ) {
	  best_option = alternative;
	}
	continue;
      }

      /* cases 2 and 3: one is playable and the other isn't */
      if ( (current_option_is_playable) and (not alternative_is_playable) ) {
	continue;
      }

      if ( (not current_option_is_playable) and (alternative_is_playable) ) {
	best_option = alternative;
	continue;
      }

      /* case 4: they're both playable */
      if ( alternative.suffix_figure_of_merit() > best_option.suffix_figure_of_merit() ) {
	best_option = alternative;
      }
    }

    ret.push_back( best_option );

    time_margin_available -= best_option.time_to_fetch;
    if ( best_option.shown ) {
      time_margin_available += 1.0 / 24.0;
    }

    /* are we done? */
    if ( not successor( best_option ).initialized() ) {
      break;
    }
  }

  /*
  cerr << "proposing a sequence of " << ret.size() << " frames\n";
  cerr << "first frame: " << ret.front().track_id << "\n";
  cerr << "time margin for first: " << ret.front().time_margin_required << "\n";
  */
  
  /*
  cerr << "PLAN:";
  for ( unsigned int i = 0; i < min( size_t( 128 ), ret.size() ); i++ ) {
    const auto & f = ret.at( i );
    cerr << " [ " << f.track_id << ", " << f.timestamp << ", " << f.time_margin_required << " ]";
  }
  cerr << "\n";
  */

  unsigned int already_buffered = 0;
  for ( const auto & x : ret ) {
    if ( x.time_to_fetch == 0 ) {
      already_buffered++;
    } else {
      break;
    }
  }

  double narrowest_margin = numeric_limits<double>::max();
  double narrowest_margin_time = numeric_limits<double>::max();
  double time_to_stall = numeric_limits<double>::max();
  double elapsed_time = 0;
  double frame_arrival_time = 0;

  for ( unsigned int i = 0; i < ret.size(); i++ ) {
    const auto & f = ret.at( i );

    if ( f.time_to_fetch ) {
      frame_arrival_time += f.time_to_fetch;

      double margin = elapsed_time - frame_arrival_time;
      if ( margin < narrowest_margin ) {
	narrowest_margin = margin;
	narrowest_margin_time = elapsed_time;
      }

      if ( (margin < 0) and (time_to_stall == numeric_limits<double>::max()) ) {
	time_to_stall = elapsed_time;
      }
    }

    if ( f.shown ) {
      elapsed_time += 1.0 / 24.0;
    }
  }

  cerr << "E[Mbit/s]: " << 8 * estimated_bytes_per_second_ / 1000000.0;
  cerr << ". Frames of plan in buffer: " << already_buffered << ".";
  if ( time_to_stall != numeric_limits<double>::max() ) {
    cerr << " PREDICTING STALL in " << time_to_stall << " seconds.";
  }

  cerr << " Narrowest margin will be " << narrowest_margin << " seconds at " << narrowest_margin_time << " seconds.";

  unsigned int cur_track = ret.empty() ? -1 : ret.front().track_id;
  unsigned int cur_count = 0;
  double cached_figure_of_merit = ret.empty() ? -1 : ret.front().suffix_figure_of_merit();

  cerr << " Plan: ";

  for ( const auto & x : ret ) {
    if ( x.track_id == cur_track ) {
      cur_count++;
    } else {
      cerr << "[ " << cur_count << " of " << cur_track << " @ " << cached_figure_of_merit << " ] ";
      cur_track = x.track_id;
      cur_count = 0;
      cached_figure_of_merit = x.suffix_figure_of_merit();
    }
  }

  cerr << "[ " << cur_count << " of " << cur_track << " @ " << cached_figure_of_merit << " ] ";

  cerr << "\n";

  return ret;
}

AnnotatedFrameInfo::AnnotatedFrameInfo( const AlfalfaProtobufs::AbridgedFrameInfo & fi,
					const unsigned int timestamp,
					const unsigned int track_id,
					const unsigned int track_index,
					const uint64_t cumulative_length )
  : offset( fi.offset() ),
    length( fi.length() ),
    frame_id( fi.frame_id() ),
    key( fi.key() ),
    shown( fi.shown() ),
    quality( fi.quality() ),
    timestamp( timestamp ),
    track_id( track_id ),
    track_index( track_index ),
    cumulative_length( cumulative_length )
{}

AnnotatedFrameInfo::AnnotatedFrameInfo( const FrameInfo & fi )
  : offset( fi.offset() ),
    length( fi.length() ),
    frame_id( fi.frame_id() ),
    key( fi.is_keyframe() ),
    shown( fi.shown() ),
    quality(),
    timestamp(),
    track_id(),
    track_index(),
    cumulative_length()
{}

void VideoMap::fetch_switch( const unsigned int track_id_,
			     const unsigned int frame_id_,
			     const unsigned int track_target_ )
{
  /* is this switch already pending? */
  if ( pending_switches_.find( make_tuple( track_id_, frame_id_, track_target_ ) ) != pending_switches_.end() ) {
    return;
  }

  pending_switches_.emplace( track_id_, frame_id_, track_target_ );

  thread newthread( [&] ( const unsigned int track_id, const unsigned int frame_id, const unsigned int track_target ) {
      cerr << "requesting switch from " << track_id << " " << frame_id << " to " << track_target << "\n";

      /* make RPC for switch */
      AlfalfaProtobufs::AbridgedSwitch sw = video_.get_outbound_switch( track_id,
									frame_id,
									track_target );
      unique_lock<mutex> lock { mutex_ };
      switches_.emplace( make_tuple( track_id, frame_id, track_target ),
			 sw );

      cerr << "got switch. total length in frames = " << sw.frame().size() << "\n";
    }, track_id_, frame_id_, track_target_ );

  newthread.detach();
}

void VideoMap::update_annotations( const double estimated_bytes_per_second,
				   const unordered_map<uint64_t, pair<uint64_t, size_t>> frame_store_ )
{
  estimated_bytes_per_second_ = estimated_bytes_per_second;

  thread newthread( [&] ( const unordered_map<uint64_t, pair<uint64_t, size_t>> frame_store ) { 
      std::unique_lock<mutex> locked { annotation_mutex_, try_to_lock };
      if ( not locked ) {
	cerr << "skipping redundant run of frame annotations\n";
	return;
      }

      unique_lock<mutex> lock { mutex_ };
    
      for ( auto & track : tracks_ ) {
	unsigned int shown_frame_count = 0;
	double running_mean = 0.0;
	double running_varsum = 0.0;
	double running_min = 1.0;

	double required_frame_arrival_time = 0;
	double elapsed_time = 0;

	/* algorithm from Knuth volume 2, per http://www.johndcook.com/blog/standard_deviation/ */
	for ( auto frame = track.rbegin(); frame != track.rend(); frame++ ) {
	  if ( frame->shown ) {
	    shown_frame_count++;

	    if ( shown_frame_count == 1 ) {
	      running_mean = running_min = frame->quality;
	      frame->average_quality_to_end = running_mean;
	      frame->stddev_quality_to_end = 0;
	      frame->min_quality_to_end = running_min;
	    } else {
	      const double new_mean = running_mean + ( frame->quality - running_mean ) / shown_frame_count;
	      const double new_varsum = running_varsum + ( frame->quality - running_mean ) * ( frame->quality - new_mean );
	      tie( running_mean, running_varsum ) = make_tuple( new_mean, new_varsum );

	      frame->average_quality_to_end = running_mean;
	      frame->stddev_quality_to_end = sqrt( running_varsum / ( shown_frame_count - 1.0 ) );

	      running_min = min( running_min, frame->quality );
	      frame->min_quality_to_end = running_min;
	    }
	  } else if ( shown_frame_count == 0 ) {
	    frame->average_quality_to_end = 1;
	    frame->stddev_quality_to_end = 0;
	    frame->min_quality_to_end = 1;
	  } else {
	    frame->average_quality_to_end = running_mean;
	    frame->stddev_quality_to_end = sqrt( running_varsum / ( shown_frame_count - 1.0 ) );
	    frame->min_quality_to_end = running_min;
	  }

	  if ( frame_store.find( frame->offset ) == frame_store.end() ) {
	    /* would need to fetch */
	    frame->time_to_fetch = frame->length / estimated_bytes_per_second_;
	  } else {
	    frame->time_to_fetch = 0;
	  }

	  required_frame_arrival_time -= frame->time_to_fetch;

	  if ( required_frame_arrival_time >= elapsed_time ) {
	    const double added_margin = required_frame_arrival_time - elapsed_time;

	    required_frame_arrival_time -= added_margin;
	  }

	  frame->time_margin_required = max( 0.0, elapsed_time - required_frame_arrival_time );

	  if ( frame->shown ) {
	    elapsed_time -= ( 1.0 / 24.0 );
	  }
	}
      }

      analysis_generation_++;
    }, move( frame_store_ ) );

  newthread.detach();
}

void VideoMap::report_feasibility() const
{
  unique_lock<mutex> lock { mutex_ };
  for ( unsigned int i = 0; i < tracks_.size(); i++ ) {
    if ( tracks_[ i ].empty() ) {
      continue;
    }

    const auto & frame = tracks_[ i ].front();
    
    cerr << "track " << i << ", average quality: " << frame.average_quality_to_end
	 << ", stddev: " << frame.stddev_quality_to_end
	 << ", min: " << frame.min_quality_to_end
	 << ", timestamp = " << frame.timestamp
	 << ", time margin required: " << frame.time_margin_required
	 << "\n";
  }
}

double AnnotatedFrameInfo::suffix_figure_of_merit() const
{
  return average_quality_to_end - stddev_quality_to_end;
}
