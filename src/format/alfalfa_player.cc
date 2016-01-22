#include "alfalfa_player.hh"

using namespace std;

// Used to control the maximum number of FrameInfos batched into a single
// AlfalfaProtobufs::FrameIterator object -- this is to ensure protobufs
// sent over the network aren't too large.
const size_t MAX_NUM_FRAMES = 1000;

template<class ObjectType>
void LRUCache<ObjectType>::put( const size_t key, const ObjectType & obj )
{
  if ( cache_.count( key ) > 0 ) {
    auto & item = cache_.at( key );
    cached_items_.erase( item.second );
    cached_items_.push_front( key );
    item.second = cached_items_.cbegin();
  }
  else {
    cached_items_.push_front( key );

    if ( cached_items_.size() > cache_capacity ) {
      cache_.erase( cached_items_.back() );
      cached_items_.pop_back();
    }
  }

  cache_.insert( make_pair( key,
                            make_pair( obj, cached_items_.cbegin() ) ) );
}

void RasterAndStateCache::put( const Decoder & decoder )
{
  raster_cache_.put( decoder.get_references().last.hash(), decoder.get_references().last );
  raster_cache_.put( decoder.get_references().golden.hash(), decoder.get_references().golden );
  raster_cache_.put( decoder.get_references().alternative_reference.hash(),
                     decoder.get_references().alternative_reference );
  state_cache_.put( decoder.get_state().hash(), decoder.get_state() );
}

template<class ObjectType>
bool LRUCache<ObjectType>::has( const size_t key ) const
{
  return cache_.count( key ) > 0;
}

template<class ObjectType>
ObjectType LRUCache<ObjectType>::get( const size_t key )
{
  /* bump entry to the front of the LRU list */
  auto & item = cache_.at( key );
  cached_items_.erase( item.second );
  cached_items_.push_front( key );
  item.second = cached_items_.cbegin();

  /* return the entry */
  return item.first;
}

template <class ObjectType>
void LRUCache<ObjectType>::clear()
{
  cache_.clear();
  cached_items_.clear();
}

template <class ObjectType>
size_t LRUCache<ObjectType>::size() const
{
  return cache_.size();
}

template <class ObjectType>
void LRUCache<ObjectType>::print_cache() const
{
  for ( auto const & item : cache_ )
  {
    cout << hex << uppercase << item.first << dec << nouppercase << endl;
  }
}

template<DependencyType DepType>
size_t AlfalfaPlayer::FrameDependency::increase_count( const size_t hash )
{
  DependencyVertex vertex{ DepType, hash };

  if ( ref_counter_.count( vertex ) ) {
    ref_counter_[ vertex ] += 1;
  }
  else {
    ref_counter_[ vertex ] = 1;
  }

  return ref_counter_[ vertex ];
}

template<DependencyType DepType>
size_t AlfalfaPlayer::FrameDependency::decrease_count( const size_t hash )
{
  DependencyVertex vertex{ DepType, hash };

  if ( ref_counter_.count( vertex ) and ref_counter_[ vertex ]  > 0 ) {
    ref_counter_[ vertex ] -= 1;
    return ref_counter_[ vertex ];
  }

  ref_counter_.erase( vertex );
  return 0;
}

template<DependencyType DepType>
size_t AlfalfaPlayer::FrameDependency::get_count( const size_t hash ) const
{
  DependencyVertex vertex{ DepType, hash };

  const auto & ref_count = ref_counter_.find( vertex );
  if ( ref_count == ref_counter_.end() ) {
    return 0;
  } else {
    return ref_count->second;
  }
}

void AlfalfaPlayer::FrameDependency::update_dependencies( const FrameInfo & frame,
							  RasterAndStateCache & cache )
{
  unresolved_.erase( DependencyVertex{ RASTER, frame.target_hash().output_hash } );
  unresolved_.erase( DependencyVertex{ STATE, frame.target_hash().state_hash } );

  Optional<size_t> hash[] = {
    frame.source_hash().last_hash, frame.source_hash().golden_hash,
    frame.source_hash().alt_hash
  };

  for ( int i = 0; i < 3; i++ ) {
    if ( hash[ i ].initialized() ) {
      if ( not cache.raster_cache().has( hash[ i ].get() ) ) {
        increase_count<RASTER>( hash[ i ].get() );
        unresolved_.insert( DependencyVertex{ RASTER, hash[ i ].get() } );
      }
    }
  }

  if ( frame.source_hash().state_hash.initialized() ) {
    if ( not cache.state_cache().has( frame.source_hash().state_hash.get() ) ) {
      increase_count<STATE>( frame.source_hash().state_hash.get() );
      unresolved_.insert( DependencyVertex{ STATE,
        frame.source_hash().state_hash.get() } );
    }
  }
}

void AlfalfaPlayer::FrameDependency::update_dependencies_forward( const FrameInfo & frame,
								  RasterAndStateCache & cache )
{
  Optional<size_t> hash[] = {
    frame.source_hash().last_hash, frame.source_hash().golden_hash,
    frame.source_hash().alt_hash
  };

  for ( int i = 0; i < 3; i++ ) {
    if ( hash[ i ].initialized() ) {
      if ( not cache.raster_cache().has( hash[ i ].get() ) ) {
        decrease_count<RASTER>( hash[ i ].get() );
      }
    }
  }

  if ( frame.source_hash().state_hash.initialized() ) {
    if ( not cache.state_cache().has( frame.source_hash().state_hash.get() ) ) {
      decrease_count<STATE>( frame.source_hash().state_hash.get() );
    }
  }
}

bool AlfalfaPlayer::FrameDependency::all_resolved() const
{
  return unresolved_.size() == 0;
}

tuple<AlfalfaPlayer::SwitchPath, Optional<AlfalfaPlayer::TrackPath>,
AlfalfaPlayer::FrameDependency>
AlfalfaPlayer::get_min_switch_seek( const size_t output_hash )
{
  auto frames = video_.get_frames_by_output_hash( output_hash );

  size_t min_cost = SIZE_MAX;
  SwitchPath min_switch_path;
  min_switch_path.cost = SIZE_MAX;
  Optional<TrackPath> min_track_path;
  FrameDependency min_dependencies;

  for ( auto target_frame : frames ) {
    auto switches = video_.get_switches_with_frame( target_frame.frame_id() );

    for ( auto sw : switches ) {
      size_t cost = 0;
      FrameDependency dependencies;

      size_t cur_switch_frame_index = sw.switch_start_index;
      for ( auto frame : sw.frames ) {
        cost += frame.length();

        dependencies.update_dependencies( frame, cache_ );

        if ( dependencies.all_resolved() ) {
          break;
        }
        cur_switch_frame_index++;
      }

      if ( not dependencies.all_resolved() ) {
        auto track_seek = get_track_seek( sw.from_track_id, sw.from_frame_index,
          dependencies );

        if ( get<2>( track_seek ) == SIZE_MAX ) {
          cost = SIZE_MAX;
          break;
        }

        cost += get<2>( track_seek );

        if ( cost < min_cost ) {
          min_cost = cost;

          min_switch_path.from_track_id = sw.from_track_id;
          min_switch_path.to_track_id = sw.to_track_id;
          min_switch_path.from_frame_index = sw.from_frame_index;
          min_switch_path.to_frame_index = sw.to_frame_index;
          min_switch_path.switch_start_index = 0;
          min_switch_path.switch_end_index = cur_switch_frame_index + 1;
          min_switch_path.cost = min_cost;

          min_track_path.clear();
          min_track_path.initialize( TrackPath{ sw.from_track_id,
            get<0>( track_seek ), sw.from_frame_index + 1,
            get<2>( track_seek ) } );
          min_dependencies = get<1>( track_seek );
        }
      }
      else {
        if ( cost < min_cost ) {
          min_cost = cost;

          min_switch_path.from_track_id = sw.from_track_id;
          min_switch_path.to_track_id = sw.to_track_id;
          min_switch_path.from_frame_index = sw.from_frame_index;
          min_switch_path.to_frame_index = sw.to_frame_index;
          min_switch_path.switch_start_index = 0;
          min_switch_path.switch_end_index = cur_switch_frame_index + 1;
          min_switch_path.cost = min_cost;

          min_dependencies = dependencies;
        }
      }
    }
  }

  return make_tuple( min_switch_path, min_track_path, min_dependencies );
}

tuple<size_t, AlfalfaPlayer::FrameDependency, size_t>
AlfalfaPlayer::get_track_seek( const size_t track_id, const size_t from_frame_index,
                               FrameDependency dependencies )
{
  int cur_frame_index = (int) from_frame_index;
  // Since range specified in get_frames_reverse is inclusive on both sides, add 1 to
  // to_frame_index.
  size_t to_frame_index = ( cur_frame_index - (int) MAX_NUM_FRAMES + 1 ) >= 0 ?
    ( from_frame_index - MAX_NUM_FRAMES + 1 ) : 0;
  auto frames_backward = video_.get_frames_reverse( track_id, from_frame_index, to_frame_index );

  if ( frames_backward.size() == 0 ) {
    return make_tuple( -1, dependencies, SIZE_MAX );
  }

  size_t cost = 0;
  while ( cur_frame_index >= 0 ) {
    for ( auto frame : frames_backward ) {
      cost += frame.length();

      dependencies.update_dependencies( frame, cache_ );

      if ( dependencies.all_resolved() ) {
        return make_tuple( (size_t) cur_frame_index, dependencies, cost );
      }
      cur_frame_index--;
    }
    if ( cur_frame_index >= 0 ) {
      to_frame_index = ( cur_frame_index - (int) MAX_NUM_FRAMES + 1 ) >= 0 ?
        ( from_frame_index - MAX_NUM_FRAMES + 1 ) : 0;
      frames_backward = video_.get_frames_reverse( track_id, (size_t) cur_frame_index, to_frame_index );
    }
  }

  return make_tuple( from_frame_index, dependencies, SIZE_MAX );
}

tuple<AlfalfaPlayer::TrackPath, AlfalfaPlayer::FrameDependency>
AlfalfaPlayer::get_min_track_seek( const size_t output_hash )
{
  size_t min_cost = SIZE_MAX;
  TrackPath min_track_path;
  min_track_path.cost = SIZE_MAX;

  FrameDependency min_frame_dependency;

  auto track_ids = video_.get_track_ids();

  for ( auto frame : video_.get_frames_by_output_hash( output_hash ) ) {
    for ( auto track_data : video_.get_track_data_by_frame_id( frame.frame_id() ) ) {
      tuple<size_t, FrameDependency, size_t> seek =
        get_track_seek( track_data.track_id, track_data.frame_index );

      if ( get<2>( seek ) < min_cost ) {
        min_cost = get<2>( seek );
        min_track_path = TrackPath{ track_data.track_id, get<0>( seek ),
                                    track_data.frame_index + 1,
                                    min_cost };
        min_frame_dependency = get<1>( seek );
      }
    }
  }

  return make_tuple( min_track_path, min_frame_dependency );
}

AlfalfaPlayer::AlfalfaPlayer( const std::string & server_address )
  : video_( server_address ),
    web_( video_.get_url() ),
    cache_(),
    downloaded_frame_bytes_( 0 ),
    current_frame_seq_(),
    current_download_pt_index_( 0 ),
    current_playhead_index_( 0 ),
    video_width_( video_.get_video_width() ),
    video_height_( video_.get_video_height() )
{}

Decoder AlfalfaPlayer::get_decoder( const FrameInfo & frame )
{
  References refs( video_width_, video_height_ );
  DecoderState state( video_width_, video_height_ );

  if ( frame.source_hash().last_hash.initialized() ) {
    refs.last = cache_.raster_cache().get( frame.source_hash().last_hash.get() );
  }

  if ( frame.source_hash().golden_hash.initialized() ) {
    refs.golden = cache_.raster_cache().get( frame.source_hash().golden_hash.get() );
  }

  if ( frame.source_hash().alt_hash.initialized() ) {
    refs.alternative_reference =
      cache_.raster_cache().get( frame.source_hash().alt_hash.get() );
  }

  if ( frame.source_hash().state_hash.initialized() ) {
    state = cache_.state_cache().get( frame.source_hash().state_hash.get() );
  }

  return Decoder( state, refs );
}

AlfalfaPlayer::FrameDependency AlfalfaPlayer::follow_track_path( TrackPath path,
                                                                  FrameDependency dependencies )
{
  References refs( video_width_, video_height_ );
  DecoderState state( video_width_, video_height_ );

  size_t from_frame_index = path.start_index;
  size_t to_frame_index = ( from_frame_index + MAX_NUM_FRAMES ) >= path.end_index ?
    path.end_index : ( from_frame_index + MAX_NUM_FRAMES );
  auto frames = video_.get_frames( path.track_id, from_frame_index, to_frame_index );

  while ( from_frame_index < path.end_index ) {
    for ( auto frame : frames ) {
      Decoder decoder = get_decoder( frame );
      pair<bool, RasterHandle> output = decoder.get_frame_output( web_.get_chunk( frame ) );
      cache_.put( decoder );
      cache_.raster_cache().put( output.second.hash(), output.second );
      dependencies.update_dependencies_forward( frame, cache_ );
    }
    from_frame_index += MAX_NUM_FRAMES;
    if ( from_frame_index < path.end_index ) {
      to_frame_index = ( from_frame_index + MAX_NUM_FRAMES ) >= path.end_index ?
        path.end_index : ( from_frame_index + MAX_NUM_FRAMES );
      frames = video_.get_frames( path.track_id, from_frame_index, to_frame_index );
    }
  }

  return dependencies;
}

AlfalfaPlayer::FrameDependency AlfalfaPlayer::follow_switch_path( SwitchPath path,
                                                                  FrameDependency dependencies )
{
  References refs( video_width_, video_height_ );
  DecoderState state( video_width_, video_height_ );

  auto frames = video_.get_frames( path.from_track_id, path.to_track_id,
    path.from_frame_index, path.switch_start_index, path.switch_end_index );

  for ( auto frame : frames ) {
    Decoder decoder = get_decoder( frame );
    pair<bool, RasterHandle> output = decoder.get_frame_output( web_.get_chunk( frame ) );
    cache_.put( decoder );
    cache_.raster_cache().put( output.second.hash(), output.second );
    dependencies.update_dependencies_forward( frame, cache_ );
  }

  return dependencies;
}

Optional<RasterHandle> AlfalfaPlayer::get_raster_track_path( const size_t output_hash )
{
  auto track_seek = get_min_track_seek( output_hash );

  if ( get<0>( track_seek ).cost == SIZE_MAX) {
    return {};
  }

  follow_track_path( get<0>( track_seek ), get<1>( track_seek ) );
  return cache_.raster_cache().get( output_hash );
}

Optional<RasterHandle> AlfalfaPlayer::get_raster_switch_path( const size_t output_hash )
{
  auto switch_seek = get_min_switch_seek( output_hash );

  if ( get<0>( switch_seek ).cost == SIZE_MAX ) {
    return {};
  }

  Optional<TrackPath> & extra_track_seek = get<1>( switch_seek );
  FrameDependency & dependencies = get<2>( switch_seek );

  if ( extra_track_seek.initialized() ) {
    dependencies = follow_track_path( extra_track_seek.get(), dependencies );
  }

  follow_switch_path( get<0>( switch_seek ), dependencies );

  return cache_.raster_cache().get( output_hash );
}

Optional<RasterHandle> AlfalfaPlayer::get_raster( const size_t output_hash,
                                                  PathType path_type, bool verbose )
{
  if ( verbose ) {
    auto track_seek = get_min_track_seek( output_hash );
    auto switch_seek = get_min_switch_seek( output_hash );

    if ( get<0>( track_seek ).cost < SIZE_MAX ) {
      cout << "> Track seek:" << endl << get<0>( track_seek ) << endl;
    }

    if ( get<0>( switch_seek ).cost < SIZE_MAX ) {
      cout << "> Switch seek:" << endl;

      if ( get<1>( switch_seek ).initialized() ) {
        cout << get<1>( switch_seek ).get() << endl;
      }

      cout << get<0>( switch_seek ) << endl;
    }
  }

  switch( path_type ) {
  case TRACK_PATH:
  {
    Optional<RasterHandle> result = get_raster_track_path( output_hash );

    if ( not result.initialized() and verbose ) {
      cout << "No track paths found." << endl;
    }

    return result;
  }
  case SWITCH_PATH:
  {
    Optional<RasterHandle> result = get_raster_switch_path( output_hash );

    if ( not result.initialized() and verbose ) {
      cout << "No switch paths found." << endl;
    }

    return result;
  }

  case MINIMUM_PATH:
  default:
    auto track_seek = get_min_track_seek( output_hash );
    auto switch_seek = get_min_switch_seek( output_hash );

    if ( get<0>( track_seek ).cost <= get<0>( switch_seek ).cost ) {
      return get_raster( output_hash, TRACK_PATH, false );
    }
    else {
      return get_raster( output_hash, SWITCH_PATH, false );
    }
  }
}

RasterHandle
AlfalfaPlayer::get_raster_sequential( const size_t dri )
{
  FrameInfoWrapper frame_info_wrapper =
    current_frame_seq_.at( current_playhead_index_++ );
  if ( frame_info_wrapper.dri > dri ) {
    throw runtime_error( "Invalid dri requested in sequential play" );
  }
  while ( frame_info_wrapper.dri <= dri and (not frame_info_wrapper.frame_info.shown() ) ) {
    Decoder decoder = get_decoder( frame_info_wrapper.frame_info );
    pair<bool, RasterHandle> output = decoder.get_frame_output( web_.get_chunk( frame_info_wrapper.frame_info ) );
    cache_.put( decoder );
    cache_.raster_cache().put( output.second.hash(), output.second );
    frame_info_wrapper = current_frame_seq_.at( current_playhead_index_++ );
  }
  Decoder decoder = get_decoder( frame_info_wrapper.frame_info );
  pair<bool, RasterHandle> output = decoder.get_frame_output( web_.get_chunk( frame_info_wrapper.frame_info ) );
  cache_.put( decoder );
  cache_.raster_cache().put( output.second.hash(), output.second );
  return output.second;
}

const VP8Raster & AlfalfaPlayer::example_raster()
{
  Decoder temp( video_width_, video_height_ );
  return temp.example_raster();
}

Optional<Chunk>
AlfalfaPlayer::get_next_chunk()
{
  if ( current_download_pt_index_ >= current_frame_seq_.size() ) {
    // No chunks remaining to get
    return {};
  }

  FrameInfo frame = current_frame_seq_.at( current_download_pt_index_ ).frame_info;
  Chunk chunk = web_.get_chunk( frame );

  frame_cache_.put( frame.frame_id(), chunk );
  downloaded_frame_bytes_ += frame.length();
  current_download_pt_index_++;

  return make_optional<Chunk>( true, chunk );
}

bool
AlfalfaPlayer::determine_feasibility( const vector<FrameInfoWrapper> prospective_track,
                                      const size_t throughput_estimate )
{
  long buffer_size = (long) downloaded_frame_bytes_;
  size_t track_index = current_playhead_index_;
  size_t prospective_track_index = 0;

  /* Currently assuming that the switch is made just before the frame at the index
     switching_track_index in the current_frame_seq_; frames in prospective_track_ are
     subsequently played. */
  while( track_index < current_download_pt_index_ or
         prospective_track_index < prospective_track.size() )
  {
    size_t frame_id;
    size_t frame_length;
    if ( track_index < current_download_pt_index_ ) {
      FrameInfo frame = current_frame_seq_.at( track_index ).frame_info;
      frame_id = frame.frame_id();
      frame_length = frame.length();
      track_index++;
    } else {
      FrameInfo frame = prospective_track.at( prospective_track_index ).frame_info;
      frame_id = frame.frame_id();
      frame_length = frame.length();
      prospective_track_index++;
    }
    buffer_size += throughput_estimate;
    if ( not frame_cache_.has( frame_id ) ) {
      buffer_size -= ( frame_length );
    }

    // If buffer size is ever negative, proposed sequence of frames is infeasible
    if ( buffer_size < 0 )
      return false;
  }
  return true;
}

FrameSequence
AlfalfaPlayer::seek_track_at_dri( const size_t track_id, const size_t dri )
{
  size_t frame_index = video_.get_frame_index_by_displayed_raster_index( track_id, dri );
  auto track_seek = get_track_seek( track_id, frame_index );
  size_t from_frame_index = get<0>( track_seek );
  vector<FrameInfo> frame_seq_tmp = video_.get_frames( track_id, from_frame_index,
    video_.get_track_size( track_id ) );
  vector<FrameInfoWrapper> frame_seq;

  double min_ssim = SIZE_MAX;
  int dri_index = (int) dri;
  for ( FrameInfo frame : frame_seq_tmp ) {
    frame_seq.push_back( FrameInfoWrapper( frame, track_id, ( size_t ) dri_index ) );
    if ( frame.shown() ) {
      double ssim = video_.get_quality( dri_index, frame );
      if ( ssim < min_ssim )
        min_ssim = ssim;
      dri_index++;
    }
  }

  return FrameSequence( frame_seq, min_ssim );
}

vector<SwitchInfo>
AlfalfaPlayer::seek_track_through_switch_at_dri( const size_t from_track_id, const size_t dri, const size_t to_track_id )
{
  vector<SwitchInfo> switch_infos;
  size_t frame_index = video_.get_frame_index_by_displayed_raster_index( from_track_id, dri );

  // Switch can start at any index in the track
  for ( size_t switch_start_index = frame_index; switch_start_index < video_.get_track_size( from_track_id ); switch_start_index++ ) {
    // Get all switches that contain this frame
    auto switch_infos = video_.get_switches_with_frame(
      video_.get_frame( from_track_id, switch_start_index ).frame_id );
    for ( auto switch_info : switch_infos ) {
      // First, verify that the switch starts from where we want it to start
      if ( switch_info.from_track_id != from_track_id or
           switch_info.to_track_id != to_track_id or
           switch_info.from_frame_index != switch_start_index )
        continue;

      switch_infos.push_back( switch_info );
    }
  }
  return switch_infos;
}

FrameSequence
AlfalfaPlayer::get_frame_seq( const SwitchInfo & switch_info, const size_t dri )
{
  int dri_index = (int) dri;
  double min_ssim = SIZE_MAX;
  vector<FrameInfoWrapper> switch_frame_seq_vec;
  for ( FrameInfo frame : video_.get_frames(
    switch_info.from_track_id, current_download_pt_index_, switch_info.from_frame_index ) ) {
    switch_frame_seq_vec.push_back( FrameInfoWrapper( frame,
                                                      switch_info.from_track_id,
                                                      ( size_t ) dri_index ) );
    if ( frame.shown() ) {
      double ssim = video_.get_quality( dri_index, frame );
      if ( ssim < min_ssim )
        min_ssim = ssim;
      dri_index++;
    }
  }

  for ( FrameInfo frame : switch_info.frames ) {
    // Frames in switches don't have track ids: we pick an invalid track id for now
    switch_frame_seq_vec.push_back( FrameInfoWrapper( frame,
                                                      SIZE_MAX,
                                                      ( size_t ) dri_index ) );
    if ( frame.shown() ) {
      double ssim = video_.get_quality( dri_index, frame );
      if ( ssim < min_ssim )
        min_ssim = ssim;
      dri_index++;
    }
  }

  for ( FrameInfo frame : video_.get_frames(
    switch_info.to_track_id, switch_info.to_frame_index, video_.get_track_size( switch_info.to_track_id ) ) ) {
    switch_frame_seq_vec.push_back( FrameInfoWrapper( frame,
                                                      switch_info.to_track_id,
                                                      ( size_t ) dri_index ) );
    if ( frame.shown() ) {
      double ssim = video_.get_quality( dri_index, frame );
      if ( ssim < min_ssim )
        min_ssim = ssim;
      dri_index++;
    }
  }

  return FrameSequence( switch_frame_seq_vec, min_ssim );
}

vector<FrameSequence>
AlfalfaPlayer::get_sequential_play_options( const size_t dri, const size_t throughput_estimate )
{
  // First add all frame sequences that are a produce of "seeks": note that staying on
  // current track is implicitly handled under this case
  vector<FrameSequence> frame_seqs;
  for ( size_t track_id : video_.get_track_ids() ) {
    FrameSequence frame_seq = seek_track_at_dri( track_id, dri );
    if ( determine_feasibility( frame_seq.frame_seq, throughput_estimate ) ) {
      frame_seqs.push_back( frame_seq );
    }
  }

  // Now, insert frame sequences derived from switches

  // If not currently on any track, or if we're already past the end of the frame buffer,
  // no switches to return
  if ( current_download_pt_index_ == 0 or
       ( ( current_download_pt_index_ - 1 ) >= current_frame_seq_.size() ) ) {
    return frame_seqs;
  }

  size_t current_track_id = current_frame_seq_.at( current_download_pt_index_ - 1).track_id;
  // If currently on a switch, we can't take another switch to reach another track
  // We have to use a random seek to move to a new track
  if ( current_track_id == SIZE_MAX ) {
    return frame_seqs;
  }

  for ( size_t to_track_id : video_.get_connected_track_ids( current_track_id ) ) {
    vector<SwitchInfo> switch_infos = seek_track_through_switch_at_dri( current_track_id, dri, to_track_id );
    for ( auto switch_info : switch_infos ) {
      FrameSequence frame_seq = get_frame_seq( switch_info, dri );
      // Take the earliest switch that is feasible
      if ( determine_feasibility( frame_seq.frame_seq, throughput_estimate ) ) {
        frame_seqs.push_back( frame_seq );
        break;
      }
    }
  }

  return frame_seqs;
}

vector<FrameSequence>
AlfalfaPlayer::get_random_seek_play_options( const size_t dri )
{
  vector<FrameSequence> frame_seqs;
  // Very similar to the code above for get_sequential_play_options, but with one key
  // differece: no feasibility checks
  for ( size_t track_id : video_.get_track_ids() ) {
    FrameSequence frame_seq = seek_track_at_dri( track_id, dri );
    frame_seqs.push_back( frame_seq );
  }

  return frame_seqs;
}

void
AlfalfaPlayer::set_current_frame_seq( const size_t dri, const size_t throughput_estimate, bool random_seek )
{
  // First, get the options to be played, depending on whether we're doing a random
  // seek or not
  vector<FrameSequence> play_options;
  if ( random_seek ) {
    play_options = get_random_seek_play_options( dri );
  } else {
    play_options = get_sequential_play_options( dri, throughput_estimate );
  }

  // Now, pick the optimal frame sequence, based on a QoS metric.
  // For now, our QoS metric is the minimum SSIM score.
  size_t optimal_frame_sequence_index = 0;
  size_t optimal_cost = 0;

  size_t i;
  for ( i = 0; i < play_options.size(); i++ ) {
    if ( play_options.at( i ).min_ssim > optimal_cost ) {
      optimal_frame_sequence_index = i;
      optimal_cost = play_options.at( i ).min_ssim;
    }
  }

  vector<FrameInfoWrapper> next_frames = play_options.at( optimal_frame_sequence_index ).frame_seq;

  if ( random_seek ) {
    current_frame_seq_ = next_frames;
    current_download_pt_index_ = 0;
    current_playhead_index_ = 0;
  } else {
    // download_pt_index and playhead_index are all unchanged
    size_t i;
    size_t current_frame_seq_size = current_frame_seq_.size();
    for ( i = current_download_pt_index_; i < current_frame_seq_size; i++ ) {
      current_frame_seq_.pop_back();
    }
    for ( FrameInfoWrapper frame_wrapper : next_frames ) {
      current_frame_seq_.push_back( frame_wrapper );
    }
  }
}

void AlfalfaPlayer::clear_cache()
{
  cache_.clear();
}

void AlfalfaPlayer::print_cache() const
{
  cache_.print_cache();
}

void RasterAndStateCache::print_cache() const
{
  cout << "Raster in cache:" << endl;
  raster_cache_.print_cache();

  cout << "###" << endl << endl << "States in cache:" << endl;
  state_cache_.print_cache();
  cout << "###" << endl;
}

size_t RasterAndStateCache::size() const
{
  return raster_cache().size() + state_cache().size();
}

void RasterAndStateCache::clear()
{
  raster_cache_.clear();
  state_cache_.clear();
}
