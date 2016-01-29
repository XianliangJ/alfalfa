#include <cstdlib>
#include <iostream>
#include <chrono>

#include <sysexits.h>

#include "alfalfa_video_client.hh"
#include "display.hh"
#include "frame_fetcher.hh"
#include "video_map.hh"

using namespace std;
using namespace std::chrono;
using AbridgedFrameInfo = AlfalfaProtobufs::AbridgedFrameInfo;

int main( const int argc, char const *argv[] )
{
  if ( argc != 2 ) {
    cerr << "Usage: " << argv[ 0 ] << " <server-address>" << "\n";
    return EX_USAGE;
  }

  const AlfalfaVideoClient video { argv[ 1 ] };
  Decoder decoder( video.get_video_width(), video.get_video_height() );
  VideoDisplay display { decoder.example_raster() };

  FrameFetcher fetcher { video.get_url() };

  const unsigned int rasters_to_display = video.get_raster_count();
  
  VideoMap video_map { argv[ 1 ], rasters_to_display };

  auto program_startup = steady_clock::now();

  /* start playing */
  auto next_raster_time = steady_clock::now();
  bool playing = false;
  auto last_feasibility_analysis = steady_clock::now();
  unsigned int analysis_generation = video_map.analysis_generation();

  deque<AnnotatedFrameInfo> current_future_of_track;
  AnnotatedFrameInfo last_frame_decoded = VideoMap::no_frame();

  const auto frame_interval = microseconds( lrint( 1000000.0 / 24.0 ) );

  int last_minute = 0;
  bool first_fetch = true;
  bool seeking = false;

  while ( true ) {
    /* do a seek? */
    auto current_time = steady_clock::now();
    duration<double> age_of_program = current_time - program_startup;
    int this_minute = int( age_of_program.count() / 60 );
    if ( this_minute != last_minute ) {
      unsigned int raster_target = 24 * 60 * (this_minute * 2);
      cout << "Seeking to minute " << this_minute * 2 << "\n";
      last_frame_decoded.track_id = 6;
      last_frame_decoded.track_index = raster_target;
      last_minute = this_minute;
      /* force recalculation */
      analysis_generation = 0;
      last_feasibility_analysis = program_startup;
      seeking = true;
    }

    /* kick off a feasibility analysis? */
    const auto now = steady_clock::now();
    if ( now - last_feasibility_analysis > milliseconds( 500 ) ) {
      video_map.update_annotations( fetcher.estimated_bytes_per_second() * 0.7,
				    fetcher.frame_db_snapshot() );
      last_feasibility_analysis = now;
    }

    /* is a new analysis available? */
    const unsigned int new_analysis_generation = video_map.analysis_generation();
    if ( new_analysis_generation != analysis_generation ) {
      current_future_of_track = video_map.best_plan( last_frame_decoded, playing, seeking );
      fetcher.set_frame_plan( current_future_of_track );

      if ( first_fetch ) {
	auto current_time = steady_clock::now();
	duration<double> age_of_program = current_time - program_startup;
	cout << "first chunk request logged on server at " << age_of_program.count() << "\n";
	first_fetch = false;
      }

      analysis_generation = new_analysis_generation;
      //      video_map.report_feasibility();
      //      cerr << "kilobits per second: " << fetcher.estimated_bytes_per_second() * 8 * 0.8 / 1000.0 << "\n";
    }

    /* are we out of available track? */
    if ( current_future_of_track.empty() ) {
      cerr << "Stalling [out of track]\n";
      playing = false;
      this_thread::sleep_for( frame_interval );
      continue;
    }
    
    /* should we resume from a stall? */
    if ( not playing ) {
      if ( fetcher.is_plan_feasible() ) {
	playing = true;
	next_raster_time = steady_clock::now();
	cerr << "Playing.\n";
      } else {
	this_thread::sleep_for( 4 * frame_interval );
	continue;
      }
    }

    /* do we need to stall? */
    if ( not fetcher.has_frame( current_future_of_track.front() ) ) {
      /* stall */
      cerr << "Stalling.\n";
      playing = false;
      continue;
    }

    /* pop the frame from the plan and apply it */
    last_frame_decoded = current_future_of_track.front(); 
    current_future_of_track.pop_front();
    seeking = false;
    const string coded_frame = fetcher.coded_frame( last_frame_decoded );

    const Optional<RasterHandle> raster = decoder.parse_and_decode_frame( coded_frame );
    if ( raster.initialized() ) {
      this_thread::sleep_until( next_raster_time );
      display.draw( raster.get() );
      next_raster_time += frame_interval;

      auto display_time = steady_clock::now();
      duration<double> age_of_program = display_time - program_startup;
      cout << "Time in video " << double( last_frame_decoded.timestamp ) / 24.0 << " which is frame " << last_frame_decoded.timestamp << " displayed at system time " << age_of_program.count() << " with vertical resolution 0  and ssim score " << last_frame_decoded.quality << "\n";
    }
  }

  return EXIT_SUCCESS;
}
