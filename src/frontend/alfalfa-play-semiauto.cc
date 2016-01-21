#include <cstdlib>
#include <iostream>

#include <sysexits.h>

#include "alfalfa_player.hh"
#include "display.hh"
#include "frame_fetcher.hh"

using namespace std;

const size_t THROUGHPUT_ESTIMATE = 1000000;
const size_t SWITCHING_DECISION_FREQUENCY = 100;

size_t get_num_dris( const AlfalfaVideoClient & video )
{
  size_t num_dris = 0;
  // All valid videos must have a track id of 0
  for ( const auto & frame : video.get_frames( 0, 0, video.get_track_size( 0 ) ) ) {
    if ( frame.shown() ) {
      num_dris++;
    }
  }
  return num_dris;
}

int main( const int argc, char const *argv[] )
{
  if ( argc != 3 ) {
    cerr << "Usage: " << argv[ 0 ] << " <server-address> <framestore-url>" << "\n";
    return EX_USAGE;
  }

  AlfalfaVideoClient video { argv[ 1 ] };
  AlfalfaPlayer player { argv[ 1 ] };
  VideoDisplay display { player.example_raster() };

  FrameFetcher fetcher { argv[ 2 ] };

  size_t dri = 0;  // Start off with the first displayed raster
  size_t num_dris = get_num_dris( video );

  while ( dri < num_dris ) {
    if ( dri % SWITCHING_DECISION_FREQUENCY == 0 ) {
      player.set_current_frame_seq( dri, THROUGHPUT_ESTIMATE, false );
    }

    RasterHandle raster = player.get_raster_sequential( dri );
    display.draw( raster );
    dri++;
  }
  
  return EXIT_SUCCESS;
}
