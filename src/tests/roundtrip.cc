#include <iostream>

#include "ivf.hh"
#include "uncompressed_chunk.hh"
#include "decoder_state.hh"

using namespace std;

int main( int argc, char *argv[] )
{
  try {
    if ( argc != 2 ) {
      cerr << "Usage: " << argv[ 0 ] << " FILENAME" << endl;
      return EXIT_FAILURE;
    }

    IVF file( argv[ 1 ] );

    if ( file.fourcc() != "VP80" ) {
      throw Unsupported( "not a VP8 file" );
    }

    /* find first key frame */
    uint32_t frame_no = 0;

    while ( frame_no < file.frame_count() ) {
      UncompressedChunk uncompressed_chunk( file.frame( frame_no ), file.width(), file.height() );
      if ( uncompressed_chunk.key_frame() ) {
	break;
      }
      frame_no++;
    }

    DecoderState decoder_state( file.width(), file.height() );

    /* write IVF header */
    cout << "DKIF";
    cout << uint8_t(0) << uint8_t(0); /* version */
    cout << uint8_t(32) << uint8_t(0); /* header length */
    cout << "VP80"; /* fourcc */
    cout << uint8_t(file.width() & 0xff) << uint8_t((file.width() >> 8) & 0xff); /* width */
    cout << uint8_t(file.height() & 0xff) << uint8_t((file.height() >> 8) & 0xff); /* height */
    cout << uint8_t(1) << uint8_t(0) << uint8_t(0) << uint8_t(0); /* bogus frame rate */
    cout << uint8_t(1) << uint8_t(0) << uint8_t(0) << uint8_t(0); /* bogus time scale */

    const uint32_t le_num_frames = htole32( file.frame_count() - frame_no );
    cout << string( reinterpret_cast<const char *>( &le_num_frames ), sizeof( le_num_frames ) ); /* num frames */

    cout << uint8_t(0) << uint8_t(0) << uint8_t(0) << uint8_t(0); /* fill out header */

    for ( uint32_t i = frame_no; i < file.frame_count(); i++ ) {
      vector< uint8_t > serialized_frame;

    UncompressedChunk whole_frame( file.frame( i ), file.width(), file.height() );
      if ( whole_frame.key_frame() ) {
	const KeyFrame parsed_frame = decoder_state.parse_and_apply<KeyFrame>( whole_frame );
	serialized_frame = parsed_frame.serialize( decoder_state.probability_tables );
      } else {
	const InterFrame parsed_frame = decoder_state.parse_and_apply<InterFrame>( whole_frame );
	serialized_frame = parsed_frame.serialize( decoder_state.probability_tables );
      }

      /* verify equality of original and re-encoded frame */
      if ( file.frame( i ).size() != serialized_frame.size() ) {
	throw internal_error( "roundtrip failure", "frame size mismatch. wanted " + to_string( file.frame( i ).size() ) + ", got " + to_string( serialized_frame.size() ) );
      } else {
	for ( unsigned int j = 0; j < file.frame( i ).size(); j++ ) {
	  if ( file.frame( i )( j ).octet() != serialized_frame.at( j ) ) {
	    throw internal_error( "roundtrip failure", "frame contents mismatch" );
	  }
	}
      }

      /* write size of frame */
      const uint32_t le_size = htole32( serialized_frame.size() );
      cout << string( reinterpret_cast<const char *>( &le_size ), sizeof( le_size ) ); /* size */

      cout << uint8_t(0) << uint8_t(0) << uint8_t(0) << uint8_t(0); /* fill out header */
      cout << uint8_t(0) << uint8_t(0) << uint8_t(0) << uint8_t(0); /* fill out header */

      /* write the frame */
      fwrite( &serialized_frame.at( 0 ), serialized_frame.size(), 1, stdout );
    }
  } catch ( const exception & e ) {
    print_exception( argv[ 0 ], e );
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
