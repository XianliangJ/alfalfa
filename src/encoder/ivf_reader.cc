#include "ivf_reader.hh"

IVFReader::IVFReader( const std::string & filename )
  : player_( filename )
{}

Optional<RasterHandle> IVFReader::get_next_frame()
{
  if ( player_.eof() ) {
    return Optional<RasterHandle>();
  }

  return make_optional<RasterHandle>( true, player_.advance() );
}
