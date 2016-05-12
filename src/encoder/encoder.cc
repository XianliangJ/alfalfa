#include <limits>

#include "encoder.hh"
#include "frame_header.hh"
#include "ssim.hh"

using namespace std;

QuantIndices::QuantIndices()
  : y_ac_qi(), y_dc(), y2_dc(), y2_ac(), uv_dc(), uv_ac()
{}

KeyFrame Encoder::make_empty_frame( const uint16_t width, const uint16_t height )
{
  BoolDecoder data { { nullptr, 0 } };
  KeyFrame frame { true, width, height, data };
  frame.parse_macroblock_headers( data, ProbabilityTables {} );
  return frame;
}

template <unsigned int size>
template <class PredictionMode>
void VP8Raster::Block<size>::intra_predict( const PredictionMode mb_mode, TwoD<uint8_t> & output )
{
  TwoDSubRange<uint8_t, size, size> subrange( output, 0, 0 );
  intra_predict( mb_mode, subrange );
}

/* Encoder */
Encoder::Encoder( const string & output_filename, const uint16_t width, const uint16_t height )
  : ivf_writer_( output_filename, "VP80", width, height, 1, 1 ), width_( width ), height_( height )
{}

template<unsigned int size>
uint32_t Encoder::variance( const VP8Raster::Block<size> & block,
                            const TwoD<uint8_t> & prediction,
                            const uint16_t dc_factor, const uint16_t ac_factor )
{
  return Encoder::variance( block, TwoDSubRange<uint8_t, size, size>( prediction, 0, 0 ),
                            dc_factor, ac_factor );
}

template<unsigned int size>
uint32_t Encoder::variance( const VP8Raster::Block<size> & block,
                            const TwoDSubRange<uint8_t, size, size> & prediction,
                            const uint16_t dc_factor, const uint16_t ac_factor )
{
  uint32_t sse = 0;
  int32_t sum = 0;

  for ( size_t i = 0; i < size; i++ ) {
    for ( size_t j = 0; j < size; j++ ) {
      int16_t diff = ( block.at( i, j ) - prediction.at( i, j ) ) / ( ( i + j == 0 ) ? dc_factor : ac_factor );
      sum += diff;
      sse += diff * diff;
    }
  }

  return sse - ( ( int64_t )sum * sum ) / ( size * size );
}

template <class MacroblockType>
size_t Encoder::luma_mb_intra_predict( const VP8Raster::Macroblock & original_mb,
                                       VP8Raster::Macroblock & reconstructed_mb,
                                       MacroblockType & frame_mb,
                                       const vector<Quantizer> & quantizers,
                                       size_t quantizer_index,
                                       double minimum_ssim ) const
{
  // Select the best prediction mode
  const Quantizer & quantizer = quantizers.at( quantizer_index );

  frame_mb.mutable_segment_id_update().clear();
  frame_mb.mutable_segment_id_update().initialize( quantizer_index );

  uint32_t min_energy = numeric_limits<uint32_t>::max();
  mbmode min_prediction_mode = DC_PRED;
  TwoD<uint8_t> min_prediction( 16, 16 );

  for ( unsigned int prediction_mode = 0; prediction_mode < num_y_modes; prediction_mode++ ) {
    TwoD<uint8_t> prediction( 16, 16 );
    uint32_t variance_val;

    if ( prediction_mode == B_PRED ) {
      variance_val = 0;

      reconstructed_mb.Y_sub.forall_ij(
        [&] ( VP8Raster::Block4 & reconstructed_sb, unsigned int sb_column, unsigned int sb_row )
        {
          auto & original_sb = original_mb.Y_sub.at( sb_column, sb_row );
          auto & frame_sb = frame_mb.Y().at( sb_column, sb_row );

          pair<bmode, TwoD<uint8_t>> sb_prediction = luma_sb_intra_predict( original_sb,
            reconstructed_sb, frame_sb, quantizer );

          variance_val += variance( original_sb, sb_prediction.second,
            quantizer.y_dc, quantizer.y_ac );

          frame_sb.mutable_coefficients().subtract_dct( original_sb,
            TwoDSubRange<uint8_t, 4, 4>( sb_prediction.second, 0, 0 ) );

          frame_sb.mutable_coefficients() = YBlock::quantize( quantizer, frame_sb.coefficients() );
          frame_sb.set_prediction_mode( sb_prediction.first );
          frame_sb.set_Y_without_Y2();
          frame_sb.calculate_has_nonzero();

          reconstructed_sb.intra_predict( sb_prediction.first );
          frame_sb.dequantize( quantizer ).idct_add( reconstructed_sb );
        } );
    }
    else {
      reconstructed_mb.Y.intra_predict( ( mbmode )prediction_mode, prediction );
      variance_val = variance( original_mb.Y, prediction, quantizer.y_dc, quantizer.y_ac );
    }

    if ( variance_val < min_energy ) {
      min_prediction = move( prediction );
      min_prediction_mode = ( mbmode )prediction_mode;
      min_energy = variance_val;
    }
  }

  // Apply
  frame_mb.Y2().set_prediction_mode( min_prediction_mode );

  if ( min_prediction_mode != B_PRED ) { // if B_PRED is selected, it is already taken care of.
    SafeArray<int16_t, 16> walsh_input;

    frame_mb.Y().forall_ij(
      [&] ( YBlock & frame_sb, unsigned int sb_column, unsigned int sb_row )
      {
        auto & original_sb = original_mb.Y_sub.at( sb_column, sb_row );
        frame_sb.set_prediction_mode( KeyFrameMacroblock::implied_subblock_mode( min_prediction_mode ) );

        frame_sb.mutable_coefficients().subtract_dct( original_sb,
          TwoDSubRange<uint8_t, 4, 4>( min_prediction, 4 * sb_column, 4 * sb_row ) );

        walsh_input.at( sb_column + 4 * sb_row ) = frame_sb.coefficients().at( 0 );
        frame_sb.set_dc_coefficient( 0 );
        frame_sb.mutable_coefficients() = YBlock::quantize( quantizer, frame_sb.coefficients() );
        frame_sb.set_Y_after_Y2();
        frame_sb.calculate_has_nonzero();
      } );

    frame_mb.Y2().set_coded( true );
    frame_mb.Y2().mutable_coefficients().wht( walsh_input );
    frame_mb.Y2().mutable_coefficients() = Y2Block::quantize( quantizer, frame_mb.Y2().coefficients() );
    frame_mb.Y2().calculate_has_nonzero();
  }
  else {
    frame_mb.Y2().set_coded( false );
  }

  double current_ssim = ssim( original_mb.Y.contents(), reconstructed_mb.Y.contents() );

  if ( current_ssim < minimum_ssim ) {
    if ( quantizer_index + 1 < quantizers.size() ) {
      return luma_mb_intra_predict( original_mb, reconstructed_mb, frame_mb, quantizers, quantizer_index + 1, minimum_ssim );
    }
  }

  return quantizer_index;
}

template <class MacroblockType>
void Encoder::chroma_mb_intra_predict( const VP8Raster::Macroblock & original_mb,
                                       VP8Raster::Macroblock & reconstructed_mb,
                                       MacroblockType & frame_mb,
                                       const Quantizer & quantizer ) const
{
  // Select the best prediction mode
  uint32_t min_energy = numeric_limits<uint32_t>::max();
  mbmode min_prediction_mode = DC_PRED;
  TwoD<uint8_t> u_min_prediction( 8, 8 );
  TwoD<uint8_t> v_min_prediction( 8, 8 );

  for ( unsigned int prediction_mode = 0; prediction_mode < num_uv_modes; prediction_mode++ ) {
    TwoD<uint8_t> u_prediction( 8, 8 );
    TwoD<uint8_t> v_prediction( 8, 8 );

    reconstructed_mb.U.intra_predict( ( mbmode )prediction_mode, u_prediction );
    reconstructed_mb.V.intra_predict( ( mbmode )prediction_mode, v_prediction );

    uint32_t variance_val = variance( original_mb.U, u_prediction, quantizer.uv_dc, quantizer.uv_ac );
    variance_val += variance( original_mb.V, v_prediction, quantizer.uv_dc, quantizer.uv_ac );

    if ( variance_val < min_energy ) {
      u_min_prediction = move( u_prediction );
      v_min_prediction = move( v_prediction );
      min_prediction_mode = ( mbmode )prediction_mode;
      min_energy = variance_val;
    }
  }

  // Apply
  frame_mb.U().at( 0, 0 ).set_prediction_mode( min_prediction_mode );

  frame_mb.U().forall_ij(
    [&] ( UVBlock & frame_sb, unsigned int sb_column, unsigned int sb_row )
    {
      auto & original_sb = original_mb.U_sub.at( sb_column, sb_row );
      frame_sb.mutable_coefficients().subtract_dct( original_sb,
        TwoDSubRange<uint8_t, 4, 4>( u_min_prediction, 4 * sb_column, 4 * sb_row ) );
      frame_sb.mutable_coefficients() = UVBlock::quantize( quantizer, frame_sb.coefficients() );
      frame_sb.calculate_has_nonzero();
    } );

  frame_mb.V().forall_ij(
    [&] ( UVBlock & frame_sb, unsigned int sb_column, unsigned int sb_row )
    {
      auto & original_sb = original_mb.V_sub.at( sb_column, sb_row );
      frame_sb.mutable_coefficients().subtract_dct( original_sb,
        TwoDSubRange<uint8_t, 4, 4>( v_min_prediction, 4 * sb_column, 4 * sb_row ) );
      frame_sb.mutable_coefficients() = UVBlock::quantize( quantizer, frame_sb.coefficients() );
      frame_sb.calculate_has_nonzero();
    } );
}

pair<bmode, TwoD<uint8_t>> Encoder::luma_sb_intra_predict( const VP8Raster::Block4 & original_sb,
                                                           VP8Raster::Block4 & reconstructed_sb,
                                                           YBlock & /* frame_sb */,
                                                           const Quantizer & quantizer ) const
{
  uint32_t min_energy = numeric_limits<uint32_t>::max();
  bmode min_prediction_mode = B_DC_PRED;
  TwoD<uint8_t> min_prediction( 4, 4 );

  for ( unsigned int prediction_mode = 0; prediction_mode < num_intra_b_modes; prediction_mode++ ) {
    TwoD<uint8_t> prediction( 4, 4 );

    reconstructed_sb.intra_predict( ( bmode )prediction_mode, prediction );
    uint32_t variance_val = variance( original_sb, prediction, quantizer.y_dc, quantizer.y_ac );

    if ( variance_val < min_energy ) {
      min_prediction = move( prediction );
      min_prediction_mode = ( bmode )prediction_mode;
      min_energy = variance_val;
    }
  }

  return make_pair( min_prediction_mode, move( min_prediction ) );
}

template<>
pair<KeyFrame, double> Encoder::encode_with_quantizer<KeyFrame>( const VP8Raster & raster,
                                                                 const vector<QuantIndices> & quant_indices,
                                                                 const DecoderState & decoder_state,
                                                                 const double minimum_ssim ) const
{
  const uint16_t width = raster.display_width();
  const uint16_t height = raster.display_height();

  KeyFrame frame = Encoder::make_empty_frame( width, height );
  frame.mutable_header().quant_indices = quant_indices[ 0 ];

  vector<Quantizer> quantizers;

  for ( auto const & q : quant_indices ) {
    quantizers.emplace_back( q );
  }

  MutableRasterHandle reconstructed_raster_handle { width, height };
  VP8Raster & reconstructed_raster = reconstructed_raster_handle.get();

  raster.macroblocks().forall_ij(
    [&] ( VP8Raster::Macroblock & original_mb, unsigned int mb_column, unsigned int mb_row )
    {
      auto & reconstructued_mb = reconstructed_raster.macroblock( mb_column, mb_row );
      auto & frame_mb = frame.mutable_macroblocks().at( mb_column, mb_row );

      size_t segment_id = luma_mb_intra_predict( original_mb, reconstructued_mb, frame_mb, quantizers, 0, minimum_ssim );
      chroma_mb_intra_predict( original_mb, reconstructued_mb, frame_mb, quantizers[ segment_id ] );

      frame.relink_y2_blocks();
      frame_mb.calculate_has_nonzero();
      frame_mb.reconstruct_intra( quantizers[ segment_id ], reconstructued_mb );
    } );

  /* fixing segmentation headers in frame */
  frame.mutable_header().update_segmentation.clear();
  frame.mutable_header().update_segmentation.initialize();
  frame.mutable_header().update_segmentation.get().update_mb_segmentation_map = true;
  frame.mutable_header().update_segmentation.get().segment_feature_data.initialize();

  Array<Flagged<Unsigned<8>>, 3> mb_segmentation_map;

  /* don't optimize the tree probabilities for now */
  for ( unsigned int i = 0; i < 3; i++ ) {
    mb_segmentation_map.at( i ).initialize( 128 );
  }

  frame.mutable_header().update_segmentation.get().mb_segmentation_map.initialize( mb_segmentation_map );

  for ( size_t i = 0; i < 4; i++ ) {
    auto & quantizer_update = frame.mutable_header().update_segmentation.get().segment_feature_data.get().quantizer_update.at( i );
    quantizer_update.initialize( quant_indices[ i ].y_ac_qi - quant_indices[ 0 ].y_ac_qi );
  }

  frame.relink_y2_blocks();
  frame.loopfilter( decoder_state.segmentation, decoder_state.filter_adjustments, reconstructed_raster );
  return make_pair( move( frame ), reconstructed_raster.quality( raster ) );
}

double Encoder::encode_as_keyframe( const VP8Raster & raster, const double minimum_ssim )
{
  const uint16_t width = raster.display_width();
  const uint16_t height = raster.display_height();

  if ( width != width_ or height != height_ ) {
    throw runtime_error( "scaling is not supported." );
  }

  DecoderState temp_state { width, height };
  vector<QuantIndices> quant_indices;
  quant_indices.resize( 4 );

  for ( size_t i = 0; i < 4; i++ ) {
    switch( i ) {
    case 0: quant_indices[ i ].y_ac_qi = 96; break;
    case 1: quant_indices[ i ].y_ac_qi = 3; break;
    case 2: quant_indices[ i ].y_ac_qi = 2;  break;
    case 3: quant_indices[ i ].y_ac_qi = 0;  break;
    }

    quant_indices[ i ].y_dc  = Signed<4>( 0 );
    quant_indices[ i ].uv_dc = Signed<4>( 15 );
    quant_indices[ i ].uv_ac = Signed<4>( 15 );
  }

  pair<KeyFrame, double> encoded_frame = make_pair( move( make_empty_frame( width, height ) ), 0 );

  encoded_frame = encode_with_quantizer<KeyFrame>( raster, quant_indices, temp_state, minimum_ssim );
  ivf_writer_.append_frame( encoded_frame.first.serialize( temp_state.probability_tables ) );

  return encoded_frame.second;
}
