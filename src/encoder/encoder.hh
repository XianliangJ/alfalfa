#ifndef ENCODER_HH
#define ENCODER_HH

#include <vector>
#include <string>
#include <tuple>

#include "vp8_raster.hh"
#include "frame.hh"
#include "ivf_writer.hh"

class Encoder
{
private:
  IVFWriter ivf_writer_;
  uint16_t width_;
  uint16_t height_;

  template<unsigned int size>
  static uint32_t variance( const VP8Raster::Block<size> & block,
                            const TwoD<uint8_t> & prediction,
                            const uint16_t dc_factor, const uint16_t ac_factor );

  template<unsigned int size>
  static uint32_t variance( const VP8Raster::Block<size> & block,
                            const TwoDSubRange<uint8_t, size, size> & prediction,
                            const uint16_t dc_factor, const uint16_t ac_factor );

  template <class MacroblockType>
  size_t luma_mb_intra_predict( const VP8Raster::Macroblock & original_mb,
                                VP8Raster::Macroblock & constructed_mb,
                                MacroblockType & frame_mb,
                                const std::vector<Quantizer> & quantizer,
                                size_t quantizer_index, double minimum_ssim ) const;

  template <class MacroblockType>
  void chroma_mb_intra_predict( const VP8Raster::Macroblock & original_mb,
                                VP8Raster::Macroblock & constructed_mb,
                                MacroblockType & frame_mb,
                                const Quantizer & quantizer ) const;

  std::pair<bmode, TwoD<uint8_t>> luma_sb_intra_predict( const VP8Raster::Block4 & original_sb,
                                                         VP8Raster::Block4 & constructed_sb,
                                                         YBlock & frame_sb,
                                                         const Quantizer & quantizer ) const;

  template<class FrameType>
  std::pair<KeyFrame, double> encode_with_quantizer( const VP8Raster & raster,
                                                     const std::vector<QuantIndices> & quant_indices,
                                                     const DecoderState & decoder_state,
                                                     const double minimum_ssim ) const;

public:
  Encoder( const std::string & output_filename, const uint16_t width, const uint16_t height );
  double encode_as_keyframe( const VP8Raster & raster, double minimum_ssim );

  static KeyFrame make_empty_frame( const uint16_t width, const uint16_t height );
};

#endif /* ENCODER_HH */
