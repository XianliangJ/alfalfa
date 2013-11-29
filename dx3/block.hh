#ifndef BLOCK_HH
#define BLOCK_HH

#include "modemv_data.hh"
#include "2d.hh"

template <class PredictionMode>
class Block
{
private:
  PredictionMode prediction_mode_ {};
  Optional< Block * > above_ {};
  Optional< Block * > left_ {};

public:
  Block() {}

  Block( typename TwoD< Block< PredictionMode > >::Context & context )
    : above_( context.above ), left_( context.left )
  {}

  const PredictionMode & prediction_mode( void ) const { return prediction_mode_; }
  void set_prediction_mode( const PredictionMode & prediction_mode ) { prediction_mode_ = prediction_mode; }

  const Optional< Block * > above( void ) const { return above_; }
  const Optional< Block * > left( void ) const { return left_; }
};

using Y2Block = Block< intra_mbmode >;
using UBlock = Block< intra_mbmode >;
using VBlock = Block< intra_mbmode >;
using YBlock = Block< intra_bmode >;

#endif /* BLOCK_HH */