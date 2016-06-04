#ifndef geo_detail_warpmemorymeter_hpp_included_
#define geo_detail_warpmemorymeter_hpp_included_

#include "gdalwarpoperation.hpp"

namespace geo { namespace detail {


class WarpMemoryMeter: private GDALWarpOperation {

public: 
    
    // initialize meter using complete warp options
    WarpMemoryMeter(const GDALWarpOptions *optionsIn);
        
    // return meter reading
    ulong measure() const { return measure_; };
        
private:
    ulong measure_;
};


} } // namespace geo::detail

#endif // geo_detail_srs_hpp_included_
