/*
 * warpmemorymeter.cpp
 */

#include "warpmemorymeter.hpp"


#include <algorithm> 
#include <boost/format.hpp>
#include "dbglog/dbglog.hpp"

#include <gdal_priv.h>


 
namespace geo { namespace detail {

WarpMemoryMeter::WarpMemoryMeter( const GDALWarpOptions *optionsIn )
 : GDALWarpOperation(), measure_(0.0) {
 
    // initialize
    Initialize(optionsIn);
 
    // collect chunk List
    WipeChunkList();
    CollectChunkList(0, 0, 
        static_cast<GDALDataset *>(psOptions->hDstDS)->GetRasterXSize(), 
        static_cast<GDALDataset *>(psOptions->hDstDS)->GetRasterYSize());
     
    // determine pixel costs (taken from GdalWarpOperation::CollectChunkList)
    int nSrcPixelCostInBits;
    
    nSrcPixelCostInBits =
        GDALGetDataTypeSize( psOptions->eWorkingDataType )
        * psOptions->nBandCount;
                    
    if( psOptions->pfnSrcDensityMaskFunc != NULL )
        nSrcPixelCostInBits += 32; /* ?? float mask */
                                    
    GDALRasterBandH hSrcBand = NULL;
    if( psOptions->nBandCount > 0 )
        hSrcBand = GDALGetRasterBand(psOptions->hSrcDS,
                                     psOptions->panSrcBands[0]);
                                                                                         
    if( psOptions->nSrcAlphaBand > 0 || psOptions->hCutline != NULL )
       nSrcPixelCostInBits += 32; /* UnifiedSrcDensity float mask */
    else if (hSrcBand != NULL && (GDALGetMaskFlags(hSrcBand) & GMF_PER_DATASET))
       nSrcPixelCostInBits += 1; /* UnifiedSrcValid bit mask */
    if( psOptions->papfnSrcPerBandValidityMaskFunc != NULL
        || psOptions->padfSrcNoDataReal != NULL )
        nSrcPixelCostInBits += psOptions->nBandCount; /* bit/band mask */

    if( psOptions->pfnSrcValidityMaskFunc != NULL )
        nSrcPixelCostInBits += 1; /* bit mask */

    int nDstPixelCostInBits;
    
    nDstPixelCostInBits =
        GDALGetDataTypeSize( psOptions->eWorkingDataType ) 
        * psOptions->nBandCount;
                        
    if( psOptions->pfnDstDensityMaskFunc != NULL )
        nDstPixelCostInBits += 32;
                                    
    if( psOptions->padfDstNoDataReal != NULL
       || psOptions->pfnDstValidityMaskFunc != NULL )
       nDstPixelCostInBits += psOptions->nBandCount; 
                                                        
    if( psOptions->nDstAlphaBand > 0 )
       nDstPixelCostInBits += 32; /* DstDensity float mask */

    // iterate through chunks finding maximum memory requirements estimate
    for (int i=0; i < nChunkListCount; i++) {
 
        const auto & chunk( pasChunkList[i] );
        
        ulong chunkMeasure = 
            ( static_cast<ulong>(nSrcPixelCostInBits) * chunk.ssx * chunk.ssy +
            static_cast<ulong>(nDstPixelCostInBits) * chunk.dsx * chunk.dsy ) >> 3;

//        LOG(debug) << boost::format( "ssx: %d, ssy: %d, spixelcost: %d, "
//            "dsx: %d, dsy; %d, dpixelcost: %d" ) % chunk.ssx % chunk.ssy %
//            nSrcPixelCostInBits % chunk.dsx % chunk.dsy % nDstPixelCostInBits;
     
        measure_ = std::max( chunkMeasure, measure_ );              
    }
    
    // all done
}

} } // namespace geo::detail 
 
