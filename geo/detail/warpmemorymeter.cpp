/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
        
        unsigned long chunkMeasure = 
            ( static_cast<unsigned long>(nSrcPixelCostInBits) * chunk.ssx * chunk.ssy +
            static_cast<unsigned long>(nDstPixelCostInBits) * chunk.dsx * chunk.dsy ) >> 3;

//        LOG(debug) << boost::format( "ssx: %d, ssy: %d, spixelcost: %d, "
//            "dsx: %d, dsy; %d, dpixelcost: %d" ) % chunk.ssx % chunk.ssy %
//            nSrcPixelCostInBits % chunk.dsx % chunk.dsy % nDstPixelCostInBits;
     
        measure_ = std::max( chunkMeasure, measure_ );              
    }
    
    // all done
}

} } // namespace geo::detail 
 
