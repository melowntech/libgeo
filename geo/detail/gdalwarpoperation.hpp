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
#ifndef geo_detail_gdalwarpoperation_hpp_included_
#define geo_detail_gdalwarpoperation_hpp_included_

/**
 * gdalwarpoperation.hpp
 *
 * Copied pretty much verbatim from gdal/gdalwarper.h (2.0.2). The sole reason behind this
 * is that we need access to GDALWarpOperation::CollectChunkList in WarpMemoryMeter.
 * Since this method is private in GDAL, I duplicate the code here. I did not want to,
 * but any other approach would result in UB (undefined behaviour).
 */

#include <gdalwarper.h>


namespace geo { namespace detail {

struct GDALWarpChunk {
    int dx, dy, dsx, dsy;
    int sx, sy, ssx, ssy;
    int sExtraSx, sExtraSy;
}; 

class GDALWarpOperation {
private:
    void            WipeOptions();
    int             ValidateOptions();

    CPLErr          ComputeSourceWindow( int nDstXOff, int nDstYOff, 
                                         int nDstXSize, int nDstYSize,
                                         int *pnSrcXOff, int *pnSrcYOff, 
                                         int *pnSrcXSize, int *pnSrcYSize,
                                         int *pnSrcXExtraSize, int *pnSrcYExtraSize,
                                         double* pdfSrcFillRatio );

    CPLErr          CreateKernelMask( GDALWarpKernel *, int iBand, 
                                      const char *pszType );

    CPLMutex        *hIOMutex;
    CPLMutex        *hWarpMutex;

    int             nChunkListMax;

    int             bReportTimings;
    unsigned long   nLastTimeReported;

    void            ReportTiming( const char * );
    
protected:    
    GDALWarpOptions *psOptions;
    void            WipeChunkList();
    CPLErr          CollectChunkList( int nDstXOff, int nDstYOff, 
                                      int nDstXSize, int nDstYSize );    
    int             nChunkListCount;
    GDALWarpChunk  *pasChunkList;
    
public:
                    GDALWarpOperation();
    virtual        ~GDALWarpOperation();

    CPLErr          Initialize( const GDALWarpOptions *psNewOptions );

    const GDALWarpOptions         *GetOptions();

    CPLErr          ChunkAndWarpImage( int nDstXOff, int nDstYOff, 
                                       int nDstXSize, int nDstYSize );
    CPLErr          ChunkAndWarpMulti( int nDstXOff, int nDstYOff, 
                                       int nDstXSize, int nDstYSize );
    CPLErr          WarpRegion( int nDstXOff, int nDstYOff, 
                                int nDstXSize, int nDstYSize,
                                int nSrcXOff=0, int nSrcYOff=0,
                                int nSrcXSize=0, int nSrcYSize=0,
                                double dfProgressBase=0.0, double dfProgressScale=1.0);
    CPLErr          WarpRegion( int nDstXOff, int nDstYOff, 
                                int nDstXSize, int nDstYSize,
                                int nSrcXOff, int nSrcYOff,
                                int nSrcXSize, int nSrcYSize,
                                int nSrcXExtraSize, int nSrcYExtraSize,
                                double dfProgressBase, double dfProgressScale);
    CPLErr          WarpRegionToBuffer( int nDstXOff, int nDstYOff, 
                                        int nDstXSize, int nDstYSize, 
                                        void *pDataBuf, 
                                        GDALDataType eBufDataType,
                                        int nSrcXOff=0, int nSrcYOff=0,
                                        int nSrcXSize=0, int nSrcYSize=0,
                                        double dfProgressBase=0.0, double dfProgressScale=1.0);
    CPLErr          WarpRegionToBuffer( int nDstXOff, int nDstYOff, 
                                        int nDstXSize, int nDstYSize, 
                                        void *pDataBuf, 
                                        GDALDataType eBufDataType,
                                        int nSrcXOff, int nSrcYOff,
                                        int nSrcXSize, int nSrcYSize,
                                        int nSrcXExtraSize, int nSrcYExtraSize,
                                        double dfProgressBase, double dfProgressScale);
};


} } // namespace geo::detail

#endif // geo_detail_srs_hpp_included_
