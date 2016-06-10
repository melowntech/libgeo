/*
 * gdalwarpoperation.cpp
 *
 * Everything here copied verbatim from GDAL (2.0.2), see header for explanation.
 */

#include "gdalwarpoperation.hpp"

#include <gdal_priv.h>

namespace geo { namespace detail {

namespace {

static const int anGWKFilterRadius[] =
{
    0,      // Nearest neighbour
    1,      // Bilinear
    2,      // Cubic Convolution (Catmull-Rom)
    2,      // Cubic B-Spline
    3,      // Lanczos windowed sinc
    0,      // Average
    0,      // Mode
    0,      // Reserved GRA_Gauss=7
    0,      // Max
    0,      // Min
    0,      // Med
    0,      // Q1
    0,      // Q3
};                                                    

int GWKGetFilterRadius(GDALResampleAlg eResampleAlg)
{
    return anGWKFilterRadius[eResampleAlg];
} 
    

}

GDALWarpOperation::GDALWarpOperation()

{
    psOptions = NULL;

    hIOMutex = NULL;
    hWarpMutex = NULL;

    nChunkListCount = 0;
    nChunkListMax = 0;
    pasChunkList = NULL;

    bReportTimings = FALSE;
    nLastTimeReported = 0;
}
 
GDALWarpOperation::~GDALWarpOperation()

{
    WipeOptions();

    if( hIOMutex != NULL )
    {
        CPLDestroyMutex( hIOMutex );
        CPLDestroyMutex( hWarpMutex );
    }

    WipeChunkList();
}

CPLErr GDALWarpOperation::CollectChunkList( 
    int nDstXOff, int nDstYOff,  int nDstXSize, int nDstYSize )

{
/* -------------------------------------------------------------------- */
/*      Compute the bounds of the input area corresponding to the       */
/*      output area.                                                    */
/* -------------------------------------------------------------------- */
    int nSrcXOff, nSrcYOff, nSrcXSize, nSrcYSize;
    int nSrcXExtraSize, nSrcYExtraSize;
    double dfSrcFillRatio;
    CPLErr eErr;

    eErr = ComputeSourceWindow( nDstXOff, nDstYOff, nDstXSize, nDstYSize,
                                &nSrcXOff, &nSrcYOff, &nSrcXSize, &nSrcYSize,
                                &nSrcXExtraSize, &nSrcYExtraSize, &dfSrcFillRatio );
    
    if( eErr != CE_None )
    {
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Unable to compute source region for output window %d,%d,%d,%d, skipping.", 
                  nDstXOff, nDstYOff, nDstXSize, nDstYSize );
        return eErr;
    }

/* -------------------------------------------------------------------- */
/*      If we are allowed to drop no-source regons, do so now if       */
/*      appropriate.                                                    */
/* -------------------------------------------------------------------- */
    if( (nSrcXSize == 0 || nSrcYSize == 0)
        && CSLFetchBoolean( psOptions->papszWarpOptions, "SKIP_NOSOURCE",0 ))
        return CE_None;

/* -------------------------------------------------------------------- */
/*      Based on the types of masks in use, how many bits will each     */
/*      source pixel cost us?                                           */
/* -------------------------------------------------------------------- */
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

/* -------------------------------------------------------------------- */
/*      What about the cost for the destination.                        */
/* -------------------------------------------------------------------- */
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

/* -------------------------------------------------------------------- */
/*      Does the cost of the current rectangle exceed our memory        */
/*      limit? If so, split the destination along the longest           */
/*      dimension and recurse.                                          */
/* -------------------------------------------------------------------- */
    double dfTotalMemoryUse;

    dfTotalMemoryUse =
        (((double) nSrcPixelCostInBits) * nSrcXSize * nSrcYSize
         + ((double) nDstPixelCostInBits) * nDstXSize * nDstYSize) / 8.0;
         
        
    int nBlockXSize = 1, nBlockYSize = 1;
    if (psOptions->hDstDS)
    {
        GDALGetBlockSize(GDALGetRasterBand(psOptions->hDstDS, 1),
                         &nBlockXSize, &nBlockYSize);
    }
    
    // If size of working buffers need exceed the allow limit, then divide
    // the target area
    // Do it also if the "fill ratio" of the source is too low (#3120), but
    // only if there's at least some source pixel intersecting. The
    // SRC_FILL_RATIO_HEURISTICS warping option is undocumented and only here
    // in case the heuristics would cause issues.
    /*CPLDebug("WARP", "dst=(%d,%d,%d,%d) src=(%d,%d,%d,%d) srcfillratio=%.18g",
             nDstXOff, nDstYOff, nDstXSize, nDstYSize,
             nSrcXOff, nSrcYOff, nSrcXSize, nSrcYSize, dfSrcFillRatio);*/
    if( (dfTotalMemoryUse > psOptions->dfWarpMemoryLimit && (nDstXSize > 2 || nDstYSize > 2)) ||
        (dfSrcFillRatio > 0 && dfSrcFillRatio < 0.5 && (nDstXSize > 100 || nDstYSize > 100) &&
         CSLFetchBoolean( psOptions->papszWarpOptions, "SRC_FILL_RATIO_HEURISTICS", TRUE )) )
    {
        CPLErr eErr2 = CE_None;
        
        int bStreamableOutput =
                CSLFetchBoolean( psOptions->papszWarpOptions, "STREAMABLE_OUTPUT", FALSE );
        int bOptimizeSize = !bStreamableOutput &&
                CSLFetchBoolean( psOptions->papszWarpOptions, "OPTIMIZE_SIZE", FALSE );

        /* If the region width is greater than the region height, */
        /* cut in half in the width. When we want to optimize the size */
        /* of a compressed output dataset, do this only if each half part */
        /* is at least as wide as the block width */
        int bHasDivided = FALSE;
        if( nDstXSize > nDstYSize &&
            ((!bOptimizeSize && !bStreamableOutput) ||
             (bOptimizeSize && (nDstXSize / 2 >= nBlockXSize || nDstYSize == 1)) ||
             (bStreamableOutput && nDstXSize / 2 >= nBlockXSize && nDstYSize == nBlockYSize)) )
        {
            bHasDivided = TRUE;
            int nChunk1 = nDstXSize / 2;
            
            /* In the optimize size case, try to stick on target block boundaries */
            if ((bOptimizeSize || bStreamableOutput) && nChunk1 > nBlockXSize)
                nChunk1 = (nChunk1 / nBlockXSize) * nBlockXSize;
            
            int nChunk2 = nDstXSize - nChunk1;

            eErr = CollectChunkList( nDstXOff, nDstYOff, 
                                     nChunk1, nDstYSize );

            eErr2 = CollectChunkList( nDstXOff+nChunk1, nDstYOff, 
                                      nChunk2, nDstYSize );
        }
        else if( !(bStreamableOutput && nDstYSize / 2 < nBlockYSize) )
        {
            bHasDivided = TRUE;
            int nChunk1 = nDstYSize / 2;

            /* In the optimize size case, try to stick on target block boundaries */
            if ((bOptimizeSize || bStreamableOutput) && nChunk1 > nBlockYSize)
                nChunk1 = (nChunk1 / nBlockYSize) * nBlockYSize;

            int nChunk2 = nDstYSize - nChunk1;

            eErr = CollectChunkList( nDstXOff, nDstYOff, 
                                     nDstXSize, nChunk1 );

            eErr2 = CollectChunkList( nDstXOff, nDstYOff+nChunk1, 
                                      nDstXSize, nChunk2 );
        }

        if( bHasDivided )
        {
            if( eErr == CE_None )
                return eErr2;
            else
                return eErr;
        }
    }

/* -------------------------------------------------------------------- */
/*      OK, everything fits, so add to the chunk list.                  */
/* -------------------------------------------------------------------- */
    if( nChunkListCount == nChunkListMax )
    {
        nChunkListMax = nChunkListMax * 2 + 1;
        pasChunkList = (GDALWarpChunk *) 
            CPLRealloc(pasChunkList,sizeof(GDALWarpChunk)*nChunkListMax );
    }

    pasChunkList[nChunkListCount].dx = nDstXOff;
    pasChunkList[nChunkListCount].dy = nDstYOff;
    pasChunkList[nChunkListCount].dsx = nDstXSize;
    pasChunkList[nChunkListCount].dsy = nDstYSize;
    pasChunkList[nChunkListCount].sx = nSrcXOff;
    pasChunkList[nChunkListCount].sy = nSrcYOff;
    pasChunkList[nChunkListCount].ssx = nSrcXSize;
    pasChunkList[nChunkListCount].ssy = nSrcYSize;
    pasChunkList[nChunkListCount].sExtraSx = nSrcXExtraSize;
    pasChunkList[nChunkListCount].sExtraSy = nSrcYExtraSize;

    nChunkListCount++;

    return CE_None;
}

CPLErr GDALWarpOperation::Initialize( const GDALWarpOptions *psNewOptions )

{
    CPLErr eErr = CE_None;

/* -------------------------------------------------------------------- */
/*      Copy the passed in options.                                     */
/* -------------------------------------------------------------------- */
    if( psOptions != NULL )
        WipeOptions();

    psOptions = GDALCloneWarpOptions( psNewOptions );
    psOptions->papszWarpOptions = CSLSetNameValue(psOptions->papszWarpOptions,
        "EXTRA_ELTS", CPLSPrintf("%d", WARP_EXTRA_ELTS));

/* -------------------------------------------------------------------- */
/*      Default band mapping if missing.                                */
/* -------------------------------------------------------------------- */
    if( psOptions->nBandCount == 0 
        && psOptions->hSrcDS != NULL
        && psOptions->hDstDS != NULL 
        && GDALGetRasterCount( psOptions->hSrcDS ) 
        == GDALGetRasterCount( psOptions->hDstDS ) )
    {
        int  i;

        psOptions->nBandCount = GDALGetRasterCount( psOptions->hSrcDS );

        psOptions->panSrcBands = (int *) 
            CPLMalloc(sizeof(int) * psOptions->nBandCount );
        psOptions->panDstBands = (int *) 
            CPLMalloc(sizeof(int) * psOptions->nBandCount );

        for( i = 0; i < psOptions->nBandCount; i++ )
        {
            psOptions->panSrcBands[i] = i+1;
            psOptions->panDstBands[i] = i+1;
        }
    }

/* -------------------------------------------------------------------- */
/*      If no working data type was provided, set one now.              */
/*                                                                      */
/*      Default to the highest resolution output band.  But if the      */
/*      input band is higher resolution and has a nodata value "out     */
/*      of band" with the output type we may need to use the higher     */
/*      resolution input type to ensure we can identify nodata values.  */
/* -------------------------------------------------------------------- */
    if( psOptions->eWorkingDataType == GDT_Unknown 
        && psOptions->hSrcDS != NULL 
        && psOptions->hDstDS != NULL 
        && psOptions->nBandCount >= 1 )
    {
        int iBand;
        psOptions->eWorkingDataType = GDT_Byte;

        for( iBand = 0; iBand < psOptions->nBandCount; iBand++ )
        {
            GDALRasterBandH hDstBand = GDALGetRasterBand( 
                psOptions->hDstDS, psOptions->panDstBands[iBand] );
            GDALRasterBandH hSrcBand = GDALGetRasterBand( 
                psOptions->hSrcDS, psOptions->panSrcBands[iBand] );
                                                  
            if( hDstBand != NULL )
                psOptions->eWorkingDataType = 
                    GDALDataTypeUnion( psOptions->eWorkingDataType, 
                                       GDALGetRasterDataType( hDstBand ) );
            
            if( hSrcBand != NULL 
                && psOptions->padfSrcNoDataReal != NULL )
            {
                int bMergeSource = FALSE;

                if( psOptions->padfSrcNoDataImag != NULL
                    && psOptions->padfSrcNoDataImag[iBand] != 0.0
                    && !GDALDataTypeIsComplex( psOptions->eWorkingDataType ) )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] < 0.0 
                         && (psOptions->eWorkingDataType == GDT_Byte
                             || psOptions->eWorkingDataType == GDT_UInt16
                             || psOptions->eWorkingDataType == GDT_UInt32) )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] < -32768.0 
                         && psOptions->eWorkingDataType == GDT_Int16 )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] < -2147483648.0 
                         && psOptions->eWorkingDataType == GDT_Int32 )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] > 256 
                         && psOptions->eWorkingDataType == GDT_Byte )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] > 32767
                         && psOptions->eWorkingDataType == GDT_Int16 )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] > 65535
                         && psOptions->eWorkingDataType == GDT_UInt16 )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] > 2147483648.0 
                         && psOptions->eWorkingDataType == GDT_Int32 )
                    bMergeSource = TRUE;
                else if( psOptions->padfSrcNoDataReal[iBand] > 4294967295.0
                         && psOptions->eWorkingDataType == GDT_UInt32 )
                    bMergeSource = TRUE;

                if( bMergeSource )
                    psOptions->eWorkingDataType = 
                        GDALDataTypeUnion( psOptions->eWorkingDataType, 
                                           GDALGetRasterDataType( hSrcBand ) );
            }
        }
    }

/* -------------------------------------------------------------------- */
/*      Default memory available.                                       */
/*                                                                      */
/*      For now we default to 64MB of RAM, but eventually we should     */
/*      try various schemes to query physical RAM.  This can            */
/*      certainly be done on Win32 and Linux.                           */
/* -------------------------------------------------------------------- */
    if( psOptions->dfWarpMemoryLimit == 0.0 )
    {
        psOptions->dfWarpMemoryLimit = 64.0 * 1024*1024;
    }

/* -------------------------------------------------------------------- */
/*      Are we doing timings?                                           */
/* -------------------------------------------------------------------- */
    bReportTimings = CSLFetchBoolean( psOptions->papszWarpOptions, 
                                      "REPORT_TIMINGS", FALSE );

/* -------------------------------------------------------------------- */
/*      Support creating cutline from text warpoption.                  */
/* -------------------------------------------------------------------- */
    const char *pszCutlineWKT = 
        CSLFetchNameValue( psOptions->papszWarpOptions, "CUTLINE" );
        
    if( pszCutlineWKT )
    {
        if( OGR_G_CreateFromWkt( (char **) &pszCutlineWKT, NULL, 
                                 (OGRGeometryH *) &(psOptions->hCutline) )
            != OGRERR_NONE )
        {
            eErr = CE_Failure;
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Failed to parse CUTLINE geometry wkt." );
        }
        else
        {
            const char *pszBD = CSLFetchNameValue( psOptions->papszWarpOptions,
                                                   "CUTLINE_BLEND_DIST" );
            if( pszBD )
                psOptions->dfCutlineBlendDist = CPLAtof(pszBD);
        }
    }

/* -------------------------------------------------------------------- */
/*      If the options don't validate, then wipe them.                  */
/* -------------------------------------------------------------------- */
    if( !ValidateOptions() )
        eErr = CE_Failure;

    if( eErr != CE_None )
        WipeOptions();

    return eErr;
}

void GDALWarpOperation::WipeChunkList()

{
    CPLFree( pasChunkList );
    pasChunkList = NULL;
    nChunkListCount = 0;
    nChunkListMax = 0;
}

CPLErr GDALWarpOperation::ComputeSourceWindow(int nDstXOff, int nDstYOff, 
                                              int nDstXSize, int nDstYSize,
                                              int *pnSrcXOff, int *pnSrcYOff, 
                                              int *pnSrcXSize, int *pnSrcYSize,
                                              int *pnSrcXExtraSize, int *pnSrcYExtraSize,
                                              double *pdfSrcFillRatio)

{
/* -------------------------------------------------------------------- */
/*      Figure out whether we just want to do the usual "along the      */
/*      edge" sampling, or using a grid.  The grid usage is             */
/*      important in some weird "inside out" cases like WGS84 to        */
/*      polar stereographic around the pole.   Also figure out the      */
/*      sampling rate.                                                  */
/* -------------------------------------------------------------------- */
    double dfStepSize;
    int nSampleMax, nStepCount = 21, bUseGrid;
    int *pabSuccess = NULL;
    double *padfX, *padfY, *padfZ;
    int    nSamplePoints;
    double dfRatio;

    if( CSLFetchNameValue( psOptions->papszWarpOptions, 
                           "SAMPLE_STEPS" ) != NULL )
    {
        nStepCount = 
            atoi(CSLFetchNameValue( psOptions->papszWarpOptions, 
                                    "SAMPLE_STEPS" ));
        nStepCount = MAX(2,nStepCount);
    }

    dfStepSize = 1.0 / (nStepCount-1);

    bUseGrid = CSLFetchBoolean( psOptions->papszWarpOptions, 
                                "SAMPLE_GRID", FALSE );

  TryAgainWithGrid:
    nSamplePoints = 0;
    if( bUseGrid )
    {
        if (nStepCount > INT_MAX / nStepCount)
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Too many steps : %d", nStepCount);
            return CE_Failure;
        }
        nSampleMax = nStepCount * nStepCount;
    }
    else
    {
        if (nStepCount > INT_MAX / 4)
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Too many steps : %d", nStepCount);
            return CE_Failure;
        }
        nSampleMax = nStepCount * 4;
    }

    pabSuccess = (int *) VSIMalloc2(sizeof(int), nSampleMax);
    padfX = (double *) VSIMalloc2(sizeof(double) * 3, nSampleMax);
    if (pabSuccess == NULL || padfX == NULL)
    {
        CPLFree( padfX );
        CPLFree( pabSuccess );
        return CE_Failure;
    }
    padfY = padfX + nSampleMax;
    padfZ = padfX + nSampleMax * 2;

/* -------------------------------------------------------------------- */
/*      Setup sample points on a grid pattern throughout the area.      */
/* -------------------------------------------------------------------- */
    if( bUseGrid )
    {
        double dfRatioY;

        for( dfRatioY = 0.0; 
             dfRatioY <= 1.0 + dfStepSize*0.5; 
             dfRatioY += dfStepSize )
        {
            for( dfRatio = 0.0; 
                 dfRatio <= 1.0 + dfStepSize*0.5; 
                 dfRatio += dfStepSize )
            {
                padfX[nSamplePoints]   = dfRatio * nDstXSize + nDstXOff;
                padfY[nSamplePoints]   = dfRatioY * nDstYSize + nDstYOff;
                padfZ[nSamplePoints++] = 0.0;
            }
        }
    }
 /* -------------------------------------------------------------------- */
 /*      Setup sample points all around the edge of the output raster.   */
 /* -------------------------------------------------------------------- */
    else
    {
        for( dfRatio = 0.0; dfRatio <= 1.0 + dfStepSize*0.5; dfRatio += dfStepSize )
        {
            // Along top 
            padfX[nSamplePoints]   = dfRatio * nDstXSize + nDstXOff;
            padfY[nSamplePoints]   = nDstYOff;
            padfZ[nSamplePoints++] = 0.0;
            
            // Along bottom 
            padfX[nSamplePoints]   = dfRatio * nDstXSize + nDstXOff;
            padfY[nSamplePoints]   = nDstYOff + nDstYSize;
            padfZ[nSamplePoints++] = 0.0;
            
            // Along left
            padfX[nSamplePoints]   = nDstXOff;
            padfY[nSamplePoints]   = dfRatio * nDstYSize + nDstYOff;
            padfZ[nSamplePoints++] = 0.0;
            
            // Along right
            padfX[nSamplePoints]   = nDstXSize + nDstXOff;
            padfY[nSamplePoints]   = dfRatio * nDstYSize + nDstYOff;
            padfZ[nSamplePoints++] = 0.0;
        }
    }
        
    CPLAssert( nSamplePoints == nSampleMax );

/* -------------------------------------------------------------------- */
/*      Transform them to the input pixel coordinate space              */
/* -------------------------------------------------------------------- */
    if( !psOptions->pfnTransformer( psOptions->pTransformerArg, 
                                    TRUE, nSamplePoints, 
                                    padfX, padfY, padfZ, pabSuccess ) )
    {
        CPLFree( padfX );
        CPLFree( pabSuccess );

        CPLError( CE_Failure, CPLE_AppDefined, 
                  "GDALWarperOperation::ComputeSourceWindow() failed because\n"
                  "the pfnTransformer failed." );
        return CE_Failure;
    }
        
/* -------------------------------------------------------------------- */
/*      Collect the bounds, ignoring any failed points.                 */
/* -------------------------------------------------------------------- */
    double dfMinXOut=0.0, dfMinYOut=0.0, dfMaxXOut=0.0, dfMaxYOut=0.0;
    int    bGotInitialPoint = FALSE;
    int    nFailedCount = 0, i;

    for( i = 0; i < nSamplePoints; i++ )
    {
        if( !pabSuccess[i] )
        {
            nFailedCount++;
            continue;
        }

        if( !bGotInitialPoint )
        {
            bGotInitialPoint = TRUE;
            dfMinXOut = dfMaxXOut = padfX[i];
            dfMinYOut = dfMaxYOut = padfY[i];
        }
        else
        {
            dfMinXOut = MIN(dfMinXOut,padfX[i]);
            dfMinYOut = MIN(dfMinYOut,padfY[i]);
            dfMaxXOut = MAX(dfMaxXOut,padfX[i]);
            dfMaxYOut = MAX(dfMaxYOut,padfY[i]);
        }
    }

    CPLFree( padfX );
    CPLFree( pabSuccess );

/* -------------------------------------------------------------------- */
/*      If we got any failures when not using a grid, we should         */
/*      really go back and try again with the grid.  Sorry for the      */
/*      goto.                                                           */
/* -------------------------------------------------------------------- */
    if( !bUseGrid && nFailedCount > 0 )
    {
        bUseGrid = TRUE;
        goto TryAgainWithGrid;
    }

/* -------------------------------------------------------------------- */
/*      If we get hardly any points (or none) transforming, we give     */
/*      up.                                                             */
/* -------------------------------------------------------------------- */
    if( nFailedCount > nSamplePoints - 5 )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Too many points (%d out of %d) failed to transform,\n"
                  "unable to compute output bounds.",
                  nFailedCount, nSamplePoints );
        return CE_Failure;
    }

    if( nFailedCount > 0 )
        CPLDebug( "GDAL", 
                  "GDALWarpOperation::ComputeSourceWindow() %d out of %d points failed to transform.", 
                  nFailedCount, nSamplePoints );

/* -------------------------------------------------------------------- */
/*   Early exit to avoid crazy values to cause a huge nResWinSize that  */
/*   would result in a result window wrongly covering the whole raster. */
/* -------------------------------------------------------------------- */
    const int nRasterXSize = GDALGetRasterXSize(psOptions->hSrcDS);
    const int nRasterYSize = GDALGetRasterYSize(psOptions->hSrcDS);
    if( dfMinXOut > nRasterXSize ||
        dfMaxXOut < 0 ||
        dfMinYOut > nRasterYSize ||
        dfMaxYOut < 0 )
    {
        *pnSrcXOff = 0;
        *pnSrcYOff = 0;
        *pnSrcXSize = 0;
        *pnSrcYSize = 0;
        if( pnSrcXExtraSize )
            *pnSrcXExtraSize = 0;
        if( pnSrcYExtraSize )
            *pnSrcYExtraSize = 0;
        if( pdfSrcFillRatio )
            *pdfSrcFillRatio = 0;
        return CE_None;
    }

/* -------------------------------------------------------------------- */
/*      How much of a window around our source pixel might we need      */
/*      to collect data from based on the resampling kernel?  Even      */
/*      if the requested central pixel falls off the source image,      */
/*      we may need to collect data if some portion of the              */
/*      resampling kernel could be on-image.                            */
/* -------------------------------------------------------------------- */
    int nResWinSize = geo::detail::GWKGetFilterRadius(psOptions->eResampleAlg);

    /* Take scaling into account */
    double dfXScale = (double)nDstXSize / (dfMaxXOut - dfMinXOut);
    double dfYScale = (double)nDstYSize / (dfMaxYOut - dfMinYOut);
    int nXRadius = ( dfXScale < 1.0 ) ?
        (int)ceil( nResWinSize / dfXScale ) :nResWinSize;
    int nYRadius = ( dfYScale < 1.0 ) ?
        (int)ceil( nResWinSize / dfYScale ) : nResWinSize;
    nResWinSize = MAX(nXRadius, nYRadius);

/* -------------------------------------------------------------------- */
/*      Allow addition of extra sample pixels to source window to       */
/*      avoid missing pixels due to sampling error.  In fact,           */
/*      fallback to adding a bit to the window if any points failed     */
/*      to transform.                                                   */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( psOptions->papszWarpOptions, 
                           "SOURCE_EXTRA" ) != NULL )
    {
        nResWinSize += atoi(
            CSLFetchNameValue( psOptions->papszWarpOptions, "SOURCE_EXTRA" ));
    }
    else if( nFailedCount > 0 )
        nResWinSize += 10;

/* -------------------------------------------------------------------- */
/*      return bounds.                                                  */
/* -------------------------------------------------------------------- */
    /*CPLDebug("WARP", "dst=(%d,%d,%d,%d) raw src=(minx=%.8g,miny=%.8g,maxx=%.8g,maxy=%.8g)",
             nDstXOff, nDstYOff, nDstXSize, nDstYSize,
             dfMinXOut, dfMinYOut, dfMaxXOut, dfMaxYOut);
    */
    *pnSrcXOff = MAX(0,(int) floor( dfMinXOut ) );
    *pnSrcYOff = MAX(0,(int) floor( dfMinYOut ) );
    *pnSrcXOff = MIN(*pnSrcXOff,nRasterXSize);
    *pnSrcYOff = MIN(*pnSrcYOff,nRasterYSize);

    double dfCeilMaxXOut = ceil(dfMaxXOut);
    if( dfCeilMaxXOut > INT_MAX )
        dfCeilMaxXOut = INT_MAX;
    double dfCeilMaxYOut = ceil(dfMaxYOut);
    if( dfCeilMaxYOut > INT_MAX )
        dfCeilMaxYOut = INT_MAX;

    int nSrcXSizeRaw = MIN( nRasterXSize - *pnSrcXOff,
                       ((int) dfCeilMaxXOut) - *pnSrcXOff );
    int nSrcYSizeRaw = MIN( nRasterYSize - *pnSrcYOff,
                       ((int) dfCeilMaxYOut) - *pnSrcYOff );
    nSrcXSizeRaw = MAX(0,nSrcXSizeRaw);
    nSrcYSizeRaw = MAX(0,nSrcYSizeRaw);
    
    *pnSrcXOff = MAX(0,(int) floor( dfMinXOut ) - nResWinSize );
    *pnSrcYOff = MAX(0,(int) floor( dfMinYOut ) - nResWinSize );
    *pnSrcXOff = MIN(*pnSrcXOff,nRasterXSize);
    *pnSrcYOff = MIN(*pnSrcYOff,nRasterYSize);

    *pnSrcXSize = MIN( nRasterXSize - *pnSrcXOff,
                       ((int) dfCeilMaxXOut) - *pnSrcXOff + nResWinSize );
    *pnSrcYSize = MIN( nRasterYSize - *pnSrcYOff,
                       ((int) dfCeilMaxYOut) - *pnSrcYOff + nResWinSize );
    *pnSrcXSize = MAX(0,*pnSrcXSize);
    *pnSrcYSize = MAX(0,*pnSrcYSize);

    if( pnSrcXExtraSize )
        *pnSrcXExtraSize = *pnSrcXSize - nSrcXSizeRaw;
    if( pnSrcYExtraSize )
        *pnSrcYExtraSize = *pnSrcYSize - nSrcYSizeRaw;
    
    // Computed the ratio of the clamped source raster window size over
    // the unclamped source raster window size
    if( pdfSrcFillRatio )
        *pdfSrcFillRatio = *pnSrcXSize * *pnSrcYSize / MAX(1.0,
        (dfMaxXOut - dfMinXOut + 2 * nResWinSize) * (dfMaxYOut - dfMinYOut + 2 * nResWinSize)); 

    return CE_None;
}

void GDALWarpOperation::WipeOptions()

{
    if( psOptions != NULL )
    {
        GDALDestroyWarpOptions( psOptions );
        psOptions = NULL;
    }
}

int GDALWarpOperation::ValidateOptions()

{
    if( psOptions == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  no options currently initialized." );
        return FALSE;
    }

    if( psOptions->dfWarpMemoryLimit < 100000.0 )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  dfWarpMemoryLimit=%g is unreasonably small.",
                  psOptions->dfWarpMemoryLimit );
        return FALSE;
    }

    if( psOptions->eResampleAlg != GRA_NearestNeighbour 
        && psOptions->eResampleAlg != GRA_Bilinear
        && psOptions->eResampleAlg != GRA_Cubic
        && psOptions->eResampleAlg != GRA_CubicSpline
        && psOptions->eResampleAlg != GRA_Lanczos
        && psOptions->eResampleAlg != GRA_Average
        && psOptions->eResampleAlg != GRA_Mode
        && psOptions->eResampleAlg != GRA_Max 
        && psOptions->eResampleAlg != GRA_Min
        && psOptions->eResampleAlg != GRA_Med
        && psOptions->eResampleAlg != GRA_Q1
        && psOptions->eResampleAlg != GRA_Q3)
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  eResampleArg=%d is not a supported value.",
                  psOptions->eResampleAlg );
        return FALSE;
    }

    if( (int) psOptions->eWorkingDataType < 1
        && (int) psOptions->eWorkingDataType >= GDT_TypeCount )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  eWorkingDataType=%d is not a supported value.",
                  psOptions->eWorkingDataType );
        return FALSE;
    }

    if( psOptions->hSrcDS == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  hSrcDS is not set." );
        return FALSE;
    }

    if( psOptions->nBandCount == 0 )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  nBandCount=0, no bands configured!" );
        return FALSE;
    }

    if( psOptions->panSrcBands == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  panSrcBands is NULL." );
        return FALSE;
    }

    if( psOptions->hDstDS != NULL && psOptions->panDstBands == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  panDstBands is NULL." );
        return FALSE;
    }

    for( int iBand = 0; iBand < psOptions->nBandCount; iBand++ )
    {
        if( psOptions->panSrcBands[iBand] < 1 
            || psOptions->panSrcBands[iBand] 
            > GDALGetRasterCount( psOptions->hSrcDS ) )
        {
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "panSrcBands[%d] = %d ... out of range for dataset.",
                      iBand, psOptions->panSrcBands[iBand] );
            return FALSE;
        }
        if( psOptions->hDstDS != NULL
            && (psOptions->panDstBands[iBand] < 1 
                || psOptions->panDstBands[iBand]
                > GDALGetRasterCount( psOptions->hDstDS ) ) )
        {
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "panDstBands[%d] = %d ... out of range for dataset.",
                      iBand, psOptions->panDstBands[iBand] );
            return FALSE;
        }

        if( psOptions->hDstDS != NULL
            && GDALGetRasterAccess( 
                GDALGetRasterBand(psOptions->hDstDS,
                                  psOptions->panDstBands[iBand]))
            == GA_ReadOnly )
        {
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "Destination band %d appears to be read-only.",
                      psOptions->panDstBands[iBand] );
            return FALSE;
        }
    }

    if( psOptions->nBandCount == 0 )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  nBandCount=0, no bands configured!" );
        return FALSE;
    }

    if( psOptions->padfSrcNoDataReal != NULL
        && psOptions->padfSrcNoDataImag == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  padfSrcNoDataReal set, but padfSrcNoDataImag not set." );
        return FALSE;
    }

    if( psOptions->pfnProgress == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  pfnProgress is NULL." );
        return FALSE;
    }

    if( psOptions->pfnTransformer == NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
                  "GDALWarpOptions.Validate()\n"
                  "  pfnTransformer is NULL." );
        return FALSE;
    }

    if( CSLFetchNameValue( psOptions->papszWarpOptions, 
                           "SAMPLE_STEPS" ) != NULL )
    {
        if( atoi(CSLFetchNameValue( psOptions->papszWarpOptions, 
                                    "SAMPLE_STEPS" )) < 2 )
        {
            CPLError( CE_Failure, CPLE_IllegalArg, 
                      "GDALWarpOptions.Validate()\n"
                      "  SAMPLE_STEPS warp option has illegal value." );
            return FALSE;
        }
    }

    if( psOptions->nSrcAlphaBand > 0)
    {
        if ( psOptions->hSrcDS == NULL ||
             psOptions->nSrcAlphaBand > GDALGetRasterCount(psOptions->hSrcDS) )
        {
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "nSrcAlphaBand = %d ... out of range for dataset.",
                      psOptions->nSrcAlphaBand );
            return FALSE;
        }
    }

    if( psOptions->nDstAlphaBand > 0)
    {
        if ( psOptions->hDstDS == NULL ||
             psOptions->nDstAlphaBand > GDALGetRasterCount(psOptions->hDstDS) )
        {
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "nDstAlphaBand = %d ... out of range for dataset.",
                      psOptions->nDstAlphaBand );
            return FALSE;
        }
    }

    if( psOptions->nSrcAlphaBand > 0 
        && psOptions->pfnSrcDensityMaskFunc != NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
               "GDALWarpOptions.Validate()\n"
               "  pfnSrcDensityMaskFunc provided as well as a SrcAlphaBand." );
        return FALSE;
    }

    if( psOptions->nDstAlphaBand > 0 
        && psOptions->pfnDstDensityMaskFunc != NULL )
    {
        CPLError( CE_Failure, CPLE_IllegalArg, 
               "GDALWarpOptions.Validate()\n"
               "  pfnDstDensityMaskFunc provided as well as a DstAlphaBand." );
        return FALSE;
    }

    return TRUE;
}


} } // end namespace geo::detail                                                 
