/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include "cpl_port.h"
#include "gdal_priv.h"

#include <cstring>

#include "cpl_conv.h"
#include "cpl_error.h"
#include "cpl_progress.h"
#include "cpl_string.h"
#include "gdal.h"
#include "gdal_mdreader.h"
#include "gdal_proxy.h"

#include "ovrdataset.hpp"

namespace geo { namespace detail {

/** Since GDALCreateOverviewDataset was removed from distribution library
 * interface we had include the functionality of this (now private) function in
 * our sources.
 *
 * Code in the following namespace gdal is reworked
 * gcore/gdaloverviewdataset.cpp from GDAL sources, orinally made by
 * Even Rouault, <even dot rouault at spatialys dot com>.
 *
 * Original file header:
 ******************************************************************************
 *
 * Project:  GDAL Core
 * Purpose:  Implementation of a dataset overview warping class
 * Author:   Even Rouault, <even dot rouault at spatialys dot com>
 *
 ******************************************************************************
 * Copyright (c) 2014, Even Rouault, <even dot rouault at spatialys dot com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************
 */
namespace gdal {

// fwd
class OverviewBand;

class OverviewDataset : public ::GDALDataset {
private:
    friend class OverviewBand;

    ::GDALDataset *main_;
    ::GDALDataset *ovr_;

    int level_;
    int thisLevelOnly_;

    int gcpCount_;
    ::GDAL_GCP *gcps_;
    char **papszMD_RPC;
    char **papszMD_GEOLOCATION;

    static void rescale(char**& papszMD, const char* pszItem
                        , double dfRatio, double dfDefaultVal);

protected:
    virtual CPLErr IRasterIO(::GDALRWFlag, int, int, int, int, void*, int, int
                             , ::GDALDataType, int, int*
                             , ::GSpacing, ::GSpacing, ::GSpacing
                             , ::GDALRasterIOExtraArg *psExtraArg)
        override;

public:
    OverviewDataset(::GDALDataset *main, int level, int thisLevelOnly);
    virtual ~OverviewDataset();

#if GDAL_VERSION_NUM >= 3000000
    const OGRSpatialReference* GetSpatialRef() const override {
        return main_->GetSpatialRef();
    }

    const OGRSpatialReference* GetGCPSpatialRef() const override {
        return main_->GetGCPSpatialRef();
    }
#else
    virtual const char* GetProjectionRef() override {
        return main_->GetProjectionRef();
    }

    virtual const char *GetGCPProjection() override {
        return main_->GetGCPProjection();
    }
#endif

    virtual CPLErr GetGeoTransform(double*) override;

    virtual int GetGCPCount() override {
        return main_->GetGCPCount();
    }

    virtual const ::GDAL_GCP* GetGCPs() override;

    virtual char** GetMetadata(const char *pszDomain = "") override;
    virtual const char* GetMetadataItem(const char *pszName
                                        , const char *pszDomain = "") override;

    virtual int CloseDependentDatasets() override;

private:
    CPL_DISALLOW_COPY_ASSIGN(OverviewDataset)
};

class OverviewBand : public GDALProxyRasterBand {
protected:
    friend class OverviewDataset;

    ::GDALRasterBand *underlyingBand_;
    virtual ::GDALRasterBand* RefUnderlyingRasterBand() override {
        return underlyingBand_;
    }

public:
    OverviewBand(OverviewDataset *ds, int band);
    virtual ~OverviewBand() { FlushCache(); }

#if GDAL_VERSION_NUM < 3040000
    virtual CPLErr FlushCache() override;
#else
    virtual CPLErr FlushCache(bool bAtClosing = false) override;
#endif

    virtual int GetOverviewCount() override;
    virtual GDALRasterBand* GetOverview(int) override;

private:
    CPL_DISALLOW_COPY_ASSIGN(OverviewBand)
};

OverviewDataset::OverviewDataset(::GDALDataset *main, int level
                                 , int thisLevelOnly)
    : main_(main), level_(level), thisLevelOnly_(thisLevelOnly)
    , gcpCount_(0), gcps_(), papszMD_RPC(), papszMD_GEOLOCATION()
{
    main->Reference();
    eAccess = main_->GetAccess();
    nRasterXSize =  main_->GetRasterBand(1)->GetOverview(level_)->GetXSize();
    nRasterYSize =  main_->GetRasterBand(1)->GetOverview(level_)->GetYSize();
    ovr_ = main_->GetRasterBand(1)->GetOverview(level_)->GetDataset();
    if (ovr_ && (ovr_ == main_)) {
        CPLDebug("geo::gdal",
                  "Dataset of overview is the same as the main band. "
                  "This is not expected");
        ovr_ = nullptr;
    }

    nBands = main_->GetRasterCount();
    for (int i = 0; i < nBands; ++i) {
        SetBand(i + 1, new OverviewBand(this, i + 1));
    }

    if (main_->GetDriver()) {
        // create a fake driver
        poDriver = new ::GDALDriver();
        poDriver->SetDescription(main_->GetDriver()->GetDescription());
        poDriver->SetMetadata(main_->GetDriver()->GetMetadata());
    }

    SetDescription(main_->GetDescription());

    CPLDebug("geo::gdal", "OverviewDataset(%s, this=%p) creation.",
             main_->GetDescription(), (void*)this);

    papszOpenOptions = CSLDuplicate(main_->GetOpenOptions());
    papszOpenOptions = CSLSetNameValue(papszOpenOptions, "OVERVIEW_LEVEL"
                                       , CPLSPrintf("%d", level_));
}

OverviewDataset::~OverviewDataset()
{
    FlushCache();

    CloseDependentDatasets();

    if (gcpCount_ > 0) {
        GDALDeinitGCPs(gcpCount_, gcps_);
        CPLFree(gcps_);
    }
    CSLDestroy(papszMD_RPC);
    CSLDestroy(papszMD_GEOLOCATION);
    delete poDriver;
}

int OverviewDataset::CloseDependentDatasets()
{
    if (!main_) { return false; }

    for (int i = 0; i < nBands; ++i) {
        if (auto band = dynamic_cast<OverviewBand*>(papoBands[i])) {
            band->underlyingBand_ = nullptr;
        } else {
            CPLError(CE_Fatal, CPLE_AppDefined,
                     "OverviewBand cast fail.");
            return false;
        }
    }

    if (main_->ReleaseRef()) {
        main_ = nullptr;
        return true;
    }

    return false;
}

CPLErr OverviewDataset::IRasterIO(::GDALRWFlag eRWFlag, int nXOff, int nYOff
                                  , int nXSize, int nYSize, void *pData
                                  , int nBufXSize, int nBufYSize
                                  , ::GDALDataType eBufType
                                  , int nBandCount, int *panBandMap
                                  , ::GSpacing nPixelSpace
                                  , ::GSpacing nLineSpace
                                  , ::GSpacing nBandSpace
                                  , ::GDALRasterIOExtraArg *psExtraArg)

{
    // In case the overview bands are really linked to a dataset, then issue
    // the request to that dataset.
    if (ovr_) {
        return ovr_->RasterIO
            (eRWFlag, nXOff, nYOff, nXSize, nYSize, pData, nBufXSize
             , nBufYSize, eBufType, nBandCount, panBandMap, nPixelSpace
             , nLineSpace, nBandSpace, psExtraArg);
    }

    ::GDALProgressFunc pfnProgressGlobal(psExtraArg->pfnProgress);
    auto pProgressDataGlobal(psExtraArg->pProgressData);
    CPLErr eErr(CE_None);

    for (int bi = 0; bi < nBandCount && (eErr == CE_None); ++bi) {
        auto band(dynamic_cast<OverviewBand *>(GetRasterBand(panBandMap[bi])));
        if (!band) {
            eErr = CE_Failure;
            break;
        }

        auto pabyBandData(static_cast<GByte *>(pData) + bi * nBandSpace);

        psExtraArg->pfnProgress = GDALScaledProgress;
        psExtraArg->pProgressData =
            ::GDALCreateScaledProgress(1.0 * bi / nBandCount
                                       , 1.0 * (bi + 1) / nBandCount
                                       , pfnProgressGlobal
                                       , pProgressDataGlobal);

        eErr = band->IRasterIO(eRWFlag, nXOff, nYOff, nXSize, nYSize
                                 , pabyBandData
                                 , nBufXSize, nBufYSize
                                 , eBufType, nPixelSpace
                                 , nLineSpace, psExtraArg);

        ::GDALDestroyScaledProgress(psExtraArg->pProgressData);
    }

    psExtraArg->pfnProgress = pfnProgressGlobal;
    psExtraArg->pProgressData = pProgressDataGlobal;

    return eErr;
}


CPLErr OverviewDataset::GetGeoTransform(double *padfTransform)
{
    double adfGeoTransform[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    if (main_->GetGeoTransform(adfGeoTransform) != CE_None) {
        return CE_Failure;
    }

    adfGeoTransform[1] *=
        static_cast<double>(main_->GetRasterXSize()) / nRasterXSize;
    adfGeoTransform[2] *=
        static_cast<double>(main_->GetRasterYSize()) / nRasterYSize;
    adfGeoTransform[4] *=
        static_cast<double>(main_->GetRasterXSize()) / nRasterXSize;
    adfGeoTransform[5] *=
        static_cast<double>(main_->GetRasterYSize()) / nRasterYSize;

    memcpy(padfTransform, adfGeoTransform, sizeof(double) * 6);

    return CE_None;
}

const ::GDAL_GCP *OverviewDataset::GetGCPs()
{
    if (gcps_) { return gcps_; }

    const auto pasGCPsMain = main_->GetGCPs();
    if (!pasGCPsMain) { return nullptr; }
    gcpCount_ = main_->GetGCPCount();

    gcps_ = ::GDALDuplicateGCPs(gcpCount_, pasGCPsMain);
    for (int i = 0; i < gcpCount_; ++i) {
        gcps_[i].dfGCPPixel *= static_cast<double>(nRasterXSize) /
            main_->GetRasterXSize();
        gcps_[i].dfGCPLine *= static_cast<double>(nRasterYSize) /
            main_->GetRasterYSize();
    }
    return gcps_;
}

void OverviewDataset::rescale(char**& papszMD, const char* pszItem
                              , double dfRatio, double dfDefaultVal)
{
    double dfVal =
        CPLAtofM(CSLFetchNameValueDef(papszMD, pszItem,
                                      CPLSPrintf("%.18g", dfDefaultVal)));
    dfVal *= dfRatio;
    papszMD = CSLSetNameValue(papszMD, pszItem, CPLSPrintf("%.18g", dfVal));
}

char** OverviewDataset::GetMetadata(const char *pszDomain)
{
    if (ovr_) {
        auto papszMD(ovr_->GetMetadata(pszDomain));
        if (papszMD) { return papszMD; }
    }

    auto papszMD(main_->GetMetadata(pszDomain));

    if (pszDomain && EQUAL(pszDomain, MD_DOMAIN_RPC) && papszMD) {
        if (papszMD_RPC) { return papszMD_RPC; }
        papszMD_RPC = CSLDuplicate(papszMD);

        rescale(papszMD_RPC, RPC_LINE_OFF
                , static_cast<double>(nRasterYSize) / main_->GetRasterYSize()
                , 0.0);
        rescale(papszMD_RPC, RPC_LINE_SCALE
                , static_cast<double>(nRasterYSize) / main_->GetRasterYSize()
                , 1.0);
        rescale(papszMD_RPC, RPC_SAMP_OFF
                , static_cast<double>(nRasterXSize) / main_->GetRasterXSize()
                , 0.0);
        rescale(papszMD_RPC, RPC_SAMP_SCALE
                , static_cast<double>(nRasterXSize) / main_->GetRasterXSize()
                , 1.0);
        papszMD = papszMD_RPC;
    }

    if (pszDomain && EQUAL(pszDomain, "GEOLOCATION") && papszMD) {
        if (papszMD_GEOLOCATION) { return papszMD_GEOLOCATION; }
        papszMD_GEOLOCATION = CSLDuplicate(papszMD);

        rescale(papszMD_GEOLOCATION, "PIXEL_OFFSET"
                , static_cast<double>(main_->GetRasterXSize()) /
                nRasterXSize, 0.0);
        rescale(papszMD_GEOLOCATION, "LINE_OFFSET"
                , static_cast<double>(main_->GetRasterYSize()) /
                nRasterYSize, 0.0);

        rescale(papszMD_GEOLOCATION, "PIXEL_STEP"
                , static_cast<double>(nRasterXSize) / main_->GetRasterXSize()
                , 1.0);
        rescale(papszMD_GEOLOCATION, "LINE_STEP"
                , static_cast<double>(nRasterYSize) / main_->GetRasterYSize()
                , 1.0);

        papszMD = papszMD_GEOLOCATION;
    }

    return papszMD;
}

const char* OverviewDataset::GetMetadataItem(const char * pszName
                                             , const char * pszDomain)
{
    if (ovr_) {
        if (auto pszValue = ovr_->GetMetadataItem(pszName, pszDomain)) {
            return pszValue;
        }
    }

    if (pszDomain
        && (EQUAL(pszDomain, "RPC") || EQUAL(pszDomain, "GEOLOCATION")))
    {
        auto papszMD(GetMetadata(pszDomain));
        return CSLFetchNameValue(papszMD, pszName);
    }

    return main_->GetMetadataItem(pszName, pszDomain);
}

OverviewBand::OverviewBand(OverviewDataset *ds, int band)
    : underlyingBand_
      (ds->main_->GetRasterBand(band)->GetOverview(ds->level_))
{
    poDS = ds;
    nBand = band;
    nRasterXSize = ds->nRasterXSize;
    nRasterYSize = ds->nRasterYSize;
    eDataType = underlyingBand_->GetRasterDataType();
    underlyingBand_->GetBlockSize(&nBlockXSize, &nBlockYSize);
}

#if GDAL_VERSION_NUM < 3040000

CPLErr OverviewBand::FlushCache()
{
    if (underlyingBand_) {
        return underlyingBand_->FlushCache();
    }
    return CE_None;
}

#else

CPLErr OverviewBand::FlushCache(bool bAtClosing)
{
    if (underlyingBand_) {
        return underlyingBand_->FlushCache(bAtClosing);
    }
    return CE_None;
}
#endif

int OverviewBand::GetOverviewCount()
{
    auto ovr(dynamic_cast<OverviewDataset*>(poDS));
    if (!ovr) {
        CPLError(CE_Fatal, CPLE_AppDefined, "OverviewDataset cast fail.");
        return 0;
    }
    if (ovr->thisLevelOnly_) { return 0; }

    return  (ovr->main_->GetRasterBand(nBand)->GetOverviewCount()
             - ovr->level_ - 1);
}

::GDALRasterBand* OverviewBand::GetOverview(int overview)
{
    if ((overview < 0) || (overview >= GetOverviewCount())) { return {}; }
    auto ovr(dynamic_cast<OverviewDataset*>(poDS));
    if (!ovr) {
        CPLError(CE_Fatal, CPLE_AppDefined, "OverviewDataset cast fail.");
        return {};
    }
    return (ovr->main_->GetRasterBand(nBand)
            ->GetOverview(overview + ovr->level_ + 1));
}

} // namespace gdal

::GDALDataset* createOverviewDataset(::GDALDataset *ds, int level)
{
    // Sanity checks.
    auto bands(ds->GetRasterCount());
    if (!bands) { return {}; }
    for (int i = 1; i <= bands; ++i) {
        if (!ds->GetRasterBand(i)->GetOverview(level)) {
            return {};
        }

        if ((ds->GetRasterBand(i)->GetOverview(level)->GetXSize()
             != ds->GetRasterBand(1)->GetOverview(level)->GetXSize())
            || (ds->GetRasterBand(i)->GetOverview(level)->GetYSize()
                != ds->GetRasterBand(1)->GetOverview(level)->GetYSize()))
        {
            return {};
        }
    }

    return new gdal::OverviewDataset(ds, level, true);
}

} } // namespace geo::detail
