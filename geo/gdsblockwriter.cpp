/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <numeric>

#include "utility/cppversion.hpp"
#include "utility/expect.hpp"

#include "gdsblockwriter.hpp"
#include "gdal.hpp"
#include "detail/options.hpp"


namespace fs = boost::filesystem;

namespace geo {

namespace {

Options buildOptions(const Options &in, const math::Size2 &blockSize)
{
    Options out("TILED", true);
    out("BLOCKXSIZE", blockSize.width);
    out("BLOCKYSIZE", blockSize.height);

    for (const auto &opt : in.options) {
        // skip explicitely stored values
        if ((opt.first == "TILED")
            || (opt.first == "BLOCKXSIZE")
            || (opt.first == "BLOCKYSIZE"))
        {
            continue;
        }
        out.options.push_back(opt);
    }

    return out;
}

std::pair<fs::path, std::string>
getDsInfo(const boost::filesystem::path &inPath
          , const GeoDataset::Format &format)
{
    std::pair<fs::path, std::string> res(inPath, "");
    auto &path(res.first);
    auto &storageFormat(res.second);

    switch (format.storageType) {
    case GeoDataset::Format::Storage::gtiff:
        storageFormat = "GTiff";
        break;

    case GeoDataset::Format::Storage::png:
        storageFormat = "PNG";
        break;

    case GeoDataset::Format::Storage::jpeg:
        storageFormat = "JPEG";
        break;

    case GeoDataset::Format::Storage::vrt:
        storageFormat = "VRT";
        break;

    case GeoDataset::Format::Storage::memory:
        storageFormat = "MEM";
        path = "MEM";
        break;

    case GeoDataset::Format::Storage::custom:
        storageFormat = format.driver;
        break;
    }

    return res;
}

typedef std::unique_ptr< ::GDALDataset> Dataset;

typedef std::vector<int> BandMap;

void writeBlock(::GDALDataset &ds, BandMap &bandMap
                , const GdsBlockWriter::Block &block
                , const math::Size2 &blockSize)
{
    if (block.channels != bandMap.size()) {
        LOGTHROW(err2, std::runtime_error)
            << "Invalid number of channels (" << block.channels
            << "), expected " << bandMap.size() << ".";
    }

    auto res(ds.RasterIO(::GDALRWFlag::GF_Write               // eRWFlag
                         , block.index(0) * blockSize.width   // nXOff
                         , block.index(1) * blockSize.height  // nYOff
                         , block.size.width                   // nXSize
                         , block.size.height                  // nYSize
                         , const_cast<void*>(block.data)      // pData
                         , block.size.width                   // nBufXSize
                         , block.size.height                  // nBufYSize
                         , block.type                         // eBufType
                         , bandMap.size()                     // nBandCount
                         , bandMap.data()                     // panBandMap
                         , block.pixelSpace                   // nPixelSpace,
                         , block.lineSpace                    // nLineSpace
                         , block.bandSpace                    // nBandSpace
                         , nullptr));

    if (res !=CE_None) {
        LOGTHROW(err2, std::runtime_error)
            << "Error writing block into dataset.";
    }
}

} // namespace

class GdsBlockWriter::Detail {
public:
    Detail(const boost::filesystem::path &inPath
           , const math::Size2 &size
           , const GeoDataset::Format &format
           , const math::Size2 &blockSize
           , const NodataValue &noDataValue
           , const Options &options)
        : running_(false)
    {
        fs::path path;
        std::string storageFormat;
        std::tie(path, storageFormat) = getDsInfo(inPath, format);

        auto driver(::GetGDALDriverManager()
                    ->GetDriverByName(storageFormat.c_str()));
        if (!driver) {
            LOGTHROW(err2, std::runtime_error)
                << "Cannot find GDAL driver for <" << storageFormat
                << "> format.";
        }

        auto metadata(driver->GetMetadata());
        if (!CSLFetchBoolean(metadata, GDAL_DCAP_CREATE, FALSE)) {
            LOGTHROW(err2, std::runtime_error)
                << "GDAL driver for <" << storageFormat
                << "> format doesn't support creation.";
        }

        /* create parent directory for output dataset */ {
            auto pp(path.parent_path());
            if (!pp.empty()) { create_directories(pp); }
        }

        Dataset ds
            (driver->Create
             (path.string().c_str()
              , size.width, size.height
              , format.channels.size(), format.channelType
              , detail::OptionsWrapper(buildOptions(options, blockSize))));

        utility::expect(ds.get(), "Failed to create new dataset.");

        // no projection, no geotrafo

        {
            int i = 1;
            for (auto ci : format.channels) {
                auto band(ds->GetRasterBand(i));
                utility::expect(band, "Cannot find band %d.", (i));
                if (noDataValue) { band->SetNoDataValue(*noDataValue); }
                band->SetColorInterpretation(ci);
                if (format.colorTable) {
                    band->SetColorTable(format.colorTable.get());
                }
                ++i;
            }
        }
        ds->FlushCache();

        start(std::move(ds), blockSize);
    }

    ~Detail() {
        try {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!running_) { return; }
            }
            LOG(warn2) << "Geo Dataset Block Writer was not flushed. "
                       << "Flushing.";
            stop();
        } catch (const std::exception &e) {
            LOG(warn3)
                << "Error during terminating Geo Dataset Block Writer: <"
                << e.what() << ">.";
        }
    }

    void start(Dataset &&ds, const math::Size2 &blockSize);

    void stop();

    void write(const Batch &batch);

private:
    void run(Dataset &&ds, const math::Size2 &blockSize);

    void checkError();

    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool running_;

    std::queue<Batch> queue_;
    std::exception_ptr error_;
};

void GdsBlockWriter::Detail::checkError()
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!error_) { return; }
    auto e(error_);
    error_ = {};
    std::rethrow_exception(e);
}

void GdsBlockWriter::Detail::start(Dataset &&ds
                                   , const math::Size2 &blockSize)
{
    worker_ = std::thread(&Detail::run, this, std::move(ds), blockSize);
    running_ = true;
}

void GdsBlockWriter::Detail::stop()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!running_) { return; }
        running_ = false;
        cond_.notify_all();
    }

    worker_.join();
    checkError();
}

void GdsBlockWriter::Detail::write(const Batch &batch)
{
    checkError();

    // TODO: release on errror
    if (batch.acquire) { batch.acquire(); }

    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(batch);
        cond_.notify_all();
    }
}

extern "C" {

void erroHandler( CPLErr eErrClass, int err_no, const char *msg)
{
    switch (eErrClass) {
    case CE_Debug:
        LOG(debug) << "gdal error " << err_no << ": " << msg;
        return;

    case CE_Warning:
        LOG(warn2) << "gdal error " << err_no << ": " << msg;
        return;

    case CE_Failure:
        LOG(err2) << "gdal error " << err_no << ": " << msg;
        return;

    case CE_Fatal:
        LOG(fatal) << "gdal error " << err_no << ": " << msg;
        return;

    default: break;
    }
}

} // extern "C"

void GdsBlockWriter::Detail::run(Dataset &&ds, const math::Size2 &blockSize)
{
    // prepare band mapping
    BandMap bandMap(ds->GetRasterCount());
    std::iota(bandMap.begin(), bandMap.end(), 1);

    ::CPLSetErrorHandler(erroHandler);

    for (;;) {
        std::unique_lock<std::mutex> lock(mutex_);
        // wait for work to be done
        if (queue_.empty()) {
            // stop if there is no work and we are asked to stop
            if (!running_) { break; }
            cond_.wait(lock);
            continue;
        }

        auto batch(std::move(queue_.front()));
        queue_.pop();
        lock.unlock();

        for (const auto &block : batch.blocks) {
            try {
                writeBlock(*ds, bandMap, block, blockSize);
            } catch (const std::exception &e) {
                LOG(warn2) << "Failed to write a block: <"
                           << e.what() << ">.";

                std::unique_lock<std::mutex> lock(mutex_);
                error_ = std::current_exception();
                break;
            }
        }

        try {
            if (batch.release) { batch.release(); }
        } catch (const std::exception &e) {
            LOG(warn2) << "Failed to release a batch: <"
                       << e.what() << ">.";
            std::unique_lock<std::mutex> lock(mutex_);
            error_ = std::current_exception();
        }
    }

    ds->FlushCache();
    ds.reset();
}

GdsBlockWriter::GdsBlockWriter(const boost::filesystem::path &path
                               , const math::Size2 &size
                               , const GeoDataset::Format &format
                               , const math::Size2 &blockSize
                               , const NodataValue &noDataValue
                               , const Options &options)
    : detail_(std::make_unique<Detail>
              (path, size, format, blockSize, noDataValue, options))
{
}

GdsBlockWriter::~GdsBlockWriter() {}

void GdsBlockWriter::write(const Batch &batch)
{
    detail_->write(batch);
}

void GdsBlockWriter::finish() {
    detail_->stop();
}

} // namespace geo
