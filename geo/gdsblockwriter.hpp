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

#ifndef geo_gdsblockwriter_hpp_included
#define geo_gdsblockwriter_hpp_included

#include <memory>
#include <functional>

#include "geodataset.hpp"

namespace geo {

/** Asynchronous geo dataset block (tile) writer.
 *
 *  Uses block-based IO.
 */
class GdsBlockWriter {
public:
    /** Non-georeferenced version.
     */
    GdsBlockWriter(const boost::filesystem::path &path
                   , const math::Size2 &size
                   , const GeoDataset::Format &format
                   , const math::Size2 &blockSize
                   , const NodataValue &noDataValue = boost::none
                   , const Options &options = Options());

    ~GdsBlockWriter();

    /** Single block. Data should have the size specified in ctor except for
     *  last row/columm block.
     */
    struct Block {
        typedef std::vector<Block> list;

        math::Point2i index;

        ::GDALDataType type;
        math::Size2 size;
        std::size_t channels;
        std::size_t pixelSpace;
        std::size_t lineSpace;
        std::size_t bandSpace;
        const void *data;

        Block(const math::Point2i &index = math::Point2i()
              , ::GDALDataType type = ::GDALDataType()
              , const void *data = nullptr)
            : index(index), type(type), channels()
            , pixelSpace(), lineSpace(), bandSpace()
            , data(data)
        {}
    };

    typedef std::function<void()> Function;

    struct Batch {
        Block::list blocks;
        Function acquire;
        Function release;
    };

    /** Schedule write of multiple blocks.
     *
     *  \param batch batch of tiles
     */
    void write(const Batch &batch);

    /** Finishes
     */
    void finish();

    struct Detail;

private:
    std::unique_ptr<Detail> detail_;
};

} // namespace geo

#endif // geo_gdsblockwriter_hpp_included
