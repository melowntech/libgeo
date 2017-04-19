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
#include <fstream>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "./featurelayers.hpp"
#include "./heightcoding.hpp"

namespace geo { namespace heightcoding {

Metadata heightCode(::GDALDataset &vectorDs
                    , const std::vector<const GeoDataset*> &rasterDs
                   , std::ostream &os, const Config &config)
{
    Metadata metadata;
    
    // remember start position in the output stream
    const auto startPos(os.tellp());

    // load feature layers from vectorDs
    FeatureLayers featureLayers(vectorDs, config.vectorDsSrs);

    // heightcode
    auto workingSrs(config.workingSrs ? config.workingSrs : config.rasterDsSrs);

    if (workingSrs) {
        LOG(info2) << "The following SRS shall be used in heightcoding: \""
                   <<  workingSrs->string() << "\"";        
    } else {
        LOG(info2) << "No hint given as to what SRS to use in heightcoding.";
    }

    // TODO: use full stack to heightcode result
    featureLayers.heightcode(*rasterDs.back()
            , workingSrs
            , config.outputVerticalAdjust
            , FeatureLayers::HeightcodeMode::auto_);
    
    // convert 3D polygons to surfaces
    featureLayers.convert3DPolygons();
    
    // transform to output srs
    if (config.outputSrs)
        featureLayers.transform(config.outputSrs.get());
    
    // update metadata.extents
    auto bb(featureLayers.boundingBox(config.outputSrs));
    if (bb) metadata.extents = bb.get();

    // output
    switch (config.format) {
    case VectorFormat::geodataJson: {
        
        os.precision(15);
        featureLayers.dumpVTSGeodata(os);
        break;
    }

    default:
        // unsupported
        LOGTHROW(err1, std::runtime_error)
            << "Unsupported output vector format <" << config.format << ">.";
    }

    // set file size in metadata
    metadata.fileSize = (os.tellp() - startPos);
    return metadata;
}

} } // namespace geo::heightcoding
