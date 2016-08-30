#include <fstream>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "geo/featurelayers.hpp"

#include "./heightcoding.hpp"

namespace geo { namespace heightcoding {

Metadata heightCode(::GDALDataset &vectorDs, const GeoDataset &rasterDs
                   , std::ostream &os, const Config &config)
{
    Metadata metadata;
    
    // remember start position in the output stream
    const auto startPos(os.tellp());

    // load feature layers from vectorDs
    FeatureLayers featureLayers(vectorDs);

    // heightcode
    auto workingSrs(config.workingSrs ? config.workingSrs : config.rasterDsSrs);

    if (workingSrs) {
        LOG(info2) << "The following SRS shall be used in heightcoding: \""
                   <<  workingSrs->string() << "\"";        
    } else {
        LOG(info2) << "No hint given as to what SRS to use in heightcoding.";
    }
    
    featureLayers.heightcode(rasterDs
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
