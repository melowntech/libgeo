#include <fstream>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "./heightcoding.hpp"

namespace geo { namespace heightcoding {

Metadata heightCode(::GDALDataset &vectorDs, const GeoDataset &rasterDs
                    , std::ostream &os, const Config &config)
{
    // remember start position in the output stream
    const auto startPos(os.tellp());

    // TODO: magic happens here
    (void) vectorDs;
    (void) rasterDs;

    Metadata metadata;

    // TODO: update metadata.extents

    // output
    switch (config.format) {
    case VectorFormat::geodataJson: {
        Json::Value output(Json::objectValue);

        // TODO: fill output from geometries here

        os.precision(15);
        Json::StyledStreamWriter().write(os, output);
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
