#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"

#include "./heightcoding.hpp"

namespace geo {

void heightCode(::GDALDataset &vectorDs
                , const GeoDataset &rasterDs
                , std::ostream &os
                , const HeightCodingConfig &config)
{
    // TODO: magic happens here
    (void) vectorDs;
    (void) rasterDs;

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
}

} // namespace geo
