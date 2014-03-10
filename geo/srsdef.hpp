#ifndef geo_srsdef_hpp_included_
#define geo_srsdef_hpp_included_

#include <string>

namespace geo {

struct SrsDefinition {
    enum class Type { proj4, wkt };

    std::string srs;
    Type type;

    SrsDefinition(const std::string &srs) : srs(srs), type(Type::proj4) {}
    SrsDefinition(const std::string &srs, Type type) : srs(srs), type(type) {}

    SrsDefinition as(Type type) const;
};

} // namespace geo

#endif // geo_srsdef_hpp_included_
