#ifndef geo_po_hpp_included_
#define geo_po_hpp_included_

#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "./srsdef.hpp"

namespace geo {

inline void validate(boost::any &v
                     , const std::vector<std::string> &values
                     , SrsDefinition*, int)
{
    namespace po = boost::program_options;
    namespace ba = boost::algorithm;

    po::validators::check_first_occurrence(v);
    std::string s(ba::trim_copy(po::validators::get_single_string(values)));

    if (s.empty()) {
        throw po::validation_error(po::validation_error::invalid_option_value);
    }
    if (s.front() == '+') {
        // proj string
        v = boost::any(SrsDefinition(s, SrsDefinition::Type::proj4));
        return;
    } else if (ba::istarts_with(s, "epsg:")) {
        // epsg
        v = boost::any(SrsDefinition(s.substr(5), SrsDefinition::Type::epsg));
    } else {
        v = boost::any(SrsDefinition(s, SrsDefinition::Type::wkt));
    }
}

} // namespace geo

#endif // geo_po_hpp_included_
