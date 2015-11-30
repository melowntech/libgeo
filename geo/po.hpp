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

    auto srs(SrsDefinition::fromString
             (po::validators::get_single_string(values)));
    if (srs.empty()) {
        throw po::validation_error(po::validation_error::invalid_option_value);
    }

    // ok
    v = boost::any(srs);
}

} // namespace geo

#endif // geo_po_hpp_included_
