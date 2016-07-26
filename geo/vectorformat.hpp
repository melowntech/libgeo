#ifndef geo_vectorformat_hpp_included_
#define geo_vectorformat_hpp_included_

#include "utility/enum-io.hpp"

namespace geo {

UTILITY_GENERATE_ENUM(VectorFormat,
                      ((geodataJson))
                      )

/** Returns content type string for given file format
 *  Always returns a valid string
 */
const char* contentType(VectorFormat format);

} // namespace geo

#endif // geo_vectorformat_hpp_included_
