#include "./vectorformat.hpp"

namespace geo {

const char* contentType(VectorFormat format)
{
    switch (format) {
    case VectorFormat::geodataJson:
        return "application/json; charset=utf-8";
    }

    // should be never reached
    throw;
}

} // namespace geo
