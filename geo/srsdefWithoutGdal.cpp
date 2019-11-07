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
#include <stdexcept>
#include <vector>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <proj_api.h>
#include <GeographicLib/Geocentric.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "srsdef.hpp"

namespace geo {

SrsDefinition SrsDefinition::as(Type dstType) const
{
    assert(type == SrsDefinition::Type::proj4);
    if (type == dstType)
        return *this;
    throw;
}

bool SrsDefinition::convertibleTo(Type dstType) const
{
    assert(type == SrsDefinition::Type::proj4);
    return dstType == SrsDefinition::Type::proj4;
}

math::Point3 ellipsoid(const SrsDefinition &srs)
{
    struct ProjInst
    {
        projPJ pj;

        ProjInst(projPJ pj) : pj(pj)
        {}

        ~ProjInst()
        {
            pj_free(pj);
        }
    };

    ProjInst pi1(pj_init_plus(srs.c_str()));
    if (!pi1.pj)
    {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to initialize proj converter: <"
            << srs << ">";
    }

    double ma = 0, ec2 = 0;
    pj_get_spheroid_defn(pi1.pj, &ma, &ec2);
    return { ma, ma, ma * std::sqrt(1 - ec2) };
}

SrsDefinition SrsDefinition::fromString(std::string value)
{
    namespace ba = boost::algorithm;
    ba::trim(value);
    if (value.empty())
        return {};
    return SrsDefinition(value, SrsDefinition::Type::proj4);
}

std::string SrsDefinition::toString() const
{
    return boost::lexical_cast<std::string>(*this);
}

} // namespace geo
