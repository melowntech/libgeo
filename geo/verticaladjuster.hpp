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
#ifndef geo_verticaladjuster_hpp_included_
#define geo_verticaladjuster_hpp_included_

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "math/geometry_core.hpp"

#include "./srsfactors.hpp"

namespace geo {

class VerticalAdjuster {
public:
    VerticalAdjuster() {}
    VerticalAdjuster(const SrsDefinition &srs) : sf_(boost::in_place(srs)) {}
    VerticalAdjuster(const SrsDefinition &srs, const SrsDefinition &srcSrs)
        : sf_(boost::in_place(srs, srcSrs))
    {}

    VerticalAdjuster(bool apply, const SrsDefinition &srs);
    VerticalAdjuster(bool apply, const SrsDefinition &srs
                     , const SrsDefinition &srcSrs);

    VerticalAdjuster(const SrsFactors &factors)
        : sf_(boost::in_place(factors)) {}

    /** Apply/unapply vertical adjustment.
     */
    math::Point3 operator()(math::Point3 p, bool inverse = false) const;

private:
    boost::optional<SrsFactors> sf_;
};

} // namespace geo

#endif // geo_verticaladjuster_hpp_included_
