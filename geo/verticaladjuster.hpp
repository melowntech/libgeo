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

#include "srsfactors.hpp"

namespace geo {

class VerticalAdjuster {
public:
    VerticalAdjuster() : inverse_() {}

    /** \param SRS for both adjustment and querty
     *  \param inverse unadjust by default if true
     */
    VerticalAdjuster(const SrsDefinition &srs, bool inverse = false);

    /** \param srs SRS for adjustment
     *  \param srcSrs SRS for query
     *  \param inverse unadjust by default if true
     */
    VerticalAdjuster(const SrsDefinition &srs, const SrsDefinition &srcSrs
                     , bool inverse = false);

    /** \param apply creates dummy adjuster if false
     *  \param SRS for both adjustment and querty
     *  \param inverse unadjust by default if true
     */
    VerticalAdjuster(bool apply, const SrsDefinition &srs
                     , bool inverse = false);

    /** \param apply creates dummy adjuster if false
     *  \param srs SRS for adjustment
     *  \param srcSrs SRS for query
     *  \param inverse unadjust by default if true
     */
    VerticalAdjuster(bool apply, const SrsDefinition &srs
                     , const SrsDefinition &srcSrs
                     , bool inverse = false);

    /** \param factors SRS factors
     *  \param inverse unadjust by default if true
     */
    VerticalAdjuster(const SrsFactors &factors, bool inverse = false)
        : sf_(boost::in_place(factors)), inverse_(inverse)
    {}

    /** There is not vertical adjustment in ENU system. Added for convenience.
     */
    VerticalAdjuster(const Enu&) : inverse_() {}

    /** There is not vertical adjustment in ENU system. Added for convenience.
     */
    VerticalAdjuster(bool, const Enu&, bool = false) : inverse_() {}

    /** Apply/unapply vertical adjustment.
     *
     *  Apply: Z coordinate is multiplied by scaling factor
     *  Unapply: Z coordinate is divided by scaling factor
     *
     *  Operation is determined from (inverse XOR inverse_):
     *     false: apply vertical adjustment
     *     true: unapply vertical adjustment
     */
    math::Point3 operator()(math::Point3 p, bool inverse = false) const;

    /** Apply/unapply vertical adjustment.
     *  Homogeneous point version.
     */
    math::Point4 operator()(const math::Point4 &p, bool inverse = false) const;

private:
    boost::optional<SrsFactors> sf_;
    bool inverse_;
};

/** Generic convertor. Can be used for points. Analogous to matrix product.
 */
math::Point4 prod(const VerticalAdjuster &conv, const math::Point4 &value);

// inlines


inline math::Point4 prod(const VerticalAdjuster &adjuster
                         , const math::Point4 &value)
{
    return adjuster(value);
}

} // namespace geo

#endif // geo_verticaladjuster_hpp_included_
