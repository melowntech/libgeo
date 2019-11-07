/**
 * Copyright (c) 2017-2019 Melown Technologies SE
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

#ifndef geo_csconvert_hpp_included_
#define geo_csconvert_hpp_included_

#include <string>
#include <memory>
#include <algorithm>
#include <iterator>

#include "math/geometry_core.hpp"

#include "project.hpp"
#include "srsdef.hpp"

namespace geo {

class CsConvertor {
public:
    CsConvertor(const SrsDefinition &from, const SrsDefinition &to);

#ifdef GEO_HAS_GDAL
    CsConvertor(const OGRSpatialReference &from
                , const OGRSpatialReference &to);
    CsConvertor(const SrsDefinition &from, const OGRSpatialReference &to);
    CsConvertor(const OGRSpatialReference &from, const SrsDefinition &to);

    CsConvertor(const SrsDefinition &from, const Enu &to);
    CsConvertor(const Enu &from, const SrsDefinition &to);
#endif // GEO_HAS_GDAL

    /** Creates no-op CS convertor. No conversion takes place.
     *  Useful when we have API that expects CS convertor instance but we have
     *  data in the proper SRS already.
     */
    CsConvertor();

    math::Point2 operator()(const math::Point2 &p) const;
    math::Point3 operator()(const math::Point3 &p) const;

    math::Points2 operator()(const math::Points2 &p) const;
    math::Points3 operator()(const math::Points3 &p) const;

    /** Homogeneous point support.
     */
    math::Point4 operator()(const math::Point4 &p) const;

    // return extents containing original extents
    math::Extents2 operator()(const math::Extents2 &p) const;
    math::Extents3 operator()(const math::Extents3 &p) const;

#ifdef GEO_HAS_GDAL
    CsConvertor inverse() const;

    /** Is destination SRS projected?
     */
    bool isProjected() const;

    /** Are source and destination SRSs equal in linear units?
     */
    bool areSrsEqual() const;
#endif // GEO_HAS_GDAL

    class Impl;

private:
    CsConvertor(const std::shared_ptr<Impl> &trans);
    std::shared_ptr<Impl> trans_;
};

/** Generic convertor. Can be used for points. Analogous to matrix product.
 */
math::Point4 prod(const CsConvertor &conv, const math::Point4 &value);

// inline method implementation

inline math::Extents2 CsConvertor::operator()(const math::Extents2 &e) const
{
    math::Extents2 res(operator()( ll(e) ));
    update(res, operator()( ul(e) ));
    update(res, operator()( ur(e) ));
    update(res, operator()( lr(e) ));
    return res;
}

inline math::Extents3 CsConvertor::operator()( const math::Extents3 &e) const
{
    math::Extents3 res(operator()( bll(e) ));
    update(res, operator()( bul(e) ));
    update(res, operator()( bur(e) ));
    update(res, operator()( blr(e) ));

    update(res, operator()( tll(e) ));
    update(res, operator()( tul(e) ));
    update(res, operator()( tur(e) ));
    update(res, operator()( tlr(e) ));

    return res;
}

inline math::Point4 prod(const CsConvertor &conv, const math::Point4 &value)
{
    return conv(value);
}

inline math::Points2 CsConvertor::operator()(const math::Points2 &p) const
{
    math::Points2 out;
    std::transform(p.begin(), p.end(), std::back_inserter(out), *this);
    return out;
}

inline math::Points3 CsConvertor::operator()(const math::Points3 &p) const
{
    math::Points3 out;
    std::transform(p.begin(), p.end(), std::back_inserter(out), *this);
    return out;
}

} // namespace geo

#endif // geo_csconvert_hpp_included_
