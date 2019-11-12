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

#include <boost/lexical_cast.hpp>

#include <proj_api.h>

#include "dbglog/dbglog.hpp"

#include "math/geometry.hpp"

#include "csconvertor.hpp"

namespace geo {

class CsConvertor::Impl : boost::noncopyable
{
public:
    typedef std::shared_ptr<Impl> pointer;
    virtual ~Impl() {}
    virtual math::Point2 convert(const math::Point2 &p) const = 0;
    virtual math::Point3 convert(const math::Point3 &p) const = 0;
};

namespace {

class NoOpConvertor : public CsConvertor::Impl
{
public:
    virtual math::Point2 convert(const math::Point2 &p) const { return p; }
    virtual math::Point3 convert(const math::Point3 &p) const { return p; }
};

class ProjInst
{
public:
    projPJ pj;

    ProjInst(const SrsDefinition &srs) : pj(nullptr)
    {
        pj = pj_init_plus(srs.c_str());
        if (!pj)
        {
            LOGTHROW(err1, std::runtime_error)
                << "Failed to initialize proj converter: <"
                << srs << ">";
        }
    }

    ~ProjInst()
    {
        pj_free(pj);
    }
};

class ProjImpl : public CsConvertor::Impl
{
public:
    ProjImpl(const SrsDefinition &from, const SrsDefinition &to)
        : preMult(1), postMult(1), gridNotFoundReported(false)
    {
        if (!from.is(SrsDefinition::Type::proj4)
                || !to.is(SrsDefinition::Type::proj4))
        {
            LOGTHROW(fatal, std::logic_error)
                << "No GDAL: only proj to proj conversion is available.";
        }

        this->from = std::make_unique<ProjInst>(from);
        this->to = std::make_unique<ProjInst>(to);

        if (pj_is_latlong(this->from->pj))
            preMult = DEG_TO_RAD;
        if (pj_is_latlong(this->to->pj))
            postMult = RAD_TO_DEG;
    }

    virtual math::Point2 convert(const math::Point2 &p) const {
        math::Point3 p3(p[0], p[1], 0);
        p3 = convert(p3);
        return math::Point2(p3[0], p3[1]);
    }

    virtual math::Point3 convert(const math::Point3 &p) const {
        assert(!std::isnan(p[0]) && !std::isnan(p[1]) && !std::isnan(p[2]));
        math::Point3 r(p);
        r[0] *= preMult;
        r[1] *= preMult;
        auto err = pj_transform(from->pj, to->pj, 1, 1, &r[0], &r[1], &r[2]);
        if (err == -38)
        {
            // prevent repeatedly logging the same message
            if (gridNotFoundReported)
            {
                throw std::runtime_error(
                        "Error in coordinate transformation <-38>:"
                        "<failed to load datum shift file>");
            }
            gridNotFoundReported = true;
        }
        if (err != 0)
        {
            LOGTHROW(err2, std::runtime_error)
                << "Error in coordinate transformation <"
                << err << ">:<" << pj_strerrno(err) << ">, point <"
                << p << ">";
        }
        r[0] *= postMult;
        r[1] *= postMult;
        return r;
    }

    std::unique_ptr<ProjInst> from, to;
    double preMult, postMult;
    mutable bool gridNotFoundReported;
};

static volatile struct Initializer {
    Initializer() {
        // initializes locks and default context
        ::pj_get_default_ctx();
    }
} initializer;

} // namespace

CsConvertor::CsConvertor(const SrsDefinition &from, const SrsDefinition &to)
    : trans_(std::make_shared<ProjImpl>(from, to))
{
    LOG(info1) << "Coordinate system transformation ("
               << from << " -> " << to << ").";
}

CsConvertor::CsConvertor()
    : trans_(std::make_shared<NoOpConvertor>())
{
    LOG(info1) << "Coordinate system transformation: no-op.";
}

CsConvertor::CsConvertor(const std::shared_ptr<Impl> &trans)
    : trans_(trans)
{}

math::Point2 CsConvertor::operator()(const math::Point2 &p) const
{
    return trans_->convert(p);
}

math::Point3 CsConvertor::operator()(const math::Point3 &p) const
{
    return trans_->convert(p);
}

math::Point4 CsConvertor::operator()(const math::Point4 &p) const
{
    const auto pp(trans_->convert
                  (math::Point3(p(0) / p(3), p(1) / p(3), p(2) / p(3))));
    return math::Point4(pp(0), pp(1), pp(2), 1.0);
}

} // namespace geo
