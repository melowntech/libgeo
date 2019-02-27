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
#ifndef geo_project_hpp_included_
#define geo_project_hpp_included_

#include <string>
#include <memory>
#include <stdexcept>

#include "math/geometry_core.hpp"

#include "srsdef.hpp"
#include "srsfactorsfwd.hpp"

namespace geo {

struct ProjectionError : public std::runtime_error {
    ProjectionError(const std::string &msg) : std::runtime_error(msg) {}
};

class Projection {
public:
    Projection(const SrsDefinition &def, bool inverse = false);

    math::Point2 operator()(const math::Point2 &p, bool deg = true) const;

    math::Point3 operator()(const math::Point3 &p, bool deg = true) const;

    Projection rev() const { return { proj_, !inverse_ }; };

    friend class SrsFactors;

private:
    Projection(const std::shared_ptr<void> proj, bool inverse)
        : proj_(proj), inverse_(inverse)
    {}

    std::shared_ptr<void> proj_;
    bool inverse_;
};

} // namespace geo

#endif // geo_project_hpp_included_
