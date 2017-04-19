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
#ifndef geo_srsdactors_hpp_included_
#define geo_srsdactors_hpp_included_

#include <string>
#include <memory>

#include "math/geometry_core.hpp"

#include "./srsdef.hpp"
#include "./project.hpp"

namespace geo {

class SrsFactors {
public:
    /** Creates SRS factors interfaces; all queries are expected to be in given
     *  SRS.
     */
    SrsFactors(const SrsDefinition &def);

    /** Creates SRS factors interfaces; all queries are expected to be in given
     *  src SRS.
     */
    SrsFactors(const SrsDefinition &def
               , const SrsDefinition &src);

    struct Factors {
        double meridionalScale;
        double parallelScale;
        double angularDistortion;
        double thetaPrime;
        double convergence;
        double arealScaleFactor;
        double minScaleError;
        double maxScaleError;

        double lambdaDx;
        double phiDx;
        double lambdaDy;
        double phiDy;
    };

    Factors operator()(const math::Point2 &p) const;

    Factors operator()(const math::Point3 &p) const {
        return operator()(math::Point2(p(0), p(1)));
    }

private:
    Projection proj_;
    Projection srcProj_;
};

} // namespace geo

#endif // geo_srsdactors_hpp_included_
