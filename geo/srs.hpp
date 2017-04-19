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
#ifndef geo_srs_hpp_included_
#define geo_srs_hpp_included_

#include <ogr_spatialref.h>

#include "./srsdef.hpp"

namespace geo {

/** Returns spatial reference as OGRSpatialReference
 */
OGRSpatialReference asOgrSr(const SrsDefinition &def);

/** Merges horizontal SRS from first parameter with vertival SRS from second
 * parameter and returns result.
 *
 * Both parameters must be either projected or geographic projections.
 *
 *  \param dst SRS to be modified
 *  \param src source SRS
 */
OGRSpatialReference merge(const OGRSpatialReference &horiz
                          , const OGRSpatialReference &vert);

/** Adds/replaces geoid to spatial reference.
 */
::OGRSpatialReference setGeoid(const ::OGRSpatialReference &srs
                               , const std::string &geoid);

} // namespace geo

#endif // geo_srs_hpp_included_
