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
/*
 * geotransform.cpp
 */

#include "./geotransform.hpp"

namespace geo {

/* class GeoTransform */

math::Point3 GeoTransform::rowcol2geo( double row, double col, double value ) const {

    math::Point2 p2 = applyGeoTransform( col + 0.5, row + 0.5 );

    return math::Point3( p2[0], p2[1], value );
}

math::Point3 GeoTransform::raster2geo( math::Point2 p, double value ) const {
    
    math::Point2 p2 = applyGeoTransform( p[0] + 0.5, p[1] + 0.5 );
    return math::Point3( p2[0], p2[1], value );
}

void GeoTransform::geo2rowcol(
    const math::Point3 & gp, double & row, double & col ) const {

   math::Point2 gp2( gp[0], gp[1] );

   applyInvGeoTransform( gp2, col, row );
   row -= 0.5, col -= 0.5;
}

math::Point2 GeoTransform::applyGeoTransform( double col, double row ) const
{

    math::Point2 retval;

    retval[0] = data()[0] + col * data()[1] + row * data()[2];
    retval[1] = data()[3] + col * data()[4] + row * data()[5];

    return retval;
}

void GeoTransform::applyInvGeoTransform( const math::Point2 & gp, double & col, 
                           double & row ) const
{

    double det = data()[1] * data()[5] - data()[2] * data()[4];

    col = ( (gp[0]-data()[0])*data()[5] - (gp[1]-data()[3])*data()[2] ) / det;
    row = ( data()[1]*(gp[1]-data()[3]) - data()[4]*(gp[0]-data()[0]) ) / det;
}

GeoTransform GeoTransform::northUpFromExtents( 
    const math::Extents2 & extents, const math::Size2 & size ) {

    GeoTransform retval;
    
    retval[0] = extents.ll[0];
    retval[1] = ( extents.ur[0] - extents.ll[0] ) / size.width;
    retval[2] = 0.0;
    retval[3] = extents.ur[1];
    retval[4] = 0.0;
    retval[5] = ( extents.ll[1] - extents.ur[1] ) / size.height;
    
    return retval;
}


GeoTransform GeoTransform::localFromOrigin(
    const math::Point2 & origin ) {
    
    GeoTransform retval;
    
    retval[0] = origin(0) - 0.5; retval[1] = 1; retval[2] = 0;
    retval[3] = origin(1) - 0.5; retval[4] = 0; retval[5] = 1;
    
    return retval;
}

bool GeoTransform::isUpright() const
{
    // upright dataset has no rotation/shear
    return !operator[](2) && !operator[](4);
}

} // namespace geo
