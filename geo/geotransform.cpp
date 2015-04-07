/*
 * geotransform.cpp
 */

#include "./geotransform.hpp"

namespace geo {

/* class GeoTransform */

math::Point3 GeoTransform::rowcol2geo( int row, int col, double value ) const {

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
    
} // namespace geo
