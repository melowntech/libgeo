/**
 * @file geotransform.hpp
 * @author Ondrej Prochazka <ondrej.prochazka@citationtech.net>
 * 
 * Georeferencing transformation, converter classes
 */

#ifndef geo_geotransform_hpp_included
#define geo_geotransform_hpp_included

#include <array>

#include "./csconvertor.hpp"

namespace geo {
  
    
/**
 * @brief A transformation defining georeferencing for a dataset
 * @details For a georeferenced dataset, geotransformation is the linear
 * transformation defining the relation between world coordinates (geographical
 * or projected) and local coordinates in the dataset (typically pixel 
 * coordinates in a raster dataset). 
 * 
 * GDAL data model defines the transformation as 6 numbers (other equivalent 
 * definitions are ESRI world files, etc). We use the same definition.
 * 
 * Note that GeoTransform has nothing to do with transformation betwen 
 * geographic coordinate systems, it merely georeferences a dataset.
 */    

class GeoTransform :  public std::array<double, 6> {

    friend class GeoDataset;
    
public :

    GeoTransform() {};

    /**
     * @brief The classic left-handed north up raster transformation.
     */
    static GeoTransform northUpFromExtents(
        const math::Extents2 & extents, const math::Size2 & size );
    
    /**
     * @brief Right handed, with zero at a given spot.
     */
    static GeoTransform localFromOrigin( const math::Point2 & origin ); 
    
    math::Point3 rowcol2geo( int row, int col, double value ) const;

    void geo2rowcol( const math::Point3 & gp, double & row, double & col ) const;

    math::Point3 raster2geo( math::Point2 p, double value ) const;

    template <typename T>
    T geo2raster( const math::Point3 & gp) const {
        double x, y; geo2rowcol(gp, y, x); return T(x, y);
    }

    template <typename T>
    T geo2raster(double gx, double gy, double gz = .0) const {
        double x, y; geo2rowcol({gx, gy, gz}, y, x); return T(x, y);
    }

    template <typename T>
    T geo2raster(const math::Point2 &gp) const {
        double x, y; geo2rowcol({gp(0), gp(1), .0}, y, x); return T(x, y);
    }
    
    template <typename T1, typename T2>
    math::Point2_<T1> convert( const math::Point2_<T2> & p ) const {
        auto ret( rwocol2geo( p(1),p(0),.0 ) );
        return math::Point2_<T1>( ret(0), ret(1) ); 
    }
    
    template <typename T1, typename T2>
    math::Point2_<T2> iconvert( const math::Point2_<T2> & gp ) const {
        double row, col;
        geo2rowcol( {gp(0),gp(1),.0}, row, col ); 
        return math::Point2_<T2>( col, row );
    }

    template <typename T1, typename T2>
    math::Point3_<T1> convert( const math::Point3_<T2> & p ) const {
        auto ret( rowcol2geo( p(1), p(0), p(2) ) );
        return math::Point3_<T1>( ret(0), ret(1), ret(2) );
    }
    
    template <typename T1, typename T2>
    math::Point3_<T2> iconvert( const math::Point3_<T2> & gp ) const {
        double row, col;
        geo2rowcol( {gp(0),gp(1),.0}, row, col ); 
        return math::Point3_<T2>( col, row, gp[2] );
    }
    
private :
    math::Point2 applyGeoTransform( double col, double row ) const;
    void applyInvGeoTransform( 
        const math::Point2 & gp, double & col, double & row ) const;
};


/**
 * @brief Convert between local coordinates and geo coordinates in a given SRS.
 * @details Useful if you want to convert from a global reference frame to 
 *  pixel coordinates and vice versa.
 */

class GeoConverter2 {
public:
    GeoConverter2( 
        const GeoTransform & srcGeo = GeoTransform(), 
        const SrsDefinition & srcSrs = SrsDefinition::longlat(),
        const SrsDefinition & dstSrs = SrsDefinition::longlat() ) :
            srcGeo_( srcGeo ), src2dst_( srcSrs, dstSrs ),
            dst2src_( dstSrs, srcSrs ) {};
    
    GeoConverter2(
        const GeoTransform & srcGeo,
        const CsConvertor & src2dst ) :
            srcGeo_( srcGeo ), src2dst_( src2dst ), 
            dst2src_( src2dst.inverse() ) {}
            
    template <typename T1 = double, typename T2 = double>
    math::Point2_<T1> convert( const math::Point2_<T2> & p ) const {
        auto ret( src2dst_(srcGeo_.convert<double>(p) ) );
        return math::Point2_<T1>(ret(0),ret(1));
    }
    
    template <typename T1 = double, typename T2 = double>
    math::Point2_<T1> iconvert( const math::Point2_<T2> & gp ) const {
        return srcGeo_.iconvert<T1>( dst2src_( math::Point2(gp(0),gp(1)) ) );
    }

    template <typename T1 = double, typename T2 = double>
    math::Point3_<T1> convert( const math::Point3_<T2> & p ) const {
        auto ret( src2dst_(srcGeo_.convert<double>(p)) );
        return math::Point3_<T1>(ret(0),ret(1),ret(2));
    }
    
    template <typename T1 = double, typename T2 = double>
    math::Point3_<T1> iconvert( const math::Point3_<T2> & gp ) const {
        return srcGeo_.iconvert<T1>( 
            dst2src_( math::Point3(gp(0),gp(1),gp(2))) );
    }
    
private:
    GeoTransform srcGeo_;
    CsConvertor src2dst_;
    CsConvertor dst2src_;
};


class GeoConverter3 {
public:
    GeoConverter3(
        const GeoTransform & srcGeo,
        const SrsDefinition & srcSrs,
        const SrsDefinition & dstSrs,
        const GeoTransform & dstGeo );
    
    GeoConverter3(
        const GeoTransform & srcGeo,
        const CsConvertor & src2dst,
        const GeoTransform & dstGeo );
    
    template <typename T1, typename T2>
    math::Point2_<T1> convert( const math::Point2_<T2> & p ) const;
    
    template <typename T1, typename T2>
    math::Point2_<T1> iconvert( const math::Point2_<T2> & p ) const;

    template <typename T1, typename T2>
    math::Point3_<T1> convert( const math::Point3_<T2> & p ) const;
    
    template <typename T1, typename T2>
    math::Point3_<T1> iconvert( const math::Point3_<T2> & p ) const;
    
private:
    GeoTransform srcGeo_, dstGeo_;
    CsConvertor src2dst_;
};


} // namespace geo

#endif // geo_geotransform_hpp_included
