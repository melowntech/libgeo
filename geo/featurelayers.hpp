/**
  * @file featurelayers.hpp
  * @author Ondrej Prochazka <ondrej.prochazka@melown.com>
  *
  * Feature layers are generic representation of geospatial 2D/3D vector data.
  * 
  * They are similar to OGC simple features (as modeled by the OGR library) with 
  * some deviations. Not all OGR geometry types are supported. We define a 
  * non-standard 'surface' geometry type for mesh-like 3D features.
  */

#ifndef vectordataset_hpp_included
#define vectordataset_hpp_included

#include "math/geometry_core.hpp"
#include "geo/srs.hpp"
#include "geo/geodataset.hpp"
#include "jsoncpp/json.hpp"


#include <algorithm>

namespace geo {


class FeatureLayers {
    
public :
    
    struct Features {
        
        typedef std::map<std::string, std::string> Properties;
        
        struct Point {
            
            std::string id;
            Properties properties;
            math::Point3 point;
            bool zDefined;

            Point( const std::string & id, const math::Point3 & point,
                   const Properties & properties, const bool zDefined )
                : id(id),  properties(properties), point(point),
                  zDefined(zDefined) {}
        };
        
        struct LineString {
            
            std::string id;
            Properties properties;
            math::Points3 points;
            bool zDefined;

            LineString( const std::string & id, const math::Points3 & points, 
                        const Properties & properties, const bool zDefined )
                : id(id), properties(properties), points(points),
                  zDefined(zDefined) {}
        };
        
        struct MultiPolygon {
            
            std::string id;
            Properties properties;
            bool zDefined;
            
            struct Polygon {
                
                math::Points3 exterior;     // exterior ring
                std::vector<math::Points3> interiors; // hole rings
            };
            
            std::vector<Polygon> polygons;

            MultiPolygon() {};
            
            MultiPolygon( const std::string & id, const Properties & properties, 
                          const std::vector<Polygon> & polygons,
                          const bool zDefined )
                : id(id), properties( properties ), zDefined(zDefined),
                  polygons(polygons) {};
        };
        
        struct Surface {
            
            struct Patch : math::Point3i {};
            typedef std::vector<uint> Boundary;
            
            std::string id;
            Properties properties;
            math::Points3 vertices;
            std::vector<Patch> surface;
            std::vector<Boundary> boundaries;
            
            Surface(const std::string & id, const Properties & properties) 
                : id(id), properties(properties) {};
                
            void addPatchesFromPolygon( 
                const Features::MultiPolygon::Polygon & polygon );
            
        private:
            
            math::Points3 triangulatePolygon( 
                const Features::MultiPolygon::Polygon & polygon );
                
        };
        
        std::vector<Point> points;
        std::vector<LineString> linestrings;
        std::vector<MultiPolygon> multipolygons;
        std::vector<Surface> surfaces;
        
        bool zAlwaysDefined; /// is z coordinate defined for all features?
        bool zNeverDefined;  /// is z coordinate defined for none of the features?
        
        Features(): zAlwaysDefined(true), zNeverDefined(true) {}
        
        void addPoint( const Point & point ) { 
            points.emplace_back( point ); 
            zAlwaysDefined &= point.zDefined; 
            zNeverDefined &= !point.zDefined;
        }

        void addLineString(const LineString & linestring) { 
            linestrings.emplace_back( linestring ); 
            zAlwaysDefined &= linestring.zDefined;
            zNeverDefined &= !linestring.zDefined;
        }

        void addMultiPolygon(const MultiPolygon & multipolygon) { 
            multipolygons.emplace_back( multipolygon ); 
            zAlwaysDefined &= multipolygon.zDefined; 
            zNeverDefined &= !multipolygon.zDefined; 
        }
        
        void addSurface(const Surface & surface) { 
            surfaces.emplace_back( surface  ); 
            zAlwaysDefined &= 1; zNeverDefined &= 0; }
    };
    
    struct Layer {
       
        std::string name;
        SrsDefinition srs;
        bool adjustVertical;
        Features features;
        boost::optional<math::Extents3> featuresBB;
        
        Layer() {}
        Layer(const std::string & name
              , const SrsDefinition & srs) : 
              name(name), srs(srs), adjustVertical(false) {}
        bool is2D() const { return features.zNeverDefined; }
        bool is3D() const { return features.zAlwaysDefined; }

        void updateBB( const math::Point3 & point );        
    };

    
    /**
     * Initialize empty
     */
    FeatureLayers() {};

    /**
     * @brief Initialize from an OGR-supported dataset
     * @param dataset pre-opened vector dataset 
     */
    FeatureLayers(::GDALDataset &dataset
        , const boost::optional<SrsDefinition> & sourceSrs = boost::none) { 
        load(dataset, sourceSrs); }
    
    /**
     * @brief Load data from an OGR-supported dataset
     * @param dataset pre-opened vector dataset
     * @param sourceSrs optional override for srs in dataset
     */
    void load(::GDALDataset &dataset
        , const boost::optional<SrsDefinition> & sourceSrs = boost::none);
    
    
    /**
     * @brief are all features in these layers three dimensional?
     */
    bool is3D() {
        return( std::all_of(layers.begin(), layers.end(), 
           [](const Layer &l) { return l.is3D(); }));
    }
    
    /**
     * @brief are all features in these layers two dimensional?
     */
    bool is2D() {
        return( std::all_of(layers.begin(), layers.end(), 
           [](const Layer &l) { return l.is2D(); }));
    }
    
    /**
     * @brief return layers bounding box
     * @param srs SRS to determine bounding box in, if none, taken from the
     *      first layer
     * @return bounding box, boost::none if empty
     */
    boost::optional<math::Extents3> boundingBox(
        boost::optional<SrsDefinition> srs = boost::none);
    
    /**
     * @brief Transform feature layers to different SRS.
     * @param targetSrs SRS to transform to.
     * 
     * If a transformation to a different datum is requested and 3D geometries 
     * are not defined, the function will complain.
     */ 
    void transform(const SrsDefinition & targetSrs);

    enum class HeightcodeMode { always, never, auto_ };
    
    /**
     * @brief Transform 2D features to 3D by adding Z coordinate from a DEM
     * @param demDataset raster dataset used as source of height information
     * @param workingSrs optional srs providing vertical dataum info for the 
     *    raster, if known. Also a hint for a suitable horizontal datum in
     *    which the demDataset should be reconstructed to obtain height. If not
     *    provided, the demDataset srs is used, this likely leads to use
     *    of ellipsoid height.
     * @param verticalAdjustment if set, vertical adjustment is performed on 
     *    the height values.
     * @param mode heightcoding mode, one of always (use DEM to override any 
     *     existing z coordinates), auto_ (use DEM to fill in missing z 
     *     coordinates), or never (never change or fill z coordinates, rendering
     *     heightcoding a noop). 
     */    
    void heightcode(const GeoDataset & demDataset
        , const boost::optional<SrsDefinition> workingSrs = boost::none
        , bool verticalAdjustment = false
        , HeightcodeMode mode = HeightcodeMode::auto_ );

    /**
     * @brief Convert 3D polygons to surfaces.
     * 
     * This step is a necessary precondition for serializing feature layers
     * with 3D polygon geometries in geodata formats.

     * The resultant vector layers do not contain any 3D polygon geometries.
     * This operation has no effect on 2D polygon geometries, which need to
     * be converted through a call to heightcode.
     */
    void convert3DPolygons();    
    
    /**
     * @brief serialize into legacy geodata format
     * @param os where to send output
     * 
     * See https://trac.citationtech.net/wiki/vadstena/PlanetGeographyFileFormat
     * for format description.
     */
    void dumpLegacyGeodata(std::ostream & os
            , const std::string & pointStyle = "point-default"
            , const std::string & linestringStyle = "linestring-default");
    
    /**
     * @brief serialize into VTS free layer geodata format.
     * @param os where to send output
     * 
     * See VTS docs for format spec.
     */
    void dumpVTSGeodata(std::ostream & os);
    
    
private :
    
    static Json::Value buildPoint3( const math::Point3 & p );
    static Json::Value buildHtml( const Features::Properties & props );    
    
    template <class Filter2, class RasterMask>
    static boost::optional<double> reconstruct( 
        const cv::Mat & from, const RasterMask & mask, 
        const math::Point2 & pos, const Filter2 & filter ); 
        
    std::vector<Layer> layers;
};


} // namespace geo


#endif // vectordataset_hpp_included