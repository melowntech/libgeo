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

#include <math/geometry_core.hpp>
#include <geo/srs.hpp>
#include <geo/geodataset.hpp>


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
            std::vector<math::Points3> vertices;
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
        
        Features(): zAlwaysDefined(true) {}
        
        void addPoint( const Point & point ) { 
            points.emplace_back( point ); 
            zAlwaysDefined &= point.zDefined; }

        void addLineString(const LineString & linestring) { 
            linestrings.emplace_back( linestring ); 
            zAlwaysDefined &= linestring.zDefined; }

        void addMultiPolygon(const MultiPolygon & multipolygon) { 
            multipolygons.emplace_back( multipolygon ); 
            zAlwaysDefined &= multipolygon.zDefined; }
        
        void addSurface(const Surface & surface) { 
            surfaces.emplace_back( surface  ); 
            zAlwaysDefined &= 1; }
    };
    
    struct Layer {
       
        std::string name;
        SrsDefinition srs;
        Features features;
        boost::optional<math::Extents3> featuresBB;
        
        Layer(const std::string & name
              , const SrsDefinition & srs) : name(name), srs(srs) {};
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
        , boost::optional<const SrsDefinition &> sourceSrs = boost::none) { 
        load(dataset, sourceSrs); }
    
    /**
     * @brief Load data from an OGR-supported dataset
     * @param dataset pre-opened vector dataset
     * @param sourceSrs optional override for srs in dataset
     */
    void load(::GDALDataset &dataset
        , boost::optional<const SrsDefinition &> sourceSrs = boost::none);
    
    /**
     * @brief Transform feature layers to different SRS.
     * @param targetSrs SRS to transform to.
     * 
     * This may fail if 3D transformation to a different datum is requested
     * and 3D geometries are not defined.
     */ 
    void transform(const SrsDefinition & targetSrs);

    enum class HeightcodeMode { always, never, auto_ };
    
    /**
     * @brief Transform 2D features to 3D by adding Z coordinate from a DEM
     * @param demDataset raster dataset used as source of height information
     * @param workingSrs optional srs providing vertical dataum info fro the 
     *    raster, if known, also a hint for a suitable horizontal datum in
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
        , boost::optional<SrsDefinition> workingSrs = boost::none
        , bool verticalAdjustment = false
        , HeightcodeMode mode = HeightcodeMode::auto_ );

    /**
     * @brief Convert 3D polygons to surfaces.
     * 
     * This step is a necessary precondition for serializing geodata formats.

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
    void dumpLegacyGeodata(std::ostream & os);
    
    /**
     * @brief serialize into VTS free layer geodata format.
     * @param os where to send output
     * 
     * See VTS docs for format spec.
     */
    void dumpVTSGeodata(std::ostream & os);
    
    
private :
    std::vector<Layer> layers;
};


} // namespace geo


#endif // vectordataset_hpp_included