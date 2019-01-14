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

        typedef std::int64_t Fid;

        struct Base {
            Fid fid;
            std::string id;
            Properties properties;

            Base() : fid() {}

            Base(Fid fid, const std::string &id, const Properties &properties)
                : fid(fid), id(id), properties(properties)
            {}
        };

        struct Point : Base {

            math::Point3 point;
            bool zDefined;

            Point(Fid fid, const std::string & id, const math::Point3 & point,
                  const Properties & properties, const bool zDefined )
                : Base(fid, id, properties), point(point),
                  zDefined(zDefined) {}
        };

        struct MultiLineString : Base {

            std::vector<math::Points3> lines;
            bool zDefined;

            MultiLineString(Fid fid, const std::string & id
                , const std::vector<math::Points3> & lines
                , const Properties & properties
                , const bool zDefined)
                    : Base(fid, id, properties), lines(lines),
                      zDefined(zDefined) {}
        };

        struct MultiPolygon : Base {

            bool zDefined;

            struct Polygon {

                math::Points3 exterior;     // exterior ring
                std::vector<math::Points3> interiors; // hole rings
            };

            std::vector<Polygon> polygons;

            MultiPolygon() : Base() {};

            MultiPolygon(Fid fid,const std::string & id
                         , const Properties & properties,
                         const std::vector<Polygon> & polygons,
                         const bool zDefined )
                : Base(fid, id, properties), zDefined(zDefined),
                  polygons(polygons) {};
        };

        struct Surface : Base {

            struct Patch : math::Point3i {};
            typedef std::vector<unsigned> Boundary;

            math::Points3 vertices;
            std::vector<Patch> surface;
            std::vector<Boundary> boundaries;

            Surface(Fid fid, const std::string & id
                    , const Properties & properties)
                : Base(fid, id, properties) {};

            void addPatchesFromPolygon(
                const Features::MultiPolygon::Polygon & polygon );

        private:

            math::Points3 triangulatePolygon(
                const Features::MultiPolygon::Polygon & polygon );

        };

        // NB: if new type of features is added update empty() function.
        std::vector<Point> points;
        std::vector<MultiLineString> multilinestrings;
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

        void addMultiLineString(const MultiLineString & multilinestring) {
            multilinestrings.emplace_back( multilinestring );
            zAlwaysDefined &= multilinestring.zDefined;
            zNeverDefined &= !multilinestring.zDefined;
        }

        void addMultiPolygon(const MultiPolygon & multipolygon) {
            multipolygons.emplace_back( multipolygon );
            zAlwaysDefined &= multipolygon.zDefined;
            zNeverDefined &= !multipolygon.zDefined;
        }

        void addSurface(const Surface & surface) {
            surfaces.emplace_back( surface  );
            zAlwaysDefined &= 1; zNeverDefined &= 0; }

        bool empty() const;

        template <typename Manipulator>
        void updateProperties(const Manipulator &manipulator);
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

        /** Layer is empty if it has no feature.
         */
        bool empty() { return features.empty(); }
    };


    /**
     * Initialize empty
     */
    FeatureLayers() {};

    /** Dataset load options.
     */
    struct LoadOptions {
        typedef std::vector<std::string> LayerNames;

        /** Overrides dataset's SRS if set . Can be used to provided SRS wtihb
         *  vertical datum since GDAL sucks at vertical datum support in
         *  datasets.
         */
        boost::optional<SrsDefinition> sourceSrs;

        /** All geometries are transformed to given given SRS if set.
         */
        boost::optional<SrsDefinition> destinationSrs;

        /** Clips all geometries by given rectangles and output only features
         *  with valid geometries. Clip is performed after any coordinate system
         *  transformation.
         */
        boost::optional<math::Extents2> clipExtents;

        /** Only named layers will be clipped if set.
         */
        boost::optional<LayerNames> clipLayers;

        /** Only named layers will be loaded if set.
         */
        boost::optional<LayerNames> layers;
    };

    /**
     * @brief Initialize from an OGR-supported dataset
     * @param dataset pre-opened vector dataset
     */
    FeatureLayers(::GDALDataset &dataset
                  , const LoadOptions &loadOptions = LoadOptions())
    {
        load(dataset, loadOptions);
    }

    /**
     * @brief Load data from an OGR-supported dataset
     * @param dataset pre-opened vector dataset
     * @param sourceSrs optional override for srs in dataset
     */
    void load(::GDALDataset &dataset
              , const LoadOptions &loadOptions = LoadOptions());


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
    void transform(const SrsDefinition & targetSrs
                   , bool verticalAdjustment = false);

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
     * @param resolution output resolution, as defined in VTS geodata spec
     *
     * See VTS docs for format spec.
     */
    void dumpVTSGeodata(std::ostream & os, const unsigned resolution = 4096);

    std::vector<Layer> layers;

private :

    template <class TPoint3>
    static Json::Value buildPoint3( const TPoint3 & p );

    static Json::Value buildHtml( const Features::Properties & props );

    template <class Filter2, class RasterMask>
    static boost::optional<double> reconstruct(
        const cv::Mat & from, const RasterMask & mask,
        const math::Point2 & pos, const Filter2 & filter );
};


UTILITY_GENERATE_ENUM_IO(geo::FeatureLayers::HeightcodeMode,
    ((always))
    ((never))
    ((auto_)("auto"))
)


// template method implementation
template <class Filter2, class RasterMask>
boost::optional<double> FeatureLayers::reconstruct(
     const cv::Mat & from, const RasterMask & mask,
     const math::Point2 & pos, const Filter2 & filter ) {

    math::Point2i ll, ur;

    ll[0] = (int) floor( pos[0] - filter.halfwinx() );
    ll[1] = (int) floor( pos[1] - filter.halfwiny() );
    ur[0] = (int) ceil( pos[0] + filter.halfwinx() );
    ur[1] = (int) ceil( pos[1] + filter.halfwiny() );

    double weightSum( 0.0 ), valueSum(0);

    for ( int i = ll[1]; i <= ur[1]; i++ )
        for ( int j = ll[0]; j <= ur[0]; j++ )
             if ( math::ccinterval( 0, (int) from.cols - 1, j ) &&
                  math::ccinterval( 0, (int) from.rows - 1, i ) &&
                  mask.get( j, i ) ) {

                 double weight = filter( j - pos[0], i - pos[1] );
                 valueSum += weight * from.at<double>(i,j);
                 weightSum += weight;
             }

    if ( weightSum > 0 )
        return boost::optional<double>( valueSum / weightSum );
    else
       return boost::none;
}

template <class TPoint3>
Json::Value FeatureLayers::buildPoint3( const TPoint3 & p ) {

    Json::Value retval = Json::arrayValue;
    retval.append(p(0)); retval.append(p(1)), retval.append(p(2));
    return retval;
}

inline bool FeatureLayers::Features::empty() const
{
    return (points.empty() && multilinestrings.empty()
            && multipolygons.empty() && surfaces.empty());
}

template <typename Manipulator>
void FeatureLayers::Features::updateProperties(const Manipulator &manipulator)
{
    for (auto &f : points) {  manipulator(f.fid, f.properties); }
    for (auto &f : multilinestrings) {  manipulator(f.fid, f.properties); }
    for (auto &f : multipolygons) {  manipulator(f.fid, f.properties); }
    for (auto &f : surfaces) {  manipulator(f.fid, f.properties); }
}

} // namespace geo

#endif // vectordataset_hpp_included
