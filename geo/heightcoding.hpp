#ifndef geo_heightcoding_hpp_included_
#define geo_heightcoding_hpp_included_

#include <iostream>
#include <vector>
#include <string>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "./geodataset.hpp"
#include "./vectorformat.hpp"

/** Height-coding-related stuff
 */
namespace geo { namespace heightcoding {

/** Height coding.
 */
struct Config {
    typedef std::vector<std::string> LayerNames;

    /** Complete raster DS SRS.
     *
     *  If valid, shoukld contains complete SRS, i.e. with proper vertical
     *  component.
     */
    boost::optional<SrsDefinition> rasterDsSrs;

    /** Working spatial reference system. Input data in vector dataset) are
     * expected to be in this dataset. Vector Dataset should have vertical
     * component the same as the data in input raster (although GDAL dataset
     * lacks any notion of vertical system).
     *
     * Raster datasets's SRS is used if invalid.
     */
    boost::optional<SrsDefinition> workingSrs;

    /** Spatial reference system of generated output. No conversion is performed
     *  if invalid.
     */
    boost::optional<SrsDefinition> outputSrs;

    /** Perform vertical adjustment of output data.
     */
    bool outputVerticalAdjust;

    /** List of layer names; none -> all layers present in input dataset
     */
    boost::optional<LayerNames> layers;

    /** Clipping extents (in workingSrs)
     */
    boost::optional<math::Extents2> clipWorkingExtents;

    /** Output format.
     */
    VectorFormat format;

    Config()
        : outputVerticalAdjust(false), format(VectorFormat::geodataJson) {}
};

/** Metadata of height-coded output.
 */
struct Metadata {
    /** Full 3D extents of generated output in output SRS.
     */
    math::Extents3 extents;

    /** Size of data written to the output.
     */
    std::size_t fileSize;
};

/** Height code vector data from vectorDs using height information from
 *  raster dataset rasterDs.
 *
 *  \param vectorDs input vector dataset
 *  \param rasterDs raster dataset used to height code vector data
 *  \param os output stream result is written to
 *  \param config work configuration
 */
Metadata heightCode(::GDALDataset &vectorDs, const GeoDataset &rasterDs
                    , std::ostream &os, const Config &config = Config());

/** Loads metadata from file.
 */
Metadata loadMetadata(const boost::filesystem::path &path);

/** Saves metadata to file.
 */
void saveMetadata(const boost::filesystem::path &path
                  , const Metadata &metadata);

/** Loads metadata from stream.
 */
Metadata loadMetadata(std::istream &in, const boost::filesystem::path &path
                      = "unknown");

/** Saves metadata to stream
 */
void saveMetadata(std::ostream &out , const Metadata &metadata);

} } // namespace geo::heightcoding

#endif // geo_heightcoding_hpp_included_
