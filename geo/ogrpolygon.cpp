#include "ogrpolygon.hpp"
#include "csconvertor.hpp"

namespace geo::ogr
{

namespace fs = boost::filesystem;

MultiPolygon::MultiPolygon(const OGRMultiPolygon& polygons)
    : polygons_(polygons)
{ }

MultiPolygon::MultiPolygon(MultiPolygon&& other) noexcept
    : polygons_(std::move(other.polygons_))
{ }

const OGRMultiPolygon& MultiPolygon::polygons() const
{
    return polygons_;
}

bool MultiPolygon::overlaps(const OGRMultiPolygon& other) const
{
    return polygons_.Intersects(&other) || polygons_.Contains(&other);
}

bool MultiPolygon::overlaps(const MultiPolygon& other) const
{
    return overlaps(other.polygons_);
}

bool MultiPolygon::overlaps(const math::Extents2& other) const
{
    OGRLinearRing ring;

    math::Point2 p { ll(other) };
    ring.addPoint(p[0], p[1]);
    p = lr(other);
    ring.addPoint(p[0], p[1]);
    p = ur(other);
    ring.addPoint(p[0], p[1]);
    p = ul(other);
    ring.addPoint(p[0], p[1]);
    p = ll(other);
    ring.addPoint(p[0], p[1]);

    OGRPolygon polygon;
    polygon.addRing(&ring);
    return polygons_.Intersects(&polygon) || polygons_.Contains(&polygon);
}

bool MultiPolygon::overlaps(const math::Points2& other) const
{
    if (other.size() < 3) return false;
    OGRLinearRing ring;
    for (const auto& p : other)
    {
        ring.addPoint(p[0], p[1]);
    }
    // close the ring
    ring.addPoint(other[0][0], other[0][1]);

    OGRPolygon polygon;
    polygon.addRing(&ring);
    return polygons_.Intersects(&polygon) || polygons_.Contains(&polygon);
}

MultiPolygon MultiPolygon::from(const math::Extents2& extents)
{
    return toPolygons(extents);
}

void convert(const CsConvertor& conv, OGRPolygon& polygon)
{
    for (auto& ring : polygon)
    {
        for (OGRPoint& p : ring)
        {
            math::Point2 p_conv = conv(math::Point2 { p.getX(), p.getY() } );
            p.setX(p_conv[0]);
            p.setY(p_conv[1]);
        }
    }
}

void convert(const CsConvertor& conv, OGRMultiPolygon& polygons)
{
    for (auto& polygon : polygons)
    {
        convert(conv, *polygon);
    }
}

MultiPolygon toPolygons(const math::Extents2& extents)
{
    OGRMultiPolygon polygons;
    OGRPolygon polygon;
    OGRLinearRing ring;

    math::Point2 p { ll(extents) };
    ring.addPoint(p[0], p[1]);
    p = lr(extents);
    ring.addPoint(p[0], p[1]);
    p = ur(extents);
    ring.addPoint(p[0], p[1]);
    p = ul(extents);
    ring.addPoint(p[0], p[1]);
    p = ll(extents);
    ring.addPoint(p[0], p[1]);

    polygon.addRing(&ring);
    polygons.addGeometry(&polygon);
    return { polygons };
}

GDALDatasetUniquePtr openDataset(const fs::path& path)
{
    GDALDatasetUniquePtr dataset {
        GDALDataset::Open(path.generic_string().c_str(), GDAL_OF_VECTOR | GDAL_OF_READONLY)
    };
    if (dataset == nullptr)
    {
        const auto code(::CPLGetLastErrorNo());
        if (code == CPLE_OpenFailed)
        {
            LOGTHROW(err3, std::runtime_error)
                << "No file found for " << path << ".";
        }

        LOGTHROW(err3, std::runtime_error)
            << "Failed to open dataset " << path
            << ", errno: " << ::CPLGetLastErrorNo()
            << ", error msg: " << ::CPLGetLastErrorMsg();
    }
    return dataset;
}

MultiPolygon loadPolygons(const fs::path& path,
                          std::optional<SrsDefinition> targetSrs)
{
    LOG(info3) << "Loading polygons from: " << path;
    GDALDatasetUniquePtr dataset { openDataset(path) };
    OGRMultiPolygon polygons {};
    OGRSpatialReference* sourceSrs = nullptr;
    CsConvertor conv {};

    for (OGRLayer* layerPtr : dataset->GetLayers())
    {
        LOG(info1) << "Found layer: " << layerPtr->GetName();
        OGRSpatialReference* layerSrsPtr = layerPtr->GetSpatialRef();
        if (sourceSrs == nullptr && layerSrsPtr != nullptr)
        {
            LOG(info1) << "Using layer SRS: " << layerSrsPtr->GetName();
            sourceSrs = layerSrsPtr;
            if (targetSrs)
            {
                LOG(info3) << "Points will be converted to target SRS from: "
                           << layerSrsPtr->GetName();
                conv = CsConvertor { *layerSrsPtr, targetSrs.value() };
            }
        }
        else if (layerSrsPtr != nullptr && !sourceSrs->IsSame(layerSrsPtr))
        {
            LOGTHROW(err3, std::runtime_error)
                << "SRS discrepancy between layers.";
        }
        for (const auto& featurePtr : *layerPtr)
        {
            OGRGeometry* geometryPtr = featurePtr->GetGeometryRef();
            if (geometryPtr == nullptr) { continue; }
            else
            {
                auto geometryType = wkbFlatten(geometryPtr->getGeometryType());
                auto geometryName = geometryPtr->getGeometryName();
                LOG(info1) << "Found geometry type: " << geometryName << " ("
                           << geometryType << ").";

                if (geometryType == wkbMultiPolygon
                    || geometryType == wkbMultiPolygon25D)
                {
                    OGRMultiPolygon* multiPolygonPtr
                        = geometryPtr->toMultiPolygon();
                    if (multiPolygonPtr == nullptr)
                    {
                        LOGTHROW(err3, std::runtime_error)
                            << "Unable to cast geometry to MultiPolygon";
                    }
                    if (!multiPolygonPtr->IsValid())
                    {
                        LOGTHROW(err3, std::runtime_error)
                            << "MultiPolygon is invalid.";
                    }
                    // add all polygons to merged multipolygon
                    for (auto& polygon : *multiPolygonPtr)
                    {
                        polygons.addGeometry(polygon);
                    }
                }
                else if (geometryType == wkbPolygon
                         || geometryType == wkbPolygon25D
                         || geometryType == wkbTriangle)
                {
                    OGRPolygon* polygonPtr = geometryPtr->toPolygon();
                    if (polygonPtr == nullptr)
                    {
                        LOGTHROW(err3, std::runtime_error)
                            << "Unable to cast geometry to Polygon";
                    }
                    if (!polygonPtr->IsValid())
                    {
                        LOGTHROW(err3, std::runtime_error)
                            << "Polygon is invalid.";
                    }
                    int err = polygons.addGeometry(polygonPtr);
                    // OGRERR_NONE (0) if OK, else
                    // OGRERR_UNSUPPORTED_GEOMETRY_TYPE
                    if (err)
                    {
                        LOGTHROW(err3, std::runtime_error)
                            << "Unable to add geometry to Multipolygon "
                               "container";
                    }
                }
                else
                {
                    LOG(warn3) << "Layer (" << layerPtr->GetName()
                               << ") contains unsupported geometry: "
                               << geometryPtr->getGeometryName() << "("
                               << geometryType << ").";
                }
            }
        }
    }
    convert(conv, polygons);
    return { polygons };
}

} // namespace geo::ogr
