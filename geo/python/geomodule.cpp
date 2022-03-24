/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include <ogr_core.h>
#include <gdal_priv.h>

#include <sstream>
#include <string>
#include <vector>
#include <mutex>

#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/handle.hpp>

#include <stdint.h>

#include "dbglog/dbglog.hpp"

#include "geo/srsdef.hpp"

#include "pysupport/package.hpp"
#include "pysupport/class.hpp"
#include "pysupport/enum.hpp"
#include "pysupport/converters.hpp"

#include "../srsdef.hpp"
#include "../enu.hpp"
#include "../coordinates.hpp"
#include "../geodataset.hpp"
#include "../gdal.hpp"

#include "geomodule.hpp"
#include "gdsblockwriter.hpp"

namespace bp = boost::python;
namespace bpc = boost::python::converter;

namespace geo { namespace py {

bp::object SrsDefinition_repr(const geo::SrsDefinition &srs)
{
    return bp::str(srs.toString());
}

bp::object Enu_repr(const geo::Enu &enu)
{
    std::ostringstream os;
    os << enu;
    return bp::str(os.str());
}

bp::object Enu_Spheroid_repr(const geo::Enu::Spheroid &s)
{
    std::ostringstream os;
    os << std::fixed << "Spheroid(" << s.a << ", " << s.b << ")";
    return bp::str(os.str());
}

bp::object SrsDefinition_reference(const geo::SrsDefinition &srs)
{
    auto osrModule(bp::import("osgeo.osr"));

    auto self(osrModule.attr("SpatialReference")());
    int status(0);

    switch (srs.type) {
    case geo::SrsDefinition::Type::proj4:
        status = bp::extract<int>(self.attr("ImportFromProj4")(srs.srs));
        break;

    case geo::SrsDefinition::Type::wkt:
        status = bp::extract<int>(self.attr("ImportFromWkt")(srs.srs));
        break;

    case geo::SrsDefinition::Type::epsg:
        status = bp::extract<int>
            (self.attr("SetFromUserInput")("EPSG:" + srs.srs));
        break;

    case geo::SrsDefinition::Type::enu:
        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS is not supported by OGR library.";
        break;

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Unsupported SRS type <" << srs.type << ">.";
        break;
    }

    if (status != OGRERR_NONE) {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to create osgeo.osr.SpatialReference: "
            << status << ".";
    }

    return self;
}

namespace {

// unpack wrapped Swig object
struct SwigSpatialReference {
    PyObject_HEAD
    OGRSpatialReference *reference;
};

} // namespace

geo::SrsDefinition
SrsDefinition_fromReference_type(const bp::object &ref
                                 , geo::SrsDefinition::Type type)
{
    auto osrModule(bp::import("osgeo.osr"));

    bp::object SpatialReference(osrModule.attr("SpatialReference"));
    if (!::PyObject_IsInstance(ref.ptr(), SpatialReference.ptr())) {
        ::PyErr_SetString
            (::PyExc_TypeError
             , "SrsDefinition.fromReference expects instance of "
             "osgeo.osr.SpatialReference.");
        bp::throw_error_already_set();
    }

    // extract
    bp::object this_(ref.attr("this"));
    auto swig(reinterpret_cast<SwigSpatialReference*>(this_.ptr()));
    return geo::SrsDefinition::fromReference(*swig->reference, type);
}

geo::SrsDefinition SrsDefinition_fromReference(const bp::object &ref)
{
    return SrsDefinition_fromReference_type
        (ref, geo::SrsDefinition::Type::proj4);
}

/** Placeholder -- actual geodataset might be exported.
 */
struct GeoDataset {};

} } // namespace geo::py

BOOST_PYTHON_MODULE(melown_geo)
{
    using namespace bp;
    namespace py = geo::py;

    // srsdef.hpp
    auto SrsDefinition = class_<geo::SrsDefinition>
        ("SrsDefinition", init<const geo::SrsDefinition&>())
        .def(init<const std::string&>())
        .def(init<const std::string&, geo::SrsDefinition::Type>())
        .def(init<int>())
        .def(init<int, int>())
        .def("__repr__", &py::SrsDefinition_repr)

        .def("empty", &geo::SrsDefinition::empty)
        .def("asType", &geo::SrsDefinition::as)
        .def("isType", &geo::SrsDefinition::is)
        .def("reference", &py::SrsDefinition_reference)

        .def_readwrite("srs", &geo::SrsDefinition::srs)
        .def_readwrite("type", &geo::SrsDefinition::type)

        .def("enu", &geo::SrsDefinition::enu)

        .def("fromString", &geo::SrsDefinition::fromString)
        .staticmethod("fromString")
        .def("fromEnu", &geo::SrsDefinition::fromEnu)
        .staticmethod("fromEnu")
        .def("fromReference", &py::SrsDefinition_fromReference)
        .def("fromReference", &py::SrsDefinition_fromReference_type)
        .staticmethod("fromReference")
        ;

    PYSUPPORT_OPTIONAL(geo::SrsDefinition);

    {
        bp::scope scope(SrsDefinition);

        pysupport::fillEnum<geo::SrsDefinition::Type>
            ("Type", "SRS type.");
    }

    // enu.hpp
    auto Enu = class_<geo::Enu>("Enu", init<geo::Enu>())
        .def(init<>())
        .def(init<const math::Point3&>())
        .def(init<const math::Point3&, const geo::SrsDefinition&>())
        .def("__repr__", &py::Enu_repr)

        .def_readwrite("lat0", &geo::Enu::lat0)
        .def_readwrite("lon0", &geo::Enu::lon0)
        .def_readwrite("h0", &geo::Enu::h0)
        .def_readwrite("towgs84", &geo::Enu::towgs84)
        ;
    pysupport::def_readwrite(Enu, "spheroid", &geo::Enu::spheroid);

    {
        bp::scope scope(Enu);
        auto Spheroid = class_<geo::Enu::Spheroid>
            ("Spheroid", init<const geo::Enu::Spheroid&>())
            .def(init<>())
            .def(init<double>())
            .def(init<double, double>())

            .def("__repr__", &py::Enu_Spheroid_repr)
            .def_readwrite("a", &geo::Enu::Spheroid::a)
            .def_readwrite("b", &geo::Enu::Spheroid::b)
            ;

        PYSUPPORT_OPTIONAL(geo::Enu::Spheroid);
    }

    // extra types, used in other modules
    pysupport::fillEnum< ::GDALDataType>
        ("GDALDataType", "GDAL data type.");
    PYSUPPORT_OPTIONAL( ::GDALDataType);

    // placeholder for GeoDataset
    auto GeoDataset = class_<py::GeoDataset>("GeoDataset", no_init)
        ;
    {
        bp::scope scope(GeoDataset);

        pysupport::fillEnum<geo::GeoDataset::Resampling>
            ("Resampling", "Enhanced GDAL resampling.");
    }

    PYSUPPORT_OPTIONAL(geo::GeoDataset::Resampling);

    // coordinates.hpp
    def<math::Matrix4(const math::Extents2&)>
        ("geo2normalized", &geo::geo2normalized);
    def<math::Matrix4(const math::Extents3&)>
        ("geo2normalized", &geo::geo2normalized);
    def<math::Matrix4(const math::Extents3&)>
        ("normalized2geo", &geo::normalized2geo);
    def<math::Matrix4(const math::Extents3&)>
        ("normalized2geo", &geo::normalized2geo);
    def("local2normalized", &geo::local2normalized);
    def<math::Matrix4(const math::Point2&)>("local2geo", &geo::local2geo);
    def<math::Matrix4(const math::Extents2&)>("local2geo", &geo::local2geo);
    def<math::Matrix4(const math::Point2&)>("geo2local", &geo::geo2local);
    def<math::Matrix4(const math::Extents2&)>("geo2local", &geo::geo2local);
}

namespace geo { namespace py {
PYSUPPORT_MODULE_IMPORT(geo)
} } // namespace geo::py
