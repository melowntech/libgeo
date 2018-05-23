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

#include <sstream>
#include <string>
#include <vector>
#include <mutex>

#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/extract.hpp>

#include <stdint.h>

#include "dbglog/dbglog.hpp"

#include "pysupport/package.hpp"
#include "pysupport/class.hpp"
#include "pysupport/enum.hpp"

#include "../srsdef.hpp"
#include "../enu.hpp"

namespace bp = boost::python;

namespace geo { namespace py {

bp::object osr;

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

bp::object SrsDefinition_reference(const geo::SrsDefinition &srs)
{
    if (!osr) {
        LOGTHROW(err1, std::runtime_error)
            << "osgeo.osr module not found";
    }

    auto self(osr.attr("SpatialReference")());
    int status(0);

    switch (srs.type) {
    case geo::SrsDefinition::Type::proj4:
        status = bp::extract<int>(self.attr("ImportFromProj4")(srs.srs));
        break;

    case geo::SrsDefinition::Type::wkt:
        status = bp::extract<int>(self.attr("ImportFromWkt")(srs.srs));
        break;

    case geo::SrsDefinition::Type::epsg:
        status = bp::extract<int>(self.attr("SetFromUserInput")(srs.srs));
        break;

    case geo::SrsDefinition::Type::enu:
        LOGTHROW(err1, std::runtime_error)
            << "ENU SRS is not supported by OGR library.";

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Unsupported SRS type <" << srs.type << ">.";
    }

    if (status != OGRERR_NONE) {
        LOGTHROW(err1, std::runtime_error)
            << "Failed to create osgeo.osr.SpatialReference: "
            << status << ".";
    }

    return self;
}

} } // namespace geo::py

BOOST_PYTHON_MODULE(melown_geo)
{
    using namespace bp;
    namespace py = geo::py;

    // camera class
    auto SrsDefinition = class_<geo::SrsDefinition>
        ("SrsDefinition", init<geo::SrsDefinition>())
        .def(init<const std::string&>())
        .def(init<const std::string&, geo::SrsDefinition::Type>())
        .def(init<int>())
        .def(init<int, int>())
        .def("__repr__", &py::SrsDefinition_repr)

        .def("empty", &geo::SrsDefinition::empty)
        .def("as", &geo::SrsDefinition::as)
        .def("is", &geo::SrsDefinition::is)
        .def("reference", &py::SrsDefinition_reference)

        .def_readwrite("srs", &geo::SrsDefinition::srs)
        .def_readwrite("type", &geo::SrsDefinition::type)

        .def("enu", &geo::SrsDefinition::enu)

        .def("fromString", &geo::SrsDefinition::fromString)
        .staticmethod("fromString")
        .def("fromEnu", &geo::SrsDefinition::fromEnu)
        .staticmethod("fromEnu")
        ;

    {
        bp::scope scope(SrsDefinition);

        pysupport::fillEnum<geo::SrsDefinition::Type>
            ("Type", "SRS type.");
    }

    auto Enu = class_<geo::Enu>("Enu", init<geo::Enu>())
        .def(init<>())
        .def(init<const math::Point3&>())
        .def(init<const math::Point3&, const geo::SrsDefinition&>())
        .def("__repr__", &py::Enu_repr)

        .def_readwrite("lat0", &geo::Enu::lat0)
        .def_readwrite("lon0", &geo::Enu::lon0)
        .def_readwrite("h0", &geo::Enu::h0)
        .def_readwrite("spheroid", &geo::Enu::spheroid)
        .def_readwrite("towgs84", &geo::Enu::towgs84)
        ;

    {
        bp::scope scope(Enu);
        auto Spheroid = class_<geo::Enu::Spheroid>
            ("Spheroid", init<geo::Enu::Spheroid>())
            .def(init<>())
            .def(init<double>())
            .def(init<double, double>())

            .def_readwrite("a", &geo::Enu::Spheroid::a)
            .def_readwrite("b", &geo::Enu::Spheroid::b)
            ;
    }

    try {
        py::osr = import("osgeo.osr");
    } catch (error_already_set) {}
}

namespace geo { namespace py {
PYSUPPORT_MODULE_IMPORT(geo)
} } // namespace geo::py
