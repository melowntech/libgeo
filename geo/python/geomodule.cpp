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

#include <sstream>
#include <string>
#include <vector>
#include <mutex>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/call.hpp>
#include <boost/python/scope.hpp>

#include <stdint.h>

#include "dbglog/dbglog.hpp"

#include "pysupport/package.hpp"
#include "pysupport/class.hpp"
#include "pysupport/enum.hpp"

#include "../srsdef.hpp"

namespace bp = boost::python;

namespace geo { namespace py {

bp::object SrsDefinition_repr(const geo::SrsDefinition &srs)
{
    return bp::str(srs.toString());
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

        .def_readwrite("srs", &geo::SrsDefinition::srs)
        .def_readwrite("type", &geo::SrsDefinition::type)
        ;

    {
        bp::scope scope(SrsDefinition);

        pysupport::fillEnum<geo::SrsDefinition::Type>
            ("Type", "SRS type.");
    }
}

namespace geo { namespace py {
PYSUPPORT_MODULE_IMPORT(geo)
} } // namespace geo::py
