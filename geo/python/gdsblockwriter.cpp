/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include <memory>

#include <pysupport/boost-python-definitions.hpp>

#include <Python.h>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/call.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <opencv2/core/core.hpp>

#include <stdint.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"

#include "pysupport/package.hpp"
#include "pysupport/class.hpp"
#include "pysupport/enum.hpp"
#include "pysupport/string.hpp"
#include "pysupport/threads.hpp"

#include "../gdsblockwriter.hpp"
#include "../gdal.hpp"

namespace bp = boost::python;

namespace geo { namespace py {

void* importNumpy()
{
    if (!PyArray_API) {
        import_array();
        if (!PyArray_API) {
            throw bp::error_already_set();
        }
    }
    return nullptr;
}

struct Tile {
    bp::object data;
    math::Point2i index;
};

::GDALDataType gdalType(char type)
{
    switch (type) {
    case 'b': return GDT_Byte;
    case 'B': return GDT_Byte;
    case 'h': return GDT_Int16;
    case 'H': return GDT_UInt16;
    case 'i': return GDT_Int32;
    case 'u': return GDT_Int32;
    case 'f': return GDT_Float32;
    case 'd': return GDT_Float64;
    }

    LOGTHROW(err1, std::logic_error)
        << "Unsupported array data type '" << type << "'.";
    throw;
}

void add(GdsBlockWriter::Batch &batch, const math::Point2i &index
         , PyArrayObject *array)
{
    if (PyArray_NDIM(array) != 3) {
        LOGTHROW(err1, std::logic_error)
            << "Block must be a 3D array; provided array has "
            << PyArray_NDIM(array) << " dimension.";
    }

    const auto dtype(PyArray_DTYPE(array));

    batch.blocks.emplace_back(index, gdalType(dtype->type)
                              , PyArray_DATA(array));
    auto &block(batch.blocks.back());

    block.size.width = PyArray_DIM(array, 1);
    block.size.height = PyArray_DIM(array, 0);
    block.channels = PyArray_DIM(array, 2);

    const auto *strides(PyArray_STRIDES(array));
    block.lineSpace = strides[0];
    block.pixelSpace = strides[1];
    block.bandSpace = strides[2];

    switch (dtype->byteorder) {
    case '=': case '|': break;
    default:
        LOGTHROW(err1, std::logic_error)
            << "Only native byte order is supported.";
    }
}

void add(GdsBlockWriter::Batch &batch, const math::Point2i &index
         , const bp::object &data)
{
    // TODO: add opencv support
    if (PyArray_Check(data.ptr())) {
        add(batch, index
            , reinterpret_cast<PyArrayObject*>(data.ptr()));
    } else {
        LOGTHROW(err1, std::logic_error)
            << "GdsBlockWriter supports only numpy array, not "
            << data.ptr()->ob_type->tp_name << ".";
    }
}

void GdsBlockWriter_write(GdsBlockWriter &writer, const bp::object &data)
{
    GdsBlockWriter::Batch batch;

    for (bp::stl_input_iterator<bp::object> idata(data), edata;
         idata != edata; ++idata)
    {
        const auto &item(*idata);

        add(batch, math::Point2i(bp::extract<int>(item[0])
                                 , bp::extract<int>(item[1]))
            , item[2]);
    }

    auto obj(data.ptr());

    batch.acquire = [obj] { Py_INCREF(obj); };

    batch.release = [obj]() {
        pysupport::EnablePython guard;
        Py_DECREF(obj);
    };

    writer.write(batch);
}

void GdsBlockWriter_finish(GdsBlockWriter &writer)
{
    pysupport::EnableThreads guard;
    writer.finish();
}

bp::object GdsBlockWriter_enter(const bp::object &self)
{
    return self;
}

void GdsBlockWriter_exit(GdsBlockWriter &writer, const bp::object&
                         , const bp::object&, const bp::object&)
{
    // close in any case
    GdsBlockWriter_finish(writer);
}

template <typename T>
std::shared_ptr<std::vector<T>> vectorFromSequence(const bp::object& sequence)
{
    auto v(std::make_shared<std::vector<T>>());
    for (bp::stl_input_iterator<T> isequence(sequence), esequence;
         isequence != esequence; ++isequence)
    {
        v->push_back(*isequence);
    }
    return v;
}

template <typename T>
void addVector(const char *type)
{
    using namespace bp;

    class_<std::vector<T>>(type)
        .def(init<>())
        .def(vector_indexing_suite<std::vector<T>>())
        .def("__init__", make_constructor(&vectorFromSequence<T>))
        ;
}

Options dictToOptions(const bp::dict &dict)
{
    Options opt;
    for (bp::stl_input_iterator<bp::str> idict(dict), edict;
         idict != edict; ++idict)
    {
        // TODO: convert value from number/boolean
        opt(pysupport::py2utf8(*idict), pysupport::py2utf8(dict[*idict]));
    }
    return opt;
}

struct from_python_options
{
    typedef bp::converter::rvalue_from_python_stage1_data Data;
    typedef bp::converter::rvalue_from_python_storage<
        Options> Storage;

    from_python_options() {
        bp::converter::registry::push_back
            (&convertible, &construct, bp::type_id<Options>());
    }

    // Determine if ptr can be converted in an Optional
    static void* convertible(::PyObject *ptr) {
        return PyDict_Check(ptr) ? ptr : nullptr;
    }

    // Convert ptr into a Options
    static void construct(::PyObject *ptr, Data *data) {
        // get memory
        void *storage(((Storage*) data)->storage.bytes);

        new (storage) Options(dictToOptions(bp::dict(bp::handle<>(ptr))));

        // Stash the memory chunk pointer for later use by boost.python
        data->convertible = storage;
    }
};

void registerGdsBlockWriter() {
    importNumpy();

    using namespace bp;

    auto GdsBlockWriter_class = class_<GdsBlockWriter, boost::noncopyable>
        ("GdsBlockWriter"
         , init<const boost::filesystem::path&, const math::Size2&
         , const GeoDataset::Format&, const math::Size2&
         , const NodataValue&, const Options&>())
        .def("write", &GdsBlockWriter_write)
        .def("finish", &GdsBlockWriter_finish)
        .def("__del__", &GdsBlockWriter_finish)
        .def("__enter__", &GdsBlockWriter_enter)
        .def("__exit__", &GdsBlockWriter_exit)
        ;

    pysupport::fillEnum< ::GDALDataType>
        ("GDALDataType", "GDAL data type.");
    addVector< ::GDALDataType>("GDALDataTypes");
    pysupport::fillEnum< ::GDALColorInterp>
        ("GDALColorInterp", "GDAL color interpretation.");
    addVector< ::GDALColorInterp>("GDALColorInterps");

    auto Format_class = class_<GeoDataset::Format>
        ("Format", init<>())
        .def(init< ::GDALDataType
             , const std::vector< ::GDALColorInterp>&>())
        .def(init< ::GDALDataType
             , const std::vector< ::GDALColorInterp>&
             , GeoDataset::Format::Storage>())
            .def_readwrite("channelType", &GeoDataset::Format::channelType)
        .def_readwrite("channels", &GeoDataset::Format::channels)
        .def_readwrite("driver", &GeoDataset::Format::driver)

        .def("gtiffRGBPhoto", &GeoDataset::Format::gtiffRGBPhoto)
        .staticmethod("gtiffRGBPhoto")
        .def("pngRGBPhoto", &GeoDataset::Format::pngRGBPhoto)
        .staticmethod("pngRGBPhoto")
        .def("pngRGBAPhoto", &GeoDataset::Format::pngRGBAPhoto)
        .staticmethod("pngRGBAPhoto")
        .def("jpegRGBPhoto", &GeoDataset::Format::jpegRGBPhoto)
        .staticmethod("jpegRGBPhoto")
            .def("coverage", &GeoDataset::Format::coverage)
        .staticmethod("coverage")
        .def("dsm", &GeoDataset::Format::dsm)
        .staticmethod("dsm")
        ;

    // inject GeoDataset::Format::Storage enum
    {
        bp::scope scope(Format_class);
        pysupport::fillEnum<GeoDataset::Format::Storage>
            ("Storage", "Storage type.");
    }

    from_python_options();

    // make sure we have thread support
#if PY_VERSION_HEX < 0x03090000
    ::PyEval_InitThreads();
#endif

    dbglog::log_thread(true);
}

} } // namespace geo::py
