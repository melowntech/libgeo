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
#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/geodataset.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class GeoInfo : public service::Cmdline
{
public:
    GeoInfo()
        : Cmdline("geo-info", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path path_;
};

void GeoInfo::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("path",  po::value(&path_)->required()
         , "Geo dataset path.")
    ;

    pd
        .add("path", 1)
        ;

    (void) config;
}

void GeoInfo::configure(const po::variables_map&) {}

bool GeoInfo::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(geo-info: geo dataset information
)RAW";
    }
    return false;
}

int GeoInfo::run()
{
    auto ds(geo::GeoDataset::open(path_));

    std::cout << "files:";
    for (const auto &file : ds.files()) {
        std::cout << "\t" << file.string() << "\n";
    }

    auto des(ds.descriptor());
    std::cout
        << std::fixed
        << "driver: " << des.driverName
        << "\nextents: " << des.extents
        << "\nsize: " << des.size
        << "\nresolution: " << des.resolution(0) << " " << des.resolution(1)
        << "\nsrs: " << des.srs.as(geo::SrsDefinition::Type::proj4)
        ;

    std::cout << std::endl;

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return GeoInfo()(argc, argv);
}
