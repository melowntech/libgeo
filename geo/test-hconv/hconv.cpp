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
#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/srsdef.hpp"
#include "geo/heightconvertor.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class HConv : public service::Cmdline
{
public:
    HConv()
        : Cmdline("hconv", BUILD_TARGET_VERSION)
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

    geo::SrsDefinition srs_;
    fs::path geodDir_;
    bool toEllipsoid_;
};

void HConv::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("geodDir", po::value(&geodDir_)->required()
         , "Path to directory with geoid data (e.g. egm96-5.pgm).")
        ("srs", po::value<std::string>()->required()
         , "SRS definition (autodetected, either proj4 or wkt)")
        ("toEllipsoid", po::value(&toEllipsoid_)->default_value(false)
         ->implicit_value(true)->required()
         , "Convert from geoid to ellipsoid instead from ellipsoid to geoid.")
    ;

    (void) pd;
    (void) config;
}

void HConv::configure(const po::variables_map &vars)
{
    auto srs(vars["srs"].as<std::string>());
    if (srs.empty()) {
        throw po::validation_error
            (po::validation_error::invalid_option_value, srs, "srs");
    }
    if (srs[0] == '+') {
        srs_ = { srs, geo::SrsDefinition::Type::proj4 };
    } else {
        srs_ = { srs, geo::SrsDefinition::Type::wkt };
    }
}

bool HConv::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(hconv: height convertor
usage
    hconv --srs=SRS

    Points are read from stdin in format "easting northing height"
)RAW";
    }
    return false;
}

int HConv::run()
{
    geo::HeightConvertor hc(srs_, geodDir_
                            , (toEllipsoid_
                               ? geo::HeightConvertor::Direction::toEllipsoid
                               : geo::HeightConvertor::Direction::toGeoid));

    std::cout << std::setprecision(15);
    double x, y, z;
    while (std::cin >> x >> y >> z) {
        std::cout
            << x << " " << y << " "
            << hc(math::Point3(x, y, z))(2) << std::endl;
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return HConv()(argc, argv);
}
