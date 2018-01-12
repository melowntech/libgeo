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

#include <ogr_spatialref.h>

#include <cstdlib>
#include <string>
#include <iostream>

#include <boost/optional.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/srsdef.hpp"
#include "geo/csconvertor.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace {

class HeightModel : public service::Cmdline
{
public:
    HeightModel()
        : Cmdline("geo-heightmodel", BUILD_TARGET_VERSION)
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map&) UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    geo::SrsDefinition srs_;
};

void HeightModel::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("srs", po::value(&srs_)->required()
         , "SRS definition.")
    ;

    pd
        .add("srs", 1)
        ;

    (void) config;
}

void HeightModel::configure(const po::variables_map &) {}

bool HeightModel::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(geo-heightmodel: report height model from SRS
)RAW";
    }
    return false;
}

int HeightModel::run()
{
    auto reference(srs_.reference());

    const auto *root(reference.GetRoot());

    std::string ellipsoid("unnamed");
    bool orthometric(false);
    std::string heightUnit("meter");

    if (const auto *datum = root->GetNode("DATUM")) {
        const auto *spheroid(datum->GetNode("SPHEROID"));
        if (const auto *spheroid0 = spheroid->GetChild(0)) {
            ellipsoid = spheroid0->GetValue();
        }
    }

    if (const auto *vertCs = root->GetNode("VERT_CS")) {
        if (const auto *vertDatum = vertCs->GetNode("VERT_DATUM")) {
            if (const auto *vertDatum0 = vertDatum->GetChild(0)) {
                orthometric = ba::icontains(vertDatum0->GetValue(), "geoid");
            }
        }

        if (const auto *unit = vertCs->GetNode("UNIT")) {
            if (const auto *unit0 = unit->GetChild(0)) {
                heightUnit = unit0->GetValue();
            }
        }
    }

    std::cout
        <<  "ellipsoid: " << ellipsoid
        << "\northometric: " << std::boolalpha << orthometric
        << "\nheightUnit: " << heightUnit
        << std::endl;

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return HeightModel()(argc, argv);
}
