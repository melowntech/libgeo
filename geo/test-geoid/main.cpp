#include <cstdlib>
#include <string>
#include <iostream>

#include <boost/optional.hpp>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/srsdef.hpp"
#include "geo/csconvertor.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Geoid : public service::Cmdline
{
public:
    Geoid()
        : Cmdline("geo-geoid", BUILD_TARGET_VERSION)
    {
    }

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
    std::string geoidGrid_;
    boost::optional<geo::SrsDefinition::Type> outputType_;
};

void Geoid::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("srs", po::value(&srs_)->required()
         , "SRS definition.")
        ("geoidGrid", po::value(&geoidGrid_)->required()
         , "Geoid grid.")
        ("type", po::value<geo::SrsDefinition::Type>()
         , "Type of output SRS definition (proj4 or wkt). "
         "Defaults to input srs type.")
    ;

    pd
        .add("srs", 1)
        .add("geoidGrid", 1)
        ;

    (void) config;
}

void Geoid::configure(const po::variables_map &vars)
{
    if (vars.count("type")) {
        outputType_ = vars["type"].as<geo::SrsDefinition::Type>();
    }
}

bool Geoid::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(geo-geoid: add geoid grid to SRS's vertical datum
usage
    geo-geoid SRS geoid
)RAW";
    }
    return false;
}

int Geoid::run()
{
    auto srs(geo::setGeoid(srs_, geoidGrid_));

    if (outputType_) { srs = srs.as(*outputType_); }

    std::cout << srs << std::endl;

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Geoid()(argc, argv);
}
