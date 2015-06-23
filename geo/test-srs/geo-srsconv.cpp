#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/srsdef.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class SrsConv : public service::Cmdline
{
public:
    SrsConv()
        : Cmdline("geo-srsconv", BUILD_TARGET_VERSION
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

    geo::SrsDefinition srcSrs_;
    geo::SrsDefinition::Type dstType_;
};

void SrsConv::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("srcSrs", po::value(&srcSrs_.srs)->required()
         , "Source SRS definition.")
        ("srcType",  po::value(&srcSrs_.type)
         ->default_value(srcSrs_.type)->required()
         , "Type of source SRS definition.")
        ("dstType",  po::value(&dstType_)->required()
         , "Type of destination SRS definition.")
    ;

    pd
        .add("srcSrs", 1)
        .add("dstType", 1)
        ;

    (void) config;
}

void SrsConv::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool SrsConv::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(geo-srsconv: Spatial Reference System convertor
)RAW";
    }
    return false;
}

int SrsConv::run()
{
    std::cout << geo::SrsDefinition(srcSrs_).as(dstType_).string()
              << std::endl;
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return SrsConv()(argc, argv);
}
