#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/srsdef.hpp"
#include "geo/csconvertor.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class CsSonv : public service::Cmdline
{
public:
    CsSonv()
        : Cmdline("csconv", BUILD_TARGET_VERSION)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map&) UTILITY_OVERRIDE {}

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    geo::SrsDefinition src_;
    geo::SrsDefinition dst_;
};

void CsSonv::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("src", po::value(&src_)->required()
         , "Source SRS definition.")
        ("dst", po::value(&dst_)->required()
         , "Destination SRS definition.")
    ;

    pd
        .add("src", 1)
        .add("dst", 1)
        ;

    (void) config;
}

bool CsSonv::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(csconv: coordinate system to coordinate system  convertor
usage
    csconv src-SRS dst-SRS [point point ...]

    Points are read from stdin in format x y.
)RAW";
    }
    return false;
}

int CsSonv::run()
{
    geo::CsConvertor cs(src_, dst_);

    std::cout << std::setprecision(15);
    math::Point3 p;
    while (std::cin >> p(0) >> p(1) >> p(2)) {
        auto p2(cs(p));
        std::cout << p2(0) << ' ' << p2(1) << ' ' << p2(2) <<std::endl;
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return CsSonv()(argc, argv);
}
