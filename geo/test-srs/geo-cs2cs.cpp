#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/csconvertor.hpp"
#include "geo/po.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Cs2Cs : public service::Cmdline
{
public:
    Cs2Cs()
        : Cmdline("geo-cs2cs", BUILD_TARGET_VERSION
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
    geo::SrsDefinition dstSrs_;
};

void Cs2Cs::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("src", po::value(&srcSrs_)->required()
         , "Source SRS definition.")
        ("dst", po::value(&dstSrs_)->required()
         , "Destination SRS definition.")
    ;

    pd
        .add("src", 1)
        .add("dst", 1)
        ;

    (void) config;
}

void Cs2Cs::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Cs2Cs::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(geo-cs2cs: SRS convertor
)RAW";
    }
    return false;
}

int Cs2Cs::run()
{
    const geo::CsConvertor conv(srcSrs_, dstSrs_);

    double x, y, z;
    while (std::cin >> x >> y >> z) {
        const auto res(conv(math::Point3(x, y, z)));
        std::cout << std::fixed << res(0)
                  << " " << res(1)
                  << " " << res(2)
                  << std::endl;
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Cs2Cs()(argc, argv);
}
