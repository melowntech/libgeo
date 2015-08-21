#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "geo/srsdef.hpp"
#include "geo/srsfactors.hpp"
#include "geo/po.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Factors : public service::Cmdline
{
public:
    Factors()
        : Cmdline("geo-factors", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
        , same_(false)
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
    geo::SrsDefinition srcSrs_;
    bool same_;
};

void Factors::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("srs",  po::value(&srs_)->required()
         , "Type of source SRS definition.")
        ("srcSrs", po::value(&srcSrs_)
         , "Source SRS definition (otherwise same as --srs).")
    ;

    pd
        .add("srs", 1)
        ;

    (void) config;
}

void Factors::configure(const po::variables_map &vars)
{
    same_ = !vars.count("srcSrs");
}

bool Factors::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(geo-factors: SRS factors information
)RAW";
    }
    return false;
}

int Factors::run()
{
    auto sf(same_ ? geo::SrsFactors(srs_) : geo::SrsFactors(srs_, srcSrs_));

    std::cout << std::setprecision(15);
    math::Point2 p;
    while (std::cin >> p(0) >> p(1)) {
        auto f(sf(p));
        std::cout
            << "meridionalScale = " << f.meridionalScale << std::endl
            << "parallelScale = " << f.parallelScale << std::endl
            << "angularDistortion = " << f.angularDistortion << std::endl
            << "thetaPrime = " << f.thetaPrime << std::endl
            << "convergence = " << f.convergence << std::endl
            << "arealScaleFactor = " << f.arealScaleFactor << std::endl
            << "minScaleError = " << f.minScaleError << std::endl
            << "maxScaleError = " << f.maxScaleError << std::endl
            << "lambdaDx = " << f.lambdaDx << std::endl
            << "phiDx = " << f.phiDx << std::endl
            << "lambdaDy = " << f.lambdaDy << std::endl
            << "phiDy = " << f.phiDy << std::endl
            ;
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Factors()(argc, argv);
}
