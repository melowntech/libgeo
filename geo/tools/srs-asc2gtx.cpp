#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"

#include "service/cmdline.hpp"

#include "math/geometry_core.hpp"

#include "../srsdef.hpp"


namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace {

class SrsAsc2Gtx : public service::Cmdline {
public:
    SrsAsc2Gtx(const std::string &name, const std::string &version)
        : service::Cmdline(name, version)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd)
        override;

    void configure(const po::variables_map &vars) override;

    int run();

    geo::SrsDefinition srs_;
};

void SrsAsc2Gtx::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("srs", po::value(&srs_)->required()
         , "Input SRS definition.")
        ;

    pd
        .add("srs", 1)
        ;

    (void) config;
}

void SrsAsc2Gtx::configure(const po::variables_map &vars)
{
    (void) vars;
}

int SrsAsc2Gtx::run()
{
    auto dstSrs(geo::asc2gtx(srs_));
    std::cout << dstSrs << std::endl;

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return SrsAsc2Gtx(BUILD_TARGET_NAME, BUILD_TARGET_VERSION)(argc, argv);
}
