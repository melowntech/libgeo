// linux specific!
#include <endian.h>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/binaryio.hpp"

#include "service/cmdline.hpp"

#include "math/geometry_core.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace {

class Asc2Gtx : public service::Cmdline {
public:
    Asc2Gtx(const std::string &name, const std::string &version)
        : service::Cmdline(name, version)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd)
        override;

    void configure(const po::variables_map &vars) override;

    int run();

    fs::path input_;
    fs::path output_;

    float nodataIn_ = -88.8888f;
    float nodataOut_ = -88.8888f;
};

void Asc2Gtx::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Input ASC file.")
        ("output", po::value(&output_)->required()
         , "Output GTX file.")

        ("nodata.in", po::value(&nodataIn_)->default_value(nodataIn_)
         , "No data value in the input file.")
        ("nodata.out", po::value(&nodataOut_)->default_value(nodataOut_)
         , "No data value in the output file.")
        ;

    pd
        .add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void Asc2Gtx::configure(const po::variables_map &vars)
{
    (void) vars;
}

struct Header {
    math::Point2 ll;
    math::Size2f pixel;
    math::Size2 size;
};

struct Grid {
    Header header;
    cv::Mat_<float> data;
};

Grid loadAsc(std::istream &is)
{
    Grid grid;
    auto &header(grid.header);
    auto &data(grid.data);

    {
        std::string h;
        std::getline(is, h);
        std::istringstream his(h);
        his.exceptions(std::ifstream::failbit);

        his >> header.ll(1) >> header.ll(0)
            >> header.pixel.height >> header.pixel.width
            >> header.size.height >> header.size.width
            ;
    };

    grid.data.create(header.size.height, header.size.width);

    for (int j(0), je(data.rows); j != je; ++j) {
        for (int i(0), ie(data.cols); i != ie; ++i) {
            is >> data(j, i);
        }
    }

    return grid;
}

Grid loadAsc(const fs::path &path)
{
    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    Grid grid;

    try {
        f.open(path.string());

        grid = loadAsc(f);
    } catch(const std::ios_base::failure &e) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot read ASC file " << path << ": " << e.what();
    }
    f.close();

    return grid;
}

namespace be {

void writeImpl(std::ostream &os, const std::uint32_t &v)
{
    const auto bev(::htobe32(v));
    os.write(reinterpret_cast<const char*>(&bev), sizeof(bev));
}

void writeImpl(std::ostream &os, const std::uint64_t &v)
{
    const auto bev(::htobe64(v));
    os.write(reinterpret_cast<const char*>(&bev), sizeof(bev));
}

void writeImpl(std::ostream &os, const float &v)
{
    writeImpl(os, reinterpret_cast<const std::uint32_t&>(v));
}

void writeImpl(std::ostream &os, const double &v)
{
    writeImpl(os, reinterpret_cast<const std::uint64_t&>(v));
}

template <typename T>
void write(std::ostream &os, const T &v)
{
    writeImpl(os, v);
}

} // namespace be

void saveGtx(std::ostream &os, const Grid &grid
             , float nodataIn, float nodataOut)
{
    const auto &header(grid.header);

    be::write<double>(os, header.ll(1));
    be::write<double>(os, header.ll(0));
    be::write<double>(os, header.pixel.height);
    be::write<double>(os, header.pixel.width);
    be::write<std::uint32_t>(os, header.size.height);
    be::write<std::uint32_t>(os, header.size.width);

    for (const auto &value : grid.data) {
        if (value == nodataIn) {
            be::write<float>(os, nodataOut);
        } else {
            be::write<float>(os, value);
        }
    }
}

void saveGtx(const fs::path &path, const Grid &grid
             , float nodataIn, float nodataOut)
{
    std::ofstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
        f.open(path.string(), std::ios_base::out | std::ios_base::trunc
                | std::ios_base::binary);
        saveGtx(f, grid, nodataIn, nodataOut);
    } catch(const std::ios_base::failure &e) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot write GTX file " << path << ": " << e.what();
    }
    f.close();
}

int Asc2Gtx::run()
{
    const auto grid(loadAsc(input_));
    saveGtx(output_, grid, nodataIn_, nodataOut_);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Asc2Gtx(BUILD_TARGET_NAME, BUILD_TARGET_VERSION)(argc, argv);
}
