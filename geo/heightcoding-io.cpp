#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "./heightcoding.hpp"

namespace geo { namespace heightcoding {

namespace {

void build(Json::Value &value, const Metadata &metadata)
{
    auto &extents(value["extents"] = Json::arrayValue);
    extents.append(metadata.extents.ll(0));
    extents.append(metadata.extents.ll(1));
    extents.append(metadata.extents.ll(2));
    extents.append(metadata.extents.ur(0));
    extents.append(metadata.extents.ur(1));
    extents.append(metadata.extents.ur(2));

    value["fileSize"] = int(metadata.fileSize);
}

void parse(Metadata &metadata, const Json::Value &value)
{
    const auto &extents(Json::check(value["extents"], Json::arrayValue));
    metadata.extents.ll(0) = extents[0].asDouble();
    metadata.extents.ll(1) = extents[1].asDouble();
    metadata.extents.ll(2) = extents[2].asDouble();
    metadata.extents.ur(0) = extents[3].asDouble();
    metadata.extents.ur(1) = extents[4].asDouble();
    metadata.extents.ur(2) = extents[5].asDouble();

    Json::get(metadata.fileSize, value, "fileSize");
}

} // namespace

void saveMetadata(std::ostream &out, const Metadata &metadata)
{
    Json::Value content;
    build(content, metadata);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

Metadata loadMetadata(std::istream &in
                      , const boost::filesystem::path &path)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to parse heightcoding::Metadata file " << path << ": "
            << reader.getFormattedErrorMessages() << ".";
    }

    Metadata metadata;
    parse(metadata, content);
    return metadata;
}

Metadata loadMetadata(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading heightcoding::Metadata from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to load heightcoding::Metadata from " << path << ".";
    }
    auto metadata(loadMetadata(f, path));
    f.close();
    return metadata;
}

void saveMetadata(const boost::filesystem::path &path
                  , const Metadata &metadata)
{
    LOG(info1) << "Saving heightcoding::Metadata into " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to save heightcoding::Metadata into "
            << path << ".";
    }
    saveMetadata(f, metadata);
    f.close();
}

} } // namespace geo::heightcoding
