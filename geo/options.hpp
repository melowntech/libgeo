/**
 * Copyright (c) 2023 Melown Technologies SE
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

/**
  * @file options.hpp
  * @author Vaclav Blazek <vaclav.blazek@melowntech.com>
  */

#ifndef geo_options_hpp_included
#define geo_options_hpp_included

#include <utility>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>

namespace geo {

/** Various options. Thin wrapper around vector of string pairs.
 *
 *  For create-options see:
 *    * http://www.gdal.org/frmt_gtiff.html for gtiff options
 *    * http://www.gdal.org/frmt_various.html#PNG for png options
 *    * http://www.gdal.org/frmt_jpeg.html for jpeg options
 *
 */
struct Options {
    typedef std::pair<std::string, std::string> Option;
    typedef std::vector<Option> OptionList;
    OptionList options;

    Options() = default;

    template <typename T>
    Options(const std::string &name, const T &value) {
        operator()(name, value);
    }

    template <typename T>
    Options& operator()(const std::string &name, const T &value) {
        options.emplace_back
            (name, boost::lexical_cast<std::string>(value));
        return *this;
    }

    /** Special handling for boolean -> YES/NO
     */
    Options& operator()(const std::string &name, bool value) {
        options.emplace_back(name, value ? "YES" : "NO");
        return *this;
    }

    bool empty() const { return options.empty(); }
};


} // namespace geo

#endif // geo_options_hpp_included
