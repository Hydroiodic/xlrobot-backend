#pragma once

#include <map>
#include <string>

namespace arms {

template <typename MapHolder> class ErrorParser {
  public:
    static std::string GetErrorMessage(int error_code) {
        auto it = MapHolder::error_map_.find(error_code);
        if (it != MapHolder::error_map_.cend()) {
            return it->second;
        } else {
            return "未知错误代码: " + std::to_string(error_code);
        }
    }
};

class JakaErrorMap {
  public:
    static const std::map<int, std::string> error_map_;
};

class TjErrorMap {
  public:
    static const std::map<int, std::string> error_map_;
};

using JakaErrorParser = ErrorParser<JakaErrorMap>;
using TjErrorParser = ErrorParser<TjErrorMap>;

} // namespace arms
