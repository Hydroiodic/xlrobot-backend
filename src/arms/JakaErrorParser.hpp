#pragma once

#include <map>
#include <string>

namespace jaka_robot {

class JakaErrorParser {
  protected:
    static const std::map<int, std::string> error_map_;

  public:
    static std::string GetErrorMessage(int error_code);
};

} // namespace jaka_robot
