/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "common/strings.h"

#include <string>
#include <vector>

std::string ros2bag::strings::join(
  std::vector<std::string> const & strings,
  std::string const & delimiter,
  const std::string & prefix,
  const std::string & suffix)
{
  if (strings.empty()) {
    return "";
  }

  std::string joined = prefix;
  for (auto i = 0; i < strings.size() - 1; ++i) {
    joined += strings[i];
    joined += delimiter;
  }

  joined += strings[strings.size() - 1];
  joined += suffix;
  return joined;
}
