//
// Created by ubuntu on 3/23/24.
//

#ifndef MAP_CREATOR_UTIL_UTIL3D_IDENTITY_H_
#define MAP_CREATOR_UTIL_UTIL3D_IDENTITY_H_

#include <string>
class Identity {
 public:
  Identity(long timeUS, std::string name) {
    this->timeUS = timeUS;
    this->name = name;
  }

  long timeUS;
  std::string name;
};

#endif //MAP_CREATOR_UTIL_UTIL3D_IDENTITY_H_
