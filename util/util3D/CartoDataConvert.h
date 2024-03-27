//
// Created by ubuntu on 3/26/24.
//


#ifndef MAP_CREATOR_UTIL_UTIL3D_CARTODATACONVERT_H_
#define MAP_CREATOR_UTIL_UTIL3D_CARTODATACONVERT_H_

#include "cartographer/mapping/proto/hybrid_grid.pb.h"

std::vector<std::vector<float>> CreatePointPointCloudFromHybridGrid(
    const cartographer::mapping::proto::HybridGrid &hybrid_grid,
    double min_probability);
/*{

}*/

#endif //MAP_CREATOR_UTIL_UTIL3D_CARTODATACONVERT_H_
