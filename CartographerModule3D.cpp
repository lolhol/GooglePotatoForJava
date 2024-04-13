//
// Created by ubuntu on 3/22/24.
//

#include "CartographerModule3D.h"

#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/io/image.h>
#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/map_builder.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>

#include "simple_grid_map.h"
#include "util/data_transform.h"
#include "util/util3D/CartoDataConvert.h"

using namespace cartographer;
using cartographer::mapping::TrajectoryBuilderInterface;
using namespace cartographer::mapping;

CartographerModule3D::CartographerModule3D(
        std::string dir, std::string file, PoseUpdateCallback3D poseUpdateCallback,
        float lidarScanTimeHz, std::vector<std::string> imuNames,
        std::vector<std::string> odomNames, std::vector<std::string> rangeNames
) : config_files_dir(dir),
    config_file_name(file),
    poseUpdateCallback(poseUpdateCallback),
    radar_scan_time(from_hz_to_sec(lidarScanTimeHz)) {
    auto currentTime = std::chrono::system_clock::now();
    auto duration = currentTime.time_since_epoch();
    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    LOG(INFO) << "Initialize 3D cartographer with configuration file " << config_files_dir << "/" << config_file_name;
    usleep(100000);
    auto file_resolver =
            absl::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{config_files_dir});
    const std::string code = file_resolver->GetFileContentOrDie(config_file_name);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    map_builder_options = mapping::CreateMapBuilderOptions(lua_parameter_dictionary.GetDictionary("map_builder").get());
    trajectory_builder_options =
            mapping::CreateTrajectoryBuilderOptions(lua_parameter_dictionary.GetDictionary("trajectory_builder").get());

    map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(map_builder_options);
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> sensor_ids;

    for (const auto &imu_name: imuNames) {
        usingIMU = true;
        sensor_ids.insert(SensorId{SensorType::IMU, imu_name});
    }

    for (const auto &odom_name: odomNames) {
        usingOdom = true;
        sensor_ids.insert(SensorId{SensorType::ODOMETRY, odom_name});
    }

    for (const auto &range_name: rangeNames) {
        sensor_ids.insert(SensorId{SensorType::RANGE, range_name});
    }

    trajectory_id = map_builder->AddTrajectoryBuilder(
            sensor_ids, trajectory_builder_options,
            [this](auto id, auto time, auto local_pose, auto range_data_in_local,
                   std::unique_ptr<const cartographer::mapping::TrajectoryBuilderInterface::InsertionResult> res) {
                // FIXME: possible error here
                map_builder->pose_graph()->GetTrajectoryNodes();
                transform::Rigid3d local2global = map_builder->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
                transform::Rigid3d pose3d = local2global * local_pose;
                this->poseUpdateCallback(Position3D(from_carto_time(time), pose3d));

                // TODO: LATER?
                /*for (auto i : res->insertion_submaps) {
                  i->ToProto(false).submap_3d().high_resolution_hybrid_grid();
                }*/
            });

    trajectory_builder = map_builder->GetTrajectoryBuilder(trajectory_id);
    if (!trajectory_builder) {
        std::cout << "Get Trajectory Builder Failed" << std::endl;
        LOG(ERROR) << "Get Trajectory Builder Failed";
    }

    pthread_mutex_init(&mutex, nullptr);
    pthread_mutex_init(&sensor_mutex, nullptr);
}

void CartographerModule3D::stopAndOptimize() {
    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0) {
        pthread_mutex_unlock(&mutex);
        return;
    }

    map_builder->FinishTrajectory(trajectory_id);
    trajectory_id = -1;
    usleep(1000000);
    map_builder->pose_graph()->RunFinalOptimization();
    pthread_mutex_unlock(&mutex);
}

int CartographerModule3D::handleImuData(IMUData3D &data) {
    if (!usingIMU) {
        return 1;
    }

    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0) {
        pthread_mutex_unlock(&mutex);
        return 1;
    }

    pthread_mutex_lock(&sensor_mutex);
    if (latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.identity.timeUS) {
        trajectory_builder->AddSensorData(data.identity.name, data.toCartoImu(startTime));
        latest_sensor_timestamp = data.identity.timeUS;
    }
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&mutex);
    return 0;
}

int CartographerModule3D::handleLidarData(PointCloudData3D &data) {
    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0) {
        pthread_mutex_unlock(&mutex);
        return 1;
    }
    // LOG(INFO)<<"Add range data at timestamp " << data.timestamp << " point count " << data.points.size(); radar_scan_time, startTime
    pthread_mutex_lock(&sensor_mutex);
    if (latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.identity.timeUS) {
        auto cartoDat = data.toTimedPointCloud(radar_scan_time, startTime);
        //std::cout << "!!! " << latest_sensor_timestamp - data.identity.timeUS << std::endl;
        trajectory_builder->AddSensorData(data.identity.name, cartoDat);
        latest_sensor_timestamp = data.identity.timeUS;
    }

    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&mutex);
    return 0;
}

int CartographerModule3D::handleOdomData(OdomData3D &data) {
    if (!usingOdom) {
        return 1;
    }

    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0) {
        pthread_mutex_unlock(&mutex);
        return 1;
    }

    pthread_mutex_lock(&sensor_mutex);
    if (latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.identity.timeUS) {
        trajectory_builder->AddSensorData(data.identity.name, data);
        latest_sensor_timestamp = data.identity.timeUS;
    }
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&mutex);
    return 0;
}

std::unordered_map<int, std::vector<std::vector<float>>>
filterSubmap(cartographer::mapping::proto::HybridGrid submap3D) {
    std::unordered_map<int, std::vector<std::vector<float>>> ret;

    for (int i = 0; i < submap3D.y_indices_size(); i++) {

    }
    return ret;
}

std::vector<std::vector<float>> CartographerModule3D::paintMap() {
    pthread_mutex_lock(&mutex);

    std::vector<std::vector<float>> point_cloud;

    std::map<SubmapId, cartographer::io::SubmapSlice> submap_slices;
    for (const auto &sub: map_builder->pose_graph()->GetAllSubmapData()) {
        const auto &submap_proto = sub.data.submap->ToProto(true).submap_3d();

        /*
        ////
        auto pose = sub.data.pose;
        auto submap_id = sub.id;
        int version = 0;

        mapping::proto::SubmapQuery::Response response_proto;
        const std::string error = map_builder->SubmapToProto(sub.id, &response_proto);

        auto first_texture = response_proto.textures().begin();
        std::string cells = first_texture->cells();
        int width = first_texture->width();
        int height = first_texture->height();
        float resolution1 = first_texture->resolution();

        transform::Rigid3d slice_pose = transform::ToRigid3(first_texture->slice_pose());
        auto pixels = io::UnpackTextureData(cells, width, height);

        cartographer::io::SubmapSlice &submap_slice = submap_slices[submap_id];
        submap_slice.pose = pose;
        submap_slice.metadata_version = 0;
        submap_slice.version = 0;
        submap_slice.width = width;
        submap_slice.height = height;
        submap_slice.slice_pose = slice_pose;
        submap_slice.resolution = resolution1;
        submap_slice.cairo_data.clear();
        submap_slice.surface = ::io::DrawTexture(
                pixels.intensity, pixels.alpha, width, height,
                &submap_slice.cairo_data);

        std::cout << width << " " << height << " " << resolution1 << " " << error << std::endl;
        ///
         */

        const auto &hybrid_grid = submap_proto.high_resolution_hybrid_grid();

        double min_probability = 0;
        double resolution = hybrid_grid.resolution();
        std::cout << "resolution: " << resolution << std::endl;
        for (int i = 0; i < hybrid_grid.x_indices_size(); i++) {
            int value = hybrid_grid.values(i);

            if (value > 32767 * min_probability) {
                int x, y, z;
                x = hybrid_grid.x_indices(i);
                y = hybrid_grid.y_indices(i);
                z = hybrid_grid.z_indices(i);

                Eigen::Vector3f point = Eigen::Vector3f(x * resolution + resolution / 2,
                                                        y * resolution + resolution / 2,
                                                        z * resolution + resolution / 2);

                int prob_int = hybrid_grid.values(i);
                point_cloud.push_back({point.x(), point.y(), point.z(),
                                       static_cast<float>(prob_int / 32767.0)});
            }
        }
    }

    pthread_mutex_unlock(&mutex);
    return point_cloud;
};