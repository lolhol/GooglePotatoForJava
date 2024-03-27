#include <jni.h>
#include <stdlib.h>

#include <string>

#include "../../../carto.h"
#include "../../../simple_grid_map.h"
#include "jni_utils.h"

class Cartographer4JavaInternal {
  CartoModule *carto;
  Position position;

 public:
  Cartographer4JavaInternal(
      const std::string &configDir,
      const std::string &configFile,
      bool use_imu,
      bool use_odom,
      double lidar_scan_time_hz) {
    // fprintf(stderr, "%d %d %f  !!!!!!!!!!!!! \n", use_imu, use_odom, lidar_scan_time_hz);

    carto = new CartoModule(
        configDir.c_str(),
        configFile.c_str(),
        [this](const Position &pose) {
          if (pose.timestamp > position.timestamp) {
            position = pose;
          }
        },
        use_imu,
        use_odom,
        lidar_scan_time_hz);
  }

  const Position &pos() { return position; }

  void handle_lidar_data(PointCloudData &data) {
    carto->handle_lidar_data(data);
  }

  void stopAndOptimize() {
    carto->stop_and_optimize();
  }

  std::vector<char> paintMap() {
    std::vector<char> map_data;
    carto->paint_map(&map_data);
    return map_data;
  }
};

extern "C" {

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer4Java_init(JNIEnv *env,
                                                      jobject thisobject,
                                                      jstring configDir,
                                                      jstring configFile,
                                                      jboolean useImu,
                                                      jboolean useOdom,
                                                      jdouble lidarScanTimeHz) {
  const std::string configDirStr = jstringToString(env, configDir);
  const std::string configFileStr = jstringToString(env, configFile);
  const bool useImuBool = convertJBooleanToBool(useImu);
  const bool useOdomBool = convertJBooleanToBool(useOdom);
  const double lidarScanTimeHzDouble = convertJDoubleToDouble(lidarScanTimeHz);

  const auto cartographer_module =
      new Cartographer4JavaInternal(configDirStr, configFileStr, useImuBool, useOdomBool, lidarScanTimeHzDouble);
  ptr_to_obj(env, thisobject, cartographer_module);
}

JNIEXPORT jfloat JNICALL
Java_brigero_cartographer4java_Cartographer4Java_posX(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (Cartographer4JavaInternal *) ptr_from_obj(env, thisobject);
  return (jfloat) cartographer_module->pos().x;
}

JNIEXPORT jfloat JNICALL
Java_brigero_cartographer4java_Cartographer4Java_angle(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (Cartographer4JavaInternal *) ptr_from_obj(env, thisobject);
  return (jfloat) cartographer_module->pos().theta;
}

JNIEXPORT jfloat JNICALL
Java_brigero_cartographer4java_Cartographer4Java_posY(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (Cartographer4JavaInternal *) ptr_from_obj(env, thisobject);
  return (jfloat) cartographer_module->pos().y;
}

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer4Java_updateLidarData(JNIEnv *env,
                                                                 jobject thisobject,
                                                                 jlong timestamp,
                                                                 jfloatArray pointsX,
                                                                 jfloatArray pointsY,
                                                                 jfloatArray intencities) {
  const auto cartographer_module = (Cartographer4JavaInternal *) ptr_from_obj(env, thisobject);
  PointCloudData data(timestamp);
  jsize length = env->GetArrayLength(pointsX);

  jfloat *pointsXElements = env->GetFloatArrayElements(pointsX, 0);
  jfloat *pointsYElements = env->GetFloatArrayElements(pointsY, 0);
  jfloat *intencitiesElements = env->GetFloatArrayElements(intencities, 0);

  for (int i = 0; i < length; i++) {
    data.add_point(pointsXElements[i], pointsYElements[i], intencitiesElements[i]);
    // printf("%.5f %.5f %.5f\n", pointsXElements[i], pointsYElements[i], intencitiesElements[i]);
  }

  env->ReleaseFloatArrayElements(pointsX, pointsXElements, 0);
  env->ReleaseFloatArrayElements(pointsY, pointsYElements, 0);
  env->ReleaseFloatArrayElements(intencities, intencitiesElements, 0);

  cartographer_module->handle_lidar_data(data);
}

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer4Java_stopAndOptimize(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (Cartographer4JavaInternal *) ptr_from_obj(env, thisobject);
  cartographer_module->stopAndOptimize();
}

JNIEXPORT jbyteArray JNICALL
Java_brigero_cartographer4java_Cartographer4Java_paintMap(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (Cartographer4JavaInternal *) ptr_from_obj(env, thisobject);
  const auto map_data = cartographer_module->paintMap();
  jbyteArray byteArray = env->NewByteArray(map_data.size());
  env->SetByteArrayRegion(byteArray, 0, map_data.size(), (const signed char *) &map_data[0]);
  return byteArray;
}
}