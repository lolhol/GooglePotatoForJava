#include <jni.h>
#include <stdlib.h>

#include <string>

#include "../../../CartographerModule3D.h"
#include "jni_utils.h"

class CartographerModule3DJava {
 public:
  CartographerModule3DJava(std::string dir, std::string file,
                           float lidarScanTimeHz,
                           std::vector<std::string> imuNames,
                           std::vector<std::string> odomNames,
                           std::vector<std::string> rangeNames) {
    cartographer = new CartographerModule3D(dir, file,
                                            [this](const Position3D &pose) {
                                              if (pose.timestamp > position.timestamp) {
                                                position = pose;
                                              }
                                            },
                                            lidarScanTimeHz,
                                            imuNames,
                                            odomNames,
                                            rangeNames);
  }

  void addPointCloudData(PointCloudData3D data) {
    cartographer->handleLidarData(data);
  }

  void addImuData(IMUData3D data) {
    cartographer->handleImuData(data);
  }

  void addOdomData(OdomData3D data) {
    cartographer->handleOdomData(data);
  }

  void stop() {
    cartographer->stopAndOptimize();
  }

  const Position3D &getCurrentPos() { return position; }

  std::vector<std::vector<float>> getMap() { return cartographer->paintMap(); }

 private:
  CartographerModule3D *cartographer;
  Position3D position;
};

extern "C" {
JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer3D_init(
    JNIEnv *env,
    jobject thisobject,
    jstring dir,
    jstring file,
    jdouble lidarScanTimeHz,
    jobjectArray imuNames,
    jobjectArray odomNames,
    jobjectArray rangeNames
) {
  const std::string configDirStr = jstringToString(env, dir);
  const std::string configFileStr = jstringToString(env, file);
  const double lidarScanTimeHzDouble = convertJDoubleToDouble(lidarScanTimeHz);

  const std::vector<std::string> imuNamesString = convertStringArray(env, imuNames);
  const std::vector<std::string> odomNamesString = convertStringArray(env, odomNames);
  const std::vector<std::string> rangeNamesString = convertStringArray(env, rangeNames);

  const auto cartographer = new CartographerModule3DJava(configDirStr,
                                                         configFileStr,
                                                         lidarScanTimeHzDouble,
                                                         imuNamesString,
                                                         odomNamesString,
                                                         rangeNamesString);
  ptr_to_obj(env, thisobject, cartographer);
}

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer3D_addLidarData(JNIEnv *env,
                                                           jobject thisobject,
                                                           jlong timestamp,
                                                           jstring name,
                                                           jfloatArray xVals,
                                                           jfloatArray yVals,
                                                           jfloatArray zVals,
                                                           jfloatArray intencities) {
  const auto cartographer_module = (CartographerModule3DJava *) ptr_from_obj(env, thisobject);
  PointCloudData3D data(Identity(timestamp, jstringToString(env, name)));
  jsize length = env->GetArrayLength(xVals);

  jfloat *pointsXElements = env->GetFloatArrayElements(xVals, 0);
  jfloat *pointsYElements = env->GetFloatArrayElements(yVals, 0);
  jfloat *pointsZElements = env->GetFloatArrayElements(zVals, 0);
  jfloat *intencitiesElements = env->GetFloatArrayElements(intencities, 0);

  for (int i = 0; i < length; i++) {
    data.add_point(pointsXElements[i], pointsYElements[i], pointsZElements[i], intencitiesElements[i]);
  }

  env->ReleaseFloatArrayElements(xVals, pointsXElements, 0);
  env->ReleaseFloatArrayElements(yVals, pointsYElements, 0);
  env->ReleaseFloatArrayElements(zVals, pointsZElements, 0);
  env->ReleaseFloatArrayElements(intencities, intencitiesElements, 0);

  cartographer_module->addPointCloudData(data);
}

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer3D_addIMUData(JNIEnv *env,
                                                         jobject thisobject,
                                                         jlong time,
                                                         jstring name,
                                                         jfloatArray linear,
                                                         jfloatArray angular) {
  const auto cartographer_module = (CartographerModule3DJava *) ptr_from_obj(env, thisobject);

  jfloat *linearElements = env->GetFloatArrayElements(linear, 0);
  jsize linearLength = env->GetArrayLength(linear);
  float linearArray[3];
  for (int i = 0; i < linearLength; i++) {
    linearArray[i] = linearElements[i];
  }

  jfloat *angularElements = env->GetFloatArrayElements(angular, 0);
  jsize angularLength = env->GetArrayLength(angular);
  float angularArray[3];
  for (int i = 0; i < angularLength; i++) {
    angularArray[i] = angularElements[i];
  }

  float quaternion[4] = {0, 0, 0, 1};

  IMUData3D data(Identity(time, jstringToString(env, name)), linearArray, angularArray, quaternion);
  cartographer_module->addImuData(data);
}

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer3D_addOdomData(JNIEnv *env,
                                                          jobject thisobject,
                                                          jlong time,
                                                          jstring name,
                                                          jfloatArray position,
                                                          jfloatArray quaternion) {
  const auto cartographer_module = (CartographerModule3DJava *) ptr_from_obj(env, thisobject);
  jfloat *positionElements = env->GetFloatArrayElements(position, 0);
  jsize positionLength = env->GetArrayLength(position);
  float positionArray[3];
  for (int i = 0; i < positionLength; i++) {
    positionArray[i] = positionElements[i];
  }

  jfloat *quaternionElements = env->GetFloatArrayElements(quaternion, 0);
  jsize quaternionLength = env->GetArrayLength(quaternion);
  float quaternionArray[4];
  for (int i = 0; i < quaternionLength; i++) {
    quaternionArray[i] = quaternionElements[i];
  }

  OdomData3D data(Identity(time, jstringToString(env, name)),
                  positionArray[0],
                  positionArray[1],
                  positionArray[2],
                  quaternionArray);

  cartographer_module->addOdomData(data);
}

JNIEXPORT jfloatArray JNICALL
Java_brigero_cartographer4java_Cartographer3D_paintMap(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (CartographerModule3DJava *) ptr_from_obj(env, thisobject);
  std::vector<std::vector<float>> mapData = cartographer_module->getMap();

  if (mapData.size() == 0) {
    return env->NewFloatArray(0);
  }

  jfloatArray jfloat_array = env->NewFloatArray(mapData.size() * 4);

  float *body = new float[mapData.size() * 4];
  for (size_t i = 0; i < mapData.size(); ++i) {
    body[i * 4] = mapData[i][0];
    body[i * 4 + 1] = mapData[i][1];
    body[i * 4 + 2] = mapData[i][2];
    body[i * 4 + 3] = mapData[i][3];
  }

  env->SetFloatArrayRegion(jfloat_array, 0, mapData.size() * 4, body);
  delete[] body;

  return jfloat_array;
}

JNIEXPORT void JNICALL
Java_brigero_cartographer4java_Cartographer3D_stopAndOptimize(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (CartographerModule3DJava *) ptr_from_obj(env, thisobject);
  cartographer_module->stop();
}

JNIEXPORT jfloatArray JNICALL
Java_brigero_cartographer4java_Cartographer3D_getPosition(JNIEnv *env, jobject thisobject) {
  const auto cartographer_module = (CartographerModule3DJava *) ptr_from_obj(env, thisobject);
  auto position = cartographer_module->getCurrentPos();
  float arr[6] = {static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z),
                  static_cast<float>(position.yaw), static_cast<float>(position.pitch),
                  static_cast<float>(position.roll)};
  jfloatArray jfloat_array = env->NewFloatArray(6);
  env->SetFloatArrayRegion(jfloat_array, 0, 6, arr);
  return jfloat_array;
}
}