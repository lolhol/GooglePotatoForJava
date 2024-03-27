project(map_creator)

cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE Release)

find_package(cartographer REQUIRED)
find_package(Boost COMPONENTS iostreams REQUIRED)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
include_directories(/usr/lib/jvm/java-1.17.0-openjdk-amd64/include)
include_directories(/usr/lib/jvm/java-1.17.0-openjdk-amd64/include/linux)

add_compile_options(-w)
add_compile_options(-g)

file(GLOB CARTOGRAPHER4JAVA_SRC "*.cpp" "util/*.cpp" "java/brigero/cartographer4java/*.cpp")

file(GLOB CARTOGRAPHER4JAVA3D_SRC "*.cpp" "util/*.cpp" "java/brigero/cartographer4java/cartographer3D4java.cpp")
add_library(cartographer4java3D SHARED ${CARTOGRAPHER4JAVA3D_SRC}
        util/util3D/PointCloudData3D.cpp
        util/util3D/PointCloudData3D.h
        util/util3D/IMUData3D.cpp
        util/util3D/IMUData3D.h
        util/util3D/Position3D.cpp
        util/util3D/Position3D.h
        util/util3D/DataTransform3D.cpp
        util/util3D/DataTransform3D.h
        CartographerModule3D.cpp
        CartographerModule3D.h
        util/util3D/OdomData3D.cpp
        util/util3D/OdomData3D.h
        util/util3D/Identity.cpp
        util/util3D/Identity.h
        util/data_transform.cpp
        util/data_transform.h
        java/brigero/cartographer4java/cartographer3D4java.cpp
)

target_link_libraries(cartographer4java3D ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer)

add_library(cartographer4java SHARED ${CARTOGRAPHER4JAVA_SRC}
        util/util3D/PointCloudData3D.cpp
        util/util3D/PointCloudData3D.h
        util/util3D/IMUData3D.cpp
        util/util3D/IMUData3D.h
        util/util3D/Position3D.cpp
        util/util3D/Position3D.h
        util/util3D/DataTransform3D.cpp
        util/util3D/DataTransform3D.h
        CartographerModule3D.cpp
        CartographerModule3D.h
        test/Mapping3DTest.cpp
        test/Mapping3DTest.h
        util/util3D/OdomData3D.cpp
        util/util3D/OdomData3D.h
        util/util3D/Identity.cpp
        util/util3D/Identity.h
        java/brigero/cartographer4java/cartographer3D4java.cpp
        util/util3D/CartoDataConvert.cpp
        util/util3D/CartoDataConvert.h
)

target_link_libraries(cartographer4java ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer)

get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(cartographer4java PUBLIC ${INCLUDE_CARTO})
target_include_directories(cartographer4java3D PUBLIC ${INCLUDE_CARTO})

install(TARGETS cartographer4java DESTINATION /usr/lib/jni)
install(TARGETS cartographer4java3D DESTINATION /usr/lib/jni)

add_custom_target(JavaCompile
        COMMAND javac -d ../java ../java/brigero/cartographer4java/*.java
)

add_custom_target(JavaJarify
        COMMAND cd ../java && find brigero/cartographer4java -name "*.class" | xargs jar cvf ../Cartographer4Java.jar
)

add_dependencies(JavaJarify JavaCompile)
add_dependencies(cartographer4java JavaJarify)
add_dependencies(cartographer4java ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)

add_custom_target(JavaCompile3D
        COMMAND javac -d ../java ../java/brigero/cartographer4java/Cartographer3D.java
)

add_custom_target(JavaJarify3D
        COMMAND cd ../java && find brigero/cartographer4java -name "Cartographer3D.class" | xargs jar cvf ../Cartographer4Java3D.jar
)

add_dependencies(JavaJarify3D JavaCompile3D)
add_dependencies(cartographer4java3D JavaJarify3D)
add_dependencies(cartographer4java3D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)