set(TARGET locateBalloon)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Eigen3 3.3 NO_MODULE)
find_package(Boost REQUIRED COMPONENTS program_options)

file(GLOB HEADER_FILES ./*.h)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(${TARGET}
        structurecomputer.cc
        balloonfinder.cc
        imagemetadata.cc
        sensorparams.cc   
        main.cc
        ${HEADER_FILES})
target_link_libraries(${TARGET} navtoolbox ${OpenCV_LIBS} Eigen3::Eigen ${Boost_LIBRARIES})

set_target_properties(${TARGET}
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/exe"
)

