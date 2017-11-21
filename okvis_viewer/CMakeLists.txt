cmake_minimum_required(VERSION 2.8.11)
project(okvis_viewer)

# require OpenCV
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 

# require pangolin for okvis_viewer
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# nothing to build for now...
add_library(${PROJECT_NAME} STATIC src/Viewer.cc src/MapDrawer.cc)
target_link_libraries(${PROJECT_NAME}
PUBLIC ${Pangolin_LIBRARIES})
# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/okvis" COMPONENT dev
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
