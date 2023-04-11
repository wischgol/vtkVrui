project(vtkVrui CXX)
cmake_minimum_required(VERSION 2.8)
 
find_package(VTK REQUIRED)
find_package(OpenGL)
#find_package(VRUI)
 
include(${VTK_USE_FILE})
 
include_directories(/usr/local/include/Vrui-4.2)

#set(CMAKE_C_FLAGS "-pthread -DVTK6_2")
set(CMAKE_C_FLAGS "-pthread")
#set(CMAKE_CXX_FLAGS "-pthread -DVTK6_2")
set(CMAKE_CXX_FLAGS "-pthread")
 
add_executable(${PROJECT_NAME} vtkVrui.cpp TouchNavigationTool.cpp)
 
#target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} ${OPENGL_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} /lib64/libdl.so.2 Geometry.g++-3 Misc.g++-3 Plugins.g++-3 GLMotif.g++-3 GLSupport.g++-3 Vrui.g++-3)

target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} ${OPENGL_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} Geometry.g++-3 Misc.g++-3 Plugins.g++-3 GLMotif.g++-3 GLSupport.g++-3 Vrui.g++-3)
