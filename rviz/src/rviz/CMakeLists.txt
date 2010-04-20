include_directories(.)

# We create one lib with the C++...
rosbuild_add_library(rviz_renderer math/vector3.cpp
								   math/quaternion.cpp
								   render_ogre/convert.cpp 
								   render_ogre/renderer.cpp
								   render_ogre/render_window.cpp
								   render_ogre/camera.cpp
								   render_ogre/scene.cpp
								   ros/server/renderer_ros.cpp
								   ros/client/render_window.cpp
								   ros/client/scene.cpp
								   ros/client/camera_proxy.cpp
								   ros/client/camera.cpp
								   ros/client/init.cpp)
target_link_libraries(rviz_renderer ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})
rosbuild_link_boost(rviz_renderer thread signals filesystem system program_options)

rosbuild_add_executable(executable rviz_app.cpp)
target_link_libraries(executable rviz_renderer ${GTK2_LIBRARIES} ${OGRE_TOOLS_GL_LIBS} ${X11_LIBS})
set_target_properties(executable
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
                      PREFIX "")
