include_directories(.)

rosbuild_add_executable(executable rviz_app.cpp)
target_link_libraries(executable ${GTK2_LIBRARIES} ${OGRE_TOOLS_GL_LIBS} ${X11_LIBS})
set_target_properties(executable
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
                      PREFIX "")
