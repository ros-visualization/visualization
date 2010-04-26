macro(rviz_add_interface iface_file)
  if(ROSBUILD_init_called)
    message(FATAL_ERROR "rviz_add_interface() cannot be called after rosbuild_init(), please change the order in your CMakeLists.txt file.")
  endif(ROSBUILD_init_called)

  rosbuild_find_ros_package(rviz_interface_gen)
  set(get_messages_exe ${rviz_interface_gen_PACKAGE_PATH}/scripts/get_messages.py)
  set(gen_msgs_exe ${rviz_interface_gen_PACKAGE_PATH}/scripts/gen_msgs.py)
  set(gen_cpp_exe ${rviz_interface_gen_PACKAGE_PATH}/scripts/gen_cpp.py)
  set(get_cpp_outputs_exe ${rviz_interface_gen_PACKAGE_PATH}/scripts/get_cpp_outputs.py)

  # Find all the messages that will be generated for the given interface
  execute_process(COMMAND ${get_messages_exe} "${PROJECT_SOURCE_DIR}/${iface_file}" OUTPUT_VARIABLE _generated_messages OUTPUT_STRIP_TRAILING_WHITESPACE)
  separate_arguments(_generated_messages)

  execute_process(COMMAND ${get_cpp_outputs_exe} "${PROJECT_SOURCE_DIR}/${iface_file}" "${PROJECT_SOURCE_DIR}/interface_gen" 
                  OUTPUT_VARIABLE _generated_cpp OUTPUT_STRIP_TRAILING_WHITESPACE)
  separate_arguments(_generated_cpp)

  set(_msg_paths "")
  foreach(msg ${_generated_messages})
    list(APPEND _msg_paths "${PROJECT_SOURCE_DIR}/msg/${msg}")
  endforeach(msg)

  add_custom_command(
    OUTPUT ${_generated_cpp}
    COMMAND ${gen_cpp_exe} "${PROJECT_SOURCE_DIR}/${iface_file}" "${PROJECT_SOURCE_DIR}/interface_gen/cpp"
    DEPENDS ${iface_file} ${gen_cpp_exe} ${ROS_MANIFEST_LIST}
  )

  add_custom_command(
    OUTPUT ${_msg_paths}
    COMMAND ${gen_msgs_exe} "${PROJECT_SOURCE_DIR}/${iface_file}"
    DEPENDS ${iface_file} ${gen_msgs_exe} ${ROS_MANIFEST_LIST} ${_generated_cpp}
  )

  string(REPLACE "/" "_" _target_name ${iface_file})
  add_custom_target(RVIZ_INTERFACES_target_${_target_name} ALL DEPENDS ${_msg_paths})

  rosbuild_add_generated_msgs(${_generated_messages})
endmacro(rviz_add_interface)
