include_directories(
  include/
)

# DOF_fit Hardware Interface
add_library(dof_fit_hw_interface
  src/dof_fit_hw_interface.cpp
)
target_link_libraries(dof_fit_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(dof_fit_hw_main src/dof_fit_hw_main.cpp)
target_link_libraries(dof_fit_hw_main
  dof_fit_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    dof_fit_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    dof_fit_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/dof_fit_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
