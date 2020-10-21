# Basic system
add_subdirectory(framestore)
add_subdirectory(choreographer)
add_subdirectory(mapper)

# Planners
add_subdirectory(planner_trapezoidal)
if (ENABLE_QP)
  add_subdirectory(planner_qp)
endif()
if (ENABLE_SCP)
  add_subdirectory(planner_scp)
endif()

# High-level tools
add_subdirectory(mobility)
