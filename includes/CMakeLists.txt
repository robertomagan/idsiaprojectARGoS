add_library(inc SHARED
  thread/threadengine.cpp
  lcm/lcmthread.cpp
  lcm/lcmhandler.cpp
  beans/node.cpp
  lcm/protectedmutex.cpp
  ${orca_IMPLEMENTATIONS}
  ${hrvo_IMPLEMENTATIONS}
  ${agentvo_IMPLEMENTATIONS}
)
target_link_libraries(inc lcm argos2_simulator_dynamic_linking)
