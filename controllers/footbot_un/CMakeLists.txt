add_library(footbot_un SHARED
  footbot_un.cpp
)
target_link_libraries(footbot_un inc lcm argos2_simulator_dynamic_linking)
