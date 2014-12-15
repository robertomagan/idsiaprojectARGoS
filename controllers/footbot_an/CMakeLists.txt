add_library(footbot_an SHARED
  footbot_an.cpp
)
target_link_libraries(footbot_an inc lcm argos2_simulator_dynamic_linking)
