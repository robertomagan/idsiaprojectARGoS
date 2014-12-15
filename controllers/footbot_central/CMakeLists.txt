add_library(footbot_central SHARED
  footbot_central.cpp
)
target_link_libraries(footbot_central inc lcm argos2_simulator_dynamic_linking)
