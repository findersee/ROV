add_library(ads1115 INTERFACE)

target_include_directories(ads1115
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(ads1115
  INTERFACE
  hardware_i2c
)

target_sources(ads1115
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/ads1115.c
  ${CMAKE_CURRENT_LIST_DIR}/include/ads1115.h
  ${CMAKE_CURRENT_LIST_DIR}/include/registers.h
)
