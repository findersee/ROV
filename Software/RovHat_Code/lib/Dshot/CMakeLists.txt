add_library(dshot INTERFACE)

target_include_directories(dshot
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(dshot
  INTERFACE
  hardware_pio
)

target_sources(dshot
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/dshot.c
  ${CMAKE_CURRENT_LIST_DIR}/include/dshot.h
  ${CMAKE_CURRENT_LIST_DIR}/include/commands.h
)
