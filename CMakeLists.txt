# Point to root directory.
set(HSM_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "root directory")

zephyr_include_directories(include)

add_subdirectory(samples)
add_subdirectory(drivers)
add_subdirectory(subsys)
