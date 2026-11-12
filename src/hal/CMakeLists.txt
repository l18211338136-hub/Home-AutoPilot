# HAL is header-only: interfaces express capabilities, not implementations.
add_library(homefly_hal INTERFACE)
target_link_libraries(homefly_hal INTERFACE homefly_include)
