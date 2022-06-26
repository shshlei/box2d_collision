add_executable(hello_world hello_world.cpp)

set_target_properties(hello_world PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
target_link_libraries(hello_world PUBLIC box2d_collision)
