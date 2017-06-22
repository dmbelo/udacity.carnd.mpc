# project(MPC)

# cmake_minimum_required (VERSION 3.5)

# # -g allows for gdb debugging
# # turn on -03 for best performance
# # add_definitions(-std=c++11 -O3 -g)
# add_definitions(-std=c++11 -g)

# set(CXX_FLAGS "-Wall")
# set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# set(sources src/Car.cpp src/MPC.cpp src/main.cpp)
# # set(sources src/Car.cpp)

# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)
# include_directories(src/Eigen-3.3)

# if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# include_directories(/usr/local/opt/openssl/include)
# link_directories(/usr/local/opt/openssl/lib)
# link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

# endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# add_executable(mpc ${sources})

# target_link_libraries(mpc ipopt z ssl uv uWS)
# # target_link_libraries(mpc ipopt python z ssl uv uWS)





project(NLP)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -g)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/Poly.cpp src/Car.cpp src/nlp_mpc.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(nlp ${sources})

target_link_libraries(nlp ipopt)