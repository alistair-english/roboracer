cmake_minimum_required(VERSION 3.22)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(jig REQUIRED)
jig_auto_package()
