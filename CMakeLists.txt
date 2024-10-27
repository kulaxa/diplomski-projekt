cmake_minimum_required(VERSION 3.15)

project(Projekt)

# Set C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find necessary packages
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)


# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# Set source files
set(SOURCE_FILES
        src/main.cpp
        src/physics/physics_engine.cpp
        imgui/imgui.cpp
        imgui/imgui_draw.cpp
        imgui/imconfig.h
        imgui/imgui.h
        imgui/imgui_tables.cpp
        imgui/imgui_internal.h
        imgui/imgui_widgets.cpp
        imgui/imgui_demo.cpp
        imgui/backends/imgui_impl_glut.h
        imgui/backends/imgui_impl_glut.cpp
        imgui/backends/imgui_impl_opengl2.h
        imgui/backends/imgui_impl_opengl2.cpp
)


# Include directories
include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        imgui
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        GL GLU glut
        OpenMP::OpenMP_CXX
        ${CMAKE_THREAD_LIBS_INIT}

)

# Set compiler flags for optimization and warnings
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()


if (UNIX)
    target_link_libraries(${PROJECT_NAME} pthread)
endif (UNIX)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Set output directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
