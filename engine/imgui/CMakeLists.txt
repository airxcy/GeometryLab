#imgui projects with no CMakeList.txt
project(imgui)
file(GLOB IMGUI_SOURCES  ./*.cpp platform/*.cpp ./*.h platform/*.h)
add_library(imgui ${IMGUI_SOURCES})
target_compile_definitions(imgui PUBLIC -DIMGUI_IMPL_OPENGL_LOADER_GLAD)
target_include_directories(imgui PUBLIC ./ platform)
target_link_libraries(imgui PRIVATE glad glfw)