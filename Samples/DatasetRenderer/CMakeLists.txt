cmake_minimum_required(VERSION 3.19)
project(DatasetRenderer)

# Model importers are shared with the Editor (they are not part of the engine library):
set(EDITOR_DIR ${WICKED_ROOT_DIR}/Editor)

add_executable(DatasetRenderer
    main_SDL2.cpp
    ${EDITOR_DIR}/ModelImporter_GLTF.cpp
    ${EDITOR_DIR}/ModelImporter_FBX.cpp
    ${EDITOR_DIR}/ModelImporter_OBJ.cpp
)

target_include_directories(DatasetRenderer PRIVATE ${EDITOR_DIR})

target_link_libraries(DatasetRenderer PUBLIC
    WickedEngine
)

# Default shader cache location (precompiled SPIR-V shaders are reused from here):
target_compile_definitions(DatasetRenderer PRIVATE
    DATASET_RENDERER_SHADER_DIR="${WICKED_ROOT_DIR}/shaders/"
)

if(WICKED_ENABLE_IPO)
set_target_properties(DatasetRenderer PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON
    INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF
)
endif()

set(LIB_DXCOMPILER "libdxcompiler.so")
set(LIBDXCOMPILER_PATH "${WICKED_ROOT_DIR}/WickedEngine/${LIB_DXCOMPILER}")

add_custom_command(
    TARGET DatasetRenderer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIBDXCOMPILER_PATH} ${CMAKE_CURRENT_BINARY_DIR}
)
