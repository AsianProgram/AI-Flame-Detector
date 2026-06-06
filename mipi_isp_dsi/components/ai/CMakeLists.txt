idf_component_register(
    SRCS "coco_detect.cpp" "ai_wrapper.cpp"
    INCLUDE_DIRS "."  # this folder
    REQUIRES esp_dl esp_mm esp_driver_cam
)
target_add_binary_data(${COMPONENT_LIB}
    "${CMAKE_SOURCE_DIR}/models/model.espdl"  # relative to ai/CMakeLists.txt
    BINARY
)