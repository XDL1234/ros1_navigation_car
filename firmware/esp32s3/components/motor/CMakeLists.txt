file(GLOB SRC_LIST "src/*.c" "src/*.cpp")
idf_component_register(
    SRCS ${SRC_LIST}
    INCLUDE_DIRS "." "include"
    REQUIRES driver pid_ctrl bdc_motor
)