file(GLOB_RECURSE srcs "main.c" "src/*.c")

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES freertos driver nvs_flash bt
)
