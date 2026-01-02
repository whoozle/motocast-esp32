set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(srcs main.c src/board.c src/video.c)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "./include")
