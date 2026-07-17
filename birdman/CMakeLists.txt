cmake_minimum_required(VERSION 3.20)
project(Birdman3D CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SFML 2.6.2 の SFMLConfigDependencies はシステムOpenAL使用時に存在しない
# "OpenAL" ターゲットへ set_property するバグがあるため、先にダミーを用意する
find_package(OpenAL REQUIRED)          # OpenAL::OpenAL (sfml-audioが参照)
if(NOT TARGET OpenAL)
    add_library(OpenAL INTERFACE IMPORTED)
endif()
find_package(SFML 2.6 COMPONENTS graphics window system audio REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)

add_executable(birdman
    src/main.cpp
    src/core/Aircraft.cpp
    src/core/Physics.cpp
    src/core/Weather.cpp
    src/core/DesignIO.cpp
    src/render/Renderer3D.cpp
    src/render/Camera.cpp
    src/render/HUD.cpp
    src/ui/Widgets.cpp
    src/ui/DesignPanel.cpp
    src/ui/DesignTools.cpp
    src/ui/SettingsPanel.cpp
    src/game/Game.cpp
    src/game/SaveData.cpp
    src/game/Career.cpp
    src/game/Audio.cpp
)
target_include_directories(birdman PRIVATE src)
target_link_libraries(birdman
    sfml-graphics sfml-window sfml-system sfml-audio
    OpenGL::GL glm::glm
)
# 同梱アセット(フォント等)をexe横へコピー
add_custom_command(TARGET birdman POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:birdman>/assets)

if(WIN32)
    target_link_libraries(birdman glu32)
else()
    find_package(OpenGL REQUIRED COMPONENTS GLU)
    target_link_libraries(birdman OpenGL::GLU)
endif()

# 物理コア単体テスト(JS版との数値比較用)
add_executable(physics_test
    tests/physics_test.cpp
    src/core/Aircraft.cpp
    src/core/Physics.cpp
    src/core/Weather.cpp
)
target_include_directories(physics_test PRIVATE src)

# 機体設計 自動最適化CLI(ヘッドレス・マルチスレッド)
add_executable(optimize
    tools/optimize.cpp
    src/core/Aircraft.cpp
    src/core/Physics.cpp
    src/core/Weather.cpp
    src/core/DesignIO.cpp
)
target_include_directories(optimize PRIVATE src)
find_package(Threads REQUIRED)
target_link_libraries(optimize Threads::Threads)
