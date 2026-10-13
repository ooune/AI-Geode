# Geode-independent learning core. Built into the mod and into the unit tests.
add_library(aigd_core STATIC
    src/Agent.cpp
    src/FailureAnalyzer.cpp
    src/Json.cpp
    src/Overlay.cpp
    src/Physics.cpp
    src/Predictor.cpp
    src/StepClock.cpp
    src/Storage.cpp
    src/Types.cpp
)
target_include_directories(aigd_core PUBLIC include)
target_compile_features(aigd_core PUBLIC cxx_std_20)
set_target_properties(aigd_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
