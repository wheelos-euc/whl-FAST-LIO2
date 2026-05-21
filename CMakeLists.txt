cmake_minimum_required(VERSION 3.16)

project(whl_fast_lio2 VERSION 0.1.0 LANGUAGES CXX)

add_library(whl_fast_lio2_core
  fast_lio2/core/fast_lio2_core.cc
  fast_lio2/core/official_pipeline_contract.cc
  fast_lio2/evaluation/scorecard.cc
  fast_lio2/map_infra/map_product_contract.cc
)

target_include_directories(whl_fast_lio2_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(whl_fast_lio2_core PUBLIC cxx_std_17)

target_compile_options(whl_fast_lio2_core PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

add_executable(core_contract_smoke
  tests/core_contract_smoke.cc
)

target_link_libraries(core_contract_smoke PRIVATE whl_fast_lio2_core)

add_executable(scorecard_contract_smoke
  tests/scorecard_contract_smoke.cc
)

target_link_libraries(scorecard_contract_smoke PRIVATE whl_fast_lio2_core)

add_executable(map_product_contract_smoke
  tests/map_product_contract_smoke.cc
)

target_link_libraries(map_product_contract_smoke PRIVATE whl_fast_lio2_core)

add_executable(official_pipeline_contract_smoke
  tests/official_pipeline_contract_smoke.cc
)

target_link_libraries(official_pipeline_contract_smoke PRIVATE whl_fast_lio2_core)
