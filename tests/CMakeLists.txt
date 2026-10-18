set(MEMORYCHECK_SUPPRESSIONS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/valgrind.supp")
include(CTest)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

set(UNIT_TESTS)

function(make_test TEST_NAME)
    target_link_libraries(${TEST_NAME}
        embedded-game-controller
        GTest::gtest_main
    )
    target_compile_definitions(${TEST_NAME} PRIVATE
        -DBUILDING_EMBEDDED_GAME_CONTROLLER
    )
    target_compile_options(${TEST_NAME} PRIVATE
        -Wno-invalid-offsetof
        -Wno-overflow
    )
    gtest_discover_tests(${TEST_NAME})
    list(APPEND UNIT_TESTS ${TEST_NAME})
endfunction()

add_executable(test_timer
    test_timer.cpp
)
make_test(test_timer)

if(ENABLE_COVERAGE)
    set(COVERAGE_EXCLUDES
        "${CMAKE_SOURCE_DIR}/tests/"
        "${CMAKE_BINARY_DIR}"
    )
    setup_target_for_coverage_gcovr_html(
        NAME check-code-coverage
        OUTPUT "${PROJECT_NAME}-coverage"
        TITLE "${PROJECT_NAME} Code Coverage"
        EXECUTABLE ctest
        DEPENDENCIES ${UNIT_TESTS})
endif()
