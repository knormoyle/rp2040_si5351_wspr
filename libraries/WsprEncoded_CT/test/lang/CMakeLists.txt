# Confirm that the header files build against C++11 with no errors

set(
    SOURCE_LIST
    TestLang.cpp
    TestLangDynamic.cpp
)

foreach(SOURCE IN LISTS SOURCE_LIST)
    get_filename_component(TEST_NAME ${SOURCE} NAME_WLE)

    add_executable(${TEST_NAME} ${SOURCE})
    target_include_directories(${TEST_NAME} PRIVATE ../../src)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
