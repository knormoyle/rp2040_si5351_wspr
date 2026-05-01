# demo applications that link against the lib via cmake.
# these are not "tests" in the cmake "do a bunch of tests" sense.

set(
    SOURCE_LIST
    TestAppChannelMap.cpp
    TestAppChannelMapSimple.cpp
    TestAppDecode.cpp
    TestAppDecodeSimple.cpp
    TestAppDecodeUserDefinedSimple.cpp
    TestAppDecodeVendorDefinedSimple.cpp
    TestAppEncode.cpp
    TestAppEncodeSimple.cpp
    TestAppEncodeUserDefinedSimple.cpp
    TestAppEncodeVendorDefinedSimple.cpp
    TestAppSizeUserDefined.cpp
)

foreach(SOURCE IN LISTS SOURCE_LIST)
    get_filename_component(TEST_NAME ${SOURCE} NAME_WLE)

    add_executable(${TEST_NAME} ${SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE WsprEncoded)
endforeach()