set(SOURCE_LIST
    TestWspr.cpp
    TestWsprChannelMap.cpp
    TestWsprMessageRegularType1.cpp
    TestWsprMessageTelemetryBasic.cpp
    TestWsprMessageTelemetryCommon.cpp
    TestWsprMessageTelemetryExtendedCommon.cpp
    TestWsprMessageTelemetryExtendedCommonDynamic.cpp
    TestWsprMessageTelemetryExtendedUserDefined.cpp
    TestWsprMessageTelemetryExtendedUserDefinedDynamic.cpp
    TestWsprMessageTelemetryExtendedVendorDefined.cpp
    TestWsprMessageTelemetryExtendedVendorDefinedDynamic.cpp
    TestWsprUtl.cpp
)

foreach(SOURCE IN LISTS SOURCE_LIST)
    get_filename_component(TEST_NAME ${SOURCE} NAME_WLE)

    add_executable(${TEST_NAME} ${SOURCE})
    target_include_directories(${TEST_NAME} PRIVATE ../../src)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    target_compile_features(${TEST_NAME} PRIVATE cxx_std_20)
endforeach()
