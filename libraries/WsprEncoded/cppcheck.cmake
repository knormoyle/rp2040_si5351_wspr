find_program(
    CPPCHECK
    NAMES cppcheck cppcheck.exe
)

if (CPPCHECK)
    # set flags common between CMake-run cppcheck and manually invoked target
    set(CPPCHECK_FLAGS
        # only output issues found, not just announce checking
        --quiet
        # allow for inline suppressions
        --inline-suppr
        --enable=all
        --inconclusive
        --check-level=exhaustive
        --suppress=missingInclude
        --suppress=missingIncludeSystem
        --suppress=unusedFunction
        # don't complain about suppressions that don't match against code
        --suppress=unmatchedSuppression
        --suppress=useStlAlgorithm
        --platform=native
    )

    # Run as part of the CMake build automagically against defined targets
    set(CMAKE_CXX_CPPCHECK
        "${CPPCHECK}"
        ${CPPCHECK_FLAGS}
        # bomb out of build on error if using the CMAKE-run implementation
        # --error-exitcode=1
    )

    # Add a build target so you can run by hand outside of a project "all" build
    add_custom_target(
        cppcheck 
        COMMAND "${CPPCHECK}"
        ${CPPCHECK_FLAGS}
        # ignore external libraries (this is where FetchContent goes)
        -i "${CMAKE_CURRENT_BINARY_DIR}/_deps"
        --cppcheck-build-dir="${CMAKE_CURRENT_BINARY_DIR}"
        --project="${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json"
    )
else()
    message(WARNING "Could not find cppcheck")
endif()

