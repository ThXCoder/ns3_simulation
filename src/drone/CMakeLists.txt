check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/drone-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME drone
    SOURCE_FILES model/normal-drone-app.cpp
                 model/Malicious.cpp
                 model/Satellite-app.cpp
                 helper/normal-drone-helper.cpp
                 helper/MaliciousDroneHelper.cpp
                 helper/SatelliteHelper.cpp

                 
    HEADER_FILES model/normal-drone-app.h
                 model/Malicious.h
                 model/Satellite-app.h
                 helper/normal-drone-helper.h
                 helper/MaliciousDroneHelper.h
                 helper/SatelliteHelper.h
    LIBRARIES_TO_LINK ${libcore}
    TEST_SOURCES test/drone-test-suite.cc
                 ${examples_as_tests_sources}
)
