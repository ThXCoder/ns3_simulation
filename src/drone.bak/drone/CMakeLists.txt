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
    LIBRARIES_TO_LINK 
    ${libinternet}
    ${libwifi}
    ${libapplications}
    ${libolsr}
    ${libstats}
    ${libconfig-store}
    )
