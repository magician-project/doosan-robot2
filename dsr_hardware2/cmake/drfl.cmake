find_package(Poco REQUIRED Foundation Net)
message(STATUS "Poco found at: ${Poco_DIR}")

include(FetchContent)
set(DSR_PATH ${CMAKE_CURRENT_BINARY_DIR}/API-DRFL)
set(DSR_OS "Linux")
set(DSR_ARCH "64bits")
set(DSR_VERSION "22.04")

FetchContent_Declare(
    DoosanApiRepo
    URL https://github.com/doosan-robotics/API-DRFL/archive/refs/heads/main.zip
    SOURCE_DIR ${DSR_PATH}
)
if(NOT DoosanApiRepo_POPULATED)
    FetchContent_Populate(DoosanApiRepo)
endif()

message(STATUS "API-DRFL Doosan repository location: ${DSR_PATH}")
message(STATUS "API-DRFL OS: ${DSR_OS}")
message(STATUS "API-DRFL architecture (x86): ${DSR_ARCH}")
message(STATUS "API-DRFL Ubuntu release: ${DSR_VERSION}")
add_library(
    doosan::drflapi 
    STATIC IMPORTED 
)
set_property(
    TARGET doosan::drflapi 
    PROPERTY IMPORTED_LOCATION ${DSR_PATH}/library/${DSR_OS}/${DSR_ARCH}/${DSR_VERSION}/libDRFL.a

)
target_include_directories(
    doosan::drflapi
    INTERFACE
    ${DSR_PATH}/include
)
target_link_libraries(
    doosan::drflapi
    INTERFACE
    Poco::Foundation
    Poco::Net
)
