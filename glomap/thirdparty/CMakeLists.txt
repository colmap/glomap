# Determine project compiler.
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(IS_MSVC TRUE)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(IS_GNU TRUE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(IS_CLANG TRUE)
endif()

# Only show moderate warnings for external library code.
if(IS_MSVC)
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    elseif(CMAKE_CXX_FLAGS MATCHES "/Wall")
        string(REGEX REPLACE "/Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
    if(CMAKE_C_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    elseif(CMAKE_C_FLAGS MATCHES "/Wall")
        string(REGEX REPLACE "/Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
elseif(IS_GNU OR IS_CLANG)
    if(CMAKE_CXX_FLAGS MATCHES "-Wall")
        string(REGEX REPLACE "-Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
    if(CMAKE_C_FLAGS MATCHES "-Wall")
        string(REGEX REPLACE "-Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
        set(REMOVED_WARNING_LEVEL TRUE)
    endif()
endif()

if(IS_MSVC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")
elseif(IS_GNU OR IS_CLANG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
endif()

include(FetchContent)
FetchContent_Declare(PoseLib
    GIT_REPOSITORY    https://github.com/PoseLib/PoseLib.git
    GIT_TAG           7e9f5f53372e43f89655040d4dfc4a00e5ace11c  # 2.0.5
    EXCLUDE_FROM_ALL
    SYSTEM
)
message(STATUS "Configuring PoseLib...")
if (FETCH_POSELIB)
    FetchContent_MakeAvailable(PoseLib)
else()
    find_package(PoseLib REQUIRED)
endif()
message(STATUS "Configuring PoseLib... done")

FetchContent_Declare(COLMAP
    GIT_REPOSITORY    https://github.com/colmap/colmap.git
    GIT_TAG           b6b7b54eca6078070f73a3f0a084f79c629a6f10  # Nov 20, 2025
    EXCLUDE_FROM_ALL
)
message(STATUS "Configuring COLMAP...")
set(UNINSTALL_ENABLED OFF CACHE INTERNAL "")
set(GUI_ENABLED OFF CACHE INTERNAL "")
if (FETCH_COLMAP)
    FetchContent_MakeAvailable(COLMAP)
else()
    find_package(COLMAP REQUIRED)
endif()
message(STATUS "Configuring COLMAP... done")
