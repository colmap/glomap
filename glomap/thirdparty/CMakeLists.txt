# Only show moderate warnings for external library code.
if(IS_MSVC)
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    elseif(CMAKE_CXX_FLAGS MATCHES "/Wall")
        string(REGEX REPLACE "/Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
    if(CMAKE_C_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    elseif(CMAKE_C_FLAGS MATCHES "/Wall")
        string(REGEX REPLACE "/Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    endif()
elseif(IS_GNU OR IS_CLANG)
    if(CMAKE_CXX_FLAGS MATCHES "-Wall")
        string(REGEX REPLACE "-Wall" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
    if(CMAKE_C_FLAGS MATCHES "-Wall")
        string(REGEX REPLACE "-Wall" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    endif()
endif()

include(FetchContent)
FetchContent_Declare(PoseLib
    GIT_REPOSITORY    https://github.com/PoseLib/PoseLib.git
    GIT_TAG           f119951fca625133112acde48daffa5f20eba451
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
    SYSTEM
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
