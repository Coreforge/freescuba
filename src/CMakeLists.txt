# SteamVR Overlay
add_subdirectory ("openvr_overlay")

# SteamVR Driver
if(WIN32)
add_subdirectory ("openvr_driver")
endif()