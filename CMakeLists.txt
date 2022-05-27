cmake_minimum_required(VERSION 3.13.4)

project(RoTMS)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://www.slicer.org/wiki/Documentation/Nightly/Extensions/RoTMS")
set(EXTENSION_CATEGORY "RoTMS")
set(EXTENSION_CONTRIBUTORS "Yihao Liu (Johns Hopkins University)")
set(EXTENSION_DESCRIPTION "This is anextension for Robotic-TMS.")
set(EXTENSION_ICONURL "http://www.example.com/Slicer/Extensions/RoTMS.png")
set(EXTENSION_SCREENSHOTURLS "http://www.example.com/Slicer/Extensions/RoTMS/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(RobotControl)
add_subdirectory(MRIPlan)
add_subdirectory(TargetVisualization)
add_subdirectory(SlicerTestExample)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
