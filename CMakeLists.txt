cmake_minimum_required(VERSION 2.8.3)
project(qt_paramedit)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Qt5 COMPONENTS Core Gui Widgets Xml REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# needed: otherwise boost signals name clash
add_definitions(-DQT_NO_KEYWORDS)

include_directories(${CMAKE_CURRENT_BINARY_DIR} include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES xmlRpcModel
    CATKIN_DEPENDS roscpp
    DEPENDS libqt5-widgets libqt5-core libqt5-gui
)

add_library(xmlRpcModel
   include/qt_paramedit/xmlRpcTreeItem.h
   include/qt_paramedit/xmlRpcItemDelegate.h
   include/qt_paramedit/xmlRpcModel.h
   src/xmlRpcTreeItem.cpp
   src/xmlRpcItemDelegate.cpp
   src/xmlRpcModel.cpp
    )
target_link_libraries(xmlRpcModel
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Xml
    ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}
    src/MainWindow.ui
    src/SettingsDialog.ui
    src/MainWindow.cpp
    src/settingsDialog.cpp
    src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Xml
    xmlRpcModel
    ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} xmlRpcModel
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

