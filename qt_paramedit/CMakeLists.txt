cmake_minimum_required(VERSION 2.8.3)
project(qt_paramedit)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

# needed: otherwise boost signals name clash
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

QT4_WRAP_UI(uis_h
   src/MainWindow.ui
   src/SettingsDialog.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR} include ${catkin_INCUDE_DIRS})

set(qt_lib_srcs
   src/xmlRpcTreeItem.cpp
   src/xmlRpcItemDelegate.cpp
   src/xmlRpcModel.cpp)

set(qt_lib_hdrs
   include/qt_paramedit/xmlRpcTreeItem.h
   include/qt_paramedit/xmlRpcItemDelegate.h
   include/qt_paramedit/xmlRpcModel.h)

set(qt_srcs
   src/MainWindow.cpp
   src/settingsDialog.cpp)

set(qt_hdrs
   src/MainWindow.h
   src/settingsDialog.h)

qt4_automoc(${qt_lib_srcs})
QT4_WRAP_CPP(qt_moc_lib_srcs ${qt_lib_hdrs})

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES xmlRpcModel
    CATKIN_DEPENDS roscpp
    DEPENDS libqt4-dev
)

add_library(xmlRpcModel ${qt_lib_srcs} ${qt_moc_lib_srcs})
target_link_libraries(xmlRpcModel ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} ${uis_h} ${qt_srcs} ${qt_moc_srcs} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} xmlRpcModel ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} xmlRpcModel
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

