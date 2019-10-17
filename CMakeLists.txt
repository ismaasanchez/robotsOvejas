cmake_minimum_required(VERSION 3.15)
project(robotsOvejas)

set(CMAKE_CXX_STANDARD 14)

include_directories(oveja/src)
include_directories(oveja/src/MyFirstComp_autogen/include)

add_executable(robotsOvejas
        oveja/src/MyFirstComp_autogen/EWIEGA46WW/moc_genericmonitor.cpp
        oveja/src/MyFirstComp_autogen/EWIEGA46WW/moc_genericworker.cpp
        oveja/src/MyFirstComp_autogen/EWIEGA46WW/moc_specificmonitor.cpp
        oveja/src/MyFirstComp_autogen/EWIEGA46WW/moc_specificworker.cpp
        oveja/src/MyFirstComp_autogen/include/moc_sigwatch.cpp
        oveja/src/MyFirstComp_autogen/include/ui_mainUI.h
        oveja/src/MyFirstComp_autogen/mocs_compilation.cpp
        oveja/src/CommonBehavior.cpp
        oveja/src/CommonBehavior.h
        oveja/src/commonbehaviorI.cpp
        oveja/src/commonbehaviorI.h
        oveja/src/config.h
        oveja/src/DifferentialRobot.cpp
        oveja/src/DifferentialRobot.h
        oveja/src/GenericBase.cpp
        oveja/src/GenericBase.h
        oveja/src/genericmonitor.cpp
        oveja/src/genericmonitor.h
        oveja/src/genericworker.cpp
        oveja/src/genericworker.h
        oveja/src/Laser.cpp
        oveja/src/Laser.h
        oveja/src/main.cpp
        oveja/src/specificmonitor.cpp
        oveja/src/specificmonitor.h
        oveja/src/specificworker.cpp
        oveja/src/specificworker.h
        oveja/src/ui_mainUI.h)
