cmake_minimum_required(VERSION 3.8)
project(champ_simulation)

find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})

    catkin_package()

    set(SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif(${ament_cmake_FOUND})

    ament_package()

    set(SHARE_DESTINATION share/${PROJECT_NAME})

endif()

# Install directories
set(directories launch worlds maps config)
foreach(dir IN LISTS directories)
    install(
        DIRECTORY ${dir}
        DESTINATION ${SHARE_DESTINATION}
    )
endforeach()
