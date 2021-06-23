cmake_minimum_required(VERSION 3.5)
if(CATKIN_ENABLE_TESTING)
  foreach(TARGET
      test_pattern_finder.py
      )
    catkin_add_nosetests(${TARGET} DEPENDENCIES ${catkin_EXPORTED_TARGETS})
  endforeach()

  find_package(rostest REQUIRED)
  foreach(LAUNCH_FILE
      test_pattern_finder.test
    )
    add_rostest(launch/${LAUNCH_FILE}
      DEPENDENCIES
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
      ${rostest_EXPORTED_TARGETS})
  endforeach()
endif()