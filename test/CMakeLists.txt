cmake_minimum_required(VERSION 3.5)
if(CATKIN_ENABLE_TESTING)
  foreach(TARGET
      test_pattern_finder.py
      )
    catkin_add_nosetests(${TARGET} DEPENDENCIES ${catkin_EXPORTED_TARGETS})
  endforeach()
endif()