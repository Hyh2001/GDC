execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE}" -B "${CONSUMER_BUILD}"
  "-Diekf_estimator_DIR=${PACKAGE_DIR}" "-DCMAKE_CXX_COMPILER=${CONSUMER_COMPILER}"
  RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Installed consumer configuration failed")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${CONSUMER_BUILD}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Installed consumer build failed")
endif()
execute_process(COMMAND "${CONSUMER_BUILD}/consumer" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Installed consumer execution failed")
endif()
