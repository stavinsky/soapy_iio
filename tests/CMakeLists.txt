add_executable(my_tests test_main.cpp )
target_include_directories(my_tests  SYSTEM PUBLIC ${SoapySDR_INCLUDE_DIRS})
target_link_libraries(my_tests PRIVATE SoapySDR  ${LIBIIO_LIBRARIES})
add_test(NAME RXFlowWithSingleChannel COMMAND my_tests)

