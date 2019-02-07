cmake_minimum_required(VERSION 2.8.3)

project(multitasker)

# Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED
    COMPONENTS message_generation std_msgs rospy)

  # Declare the message files to be built
  add_message_files(FILES
    TaskState.msg
  #  MyMessage2.msg
  )


  # Declare the service files to be built
  add_service_files(FILES
    TaskRequest.srv
    LaunchConditions.srv
    StartTask.srv
    HoldConditions.srv
    SchedulingConditions.srv
  )

  catkin_python_setup()
if(USE_PDDL)
execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/compile-pddl-planner.sh

		WORKIN_DIRECTORY ${CATKIN_PACKAGE_SOURCE_DIR}
		)
endif()

#message( ${CMAKE_CURRENT_SOURCE_DIR} )
#add_custom_command(TARGET VAL
#                   PRE_BUILD 
#                   COMMAND make clean
#		   COMMAND make
#                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../VAL
#		  )
#add_custom_command(TARGET universal-pddl-parser
#                   PRE_BUILD 
#                   COMMAND scons 
#                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../universal-pddl-parser 
#                  )
#add_custom_command(TARGET temporal-planning
#                   PRE_BUILD 
#                   COMMAND python fd_copy/build.py release64
#		   COMMAND scons 
#                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../temporal-planning
#                  )


  # Actually generate the language-specific message and service files
  generate_messages(DEPENDENCIES std_msgs)

# Catkin package  
  catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rospy
   DEPENDS )



  # define executable using MyMessage1 etc.
  #add_executable(message_program src/main.cpp)
  #add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # define executable not using any messages/services provided by this package
  #add_executable(does_not_use_local_messages_program src/main.cpp)
  #add_dependencies(does_not_use_local_messages_program ${catkin_EXPORTED_TARGETS})

  catkin_install_python(PROGRAMS 
  		bin/exampleState
  		bin/txt_guideHuman
  		bin/txt_hazardDetection
      bin/txt_human_fell
      bin/harmonization_table_generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
