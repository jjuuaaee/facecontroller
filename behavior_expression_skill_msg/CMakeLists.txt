##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(behavior_expression_skill_msg)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  FaceInfo.msg
  TTSSetProperties.msg
  GestureProfileInfo.msg
  GestureInfo.msg
  DanceStartInfo.msg
)


add_service_files(
  DIRECTORY srv
  FILES
  ExpressionCommand.srv
  Face.srv
  SpeechSynthesis.srv
  AffectiveState.srv
  Unit.srv
  UnitInfo.srv
  DanceInfo.srv
)


generate_messages(DEPENDENCIES std_msgs)

##############################################################################
# Package
##############################################################################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
