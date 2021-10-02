idf_component_register(
  SRC_DIRS src
  INCLUDE_DIRS "./src/include"
  PRIV_INCLUDE_DIRS "./src"
  EXCLUDE_SRCS "./src/Ps3Controller.cpp" "./src/Ps3Controller.h"
  REQUIRES "bt"
)
