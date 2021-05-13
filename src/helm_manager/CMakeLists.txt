
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(rqt_helm_manager_SRCS
  rqt_helm_manager.cpp
  helm_manager.cpp
)

set(rqt_helm_manager_HDRS
  helm_manager.h
  rqt_helm_manager.h
)

set(rqt_helm_manager_UIS
  helm_manager.ui
  rqt_helm_manager.ui
)

add_library(rqt_helm_manager
  ${rqt_helm_manager_SRCS}
)

qt5_use_modules(rqt_helm_manager Widgets)

target_link_libraries(rqt_helm_manager
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

class_loader_hide_library_symbols(rqt_helm_manager)

install(TARGETS rqt_helm_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
