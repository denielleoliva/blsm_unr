# ---- Install targets ----
install(TARGETS
  ${PROJECT_NAME}
  calibrate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# Plugin XML (guarded)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/blossom_dxl_hw_plugins.xml)
  install(FILES blossom_dxl_hw_plugins.xml DESTINATION share/${PROJECT_NAME})
endif()

# Share data (URDF, config, launch), guarded & relative
set(PKG_SHARE_DIR share/${PROJECT_NAME})
foreach(dir IN LISTS)
  # (no-op if you paste this blank; see next foreach)
endforeach()

foreach(_dir urdf config launch)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_dir})
    install(DIRECTORY ${_dir}/ DESTINATION ${PKG_SHARE_DIR}/${_dir})
  endif()
endforeach()

# Python scripts (only if present)
foreach(_py src/blossom_yaml_player.py src/blossom_behaviors.py)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_py})
    install(PROGRAMS ${_py} DESTINATION lib/${PROJECT_NAME})
  endif()
endforeach()
