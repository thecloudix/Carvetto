# T-0352: install rules shared by every packaging target (Windows installer now; Linux packages, T-0353).
# Layout is relative to the prefix and platform neutral:
#   bin/carvetto[.exe], bin/carvetto-cli[.exe]
#   share/carvetto/{posts,gadgets,tooldb,fonts,translations,licenses}
# The app finds posts and gadgets at ../share/carvetto/<dir> from its own directory
# (shippedPostsDir / shippedGadgetsDir), so nothing here needs a compile-time prefix.
include(GNUInstallDirs)

set(CARVETTO_INSTALL_DATADIR ${CMAKE_INSTALL_DATADIR}/carvetto)

if(TARGET carvetto)
  install(TARGETS carvetto RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(TARGET carvetto-cli)
  install(TARGETS carvetto-cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/posts/ DESTINATION ${CARVETTO_INSTALL_DATADIR}/posts
        PATTERN "samples" EXCLUDE)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/gadgets/stock/ DESTINATION ${CARVETTO_INSTALL_DATADIR}/gadgets)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/data/tools/ DESTINATION ${CARVETTO_INSTALL_DATADIR}/tooldb)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/data/fonts/ DESTINATION ${CARVETTO_INSTALL_DATADIR}/fonts)
# Compiled translations (*.qm) are installed when the tree has them (none yet).
if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/data/translations)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/data/translations/ DESTINATION ${CARVETTO_INSTALL_DATADIR}/translations
          FILES_MATCHING PATTERN "*.qm")
endif()
install(FILES ${PROJECT_SOURCE_DIR}/LICENSE ${PROJECT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md
        DESTINATION ${CARVETTO_INSTALL_DATADIR}/licenses)
