# T-0352: Windows installer (CPack NSIS, ADR-0120). Included only when WIN32 and the app is built.
# Layout and shared rules: cmake/CarvettoInstall.cmake.

# Qt DLLs, plugins and translations: Qt's deploy script runs windeployqt at install time.
qt_generate_deploy_app_script(TARGET carvetto OUTPUT_SCRIPT carvetto_deploy_script
                              NO_UNSUPPORTED_PLATFORM_ERROR)
install(SCRIPT ${carvetto_deploy_script})
# windeployqt puts plugins/translations beside bin/ (plugins/platforms/...), and the deploy script writes
# no qt.conf, so tell Qt where they are: paths are relative to the directory holding qt.conf.
install(CODE [[
  file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/qt.conf"
       "[Paths]\nPrefix = ..\nPlugins = plugins\nTranslations = translations\n")
]])

# MSVC runtime (VCRUNTIME/MSVCP/UCRT) beside the executables, so a clean machine needs no redistributable.
set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)
set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION ${CMAKE_INSTALL_BINDIR})
include(InstallRequiredSystemLibraries)

set(CPACK_GENERATOR NSIS)
set(CPACK_PACKAGE_NAME Carvetto)
set(CPACK_PACKAGE_VENDOR Carvetto)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Open-source CNC CAD/CAM")
set(CPACK_PACKAGE_FILE_NAME "Carvetto-${PROJECT_VERSION}-win64")
set(CPACK_PACKAGE_INSTALL_DIRECTORY Carvetto)
set(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE)

# T-0363: installer and uninstaller icons (forward slashes are what CPack's NSIS template expects).
install(FILES ${PROJECT_SOURCE_DIR}/packaging/icons/carvetto.ico DESTINATION ${CARVETTO_INSTALL_DATADIR})
set(CPACK_NSIS_MUI_ICON ${PROJECT_SOURCE_DIR}/packaging/icons/carvetto.ico)
set(CPACK_NSIS_MUI_UNIICON ${PROJECT_SOURCE_DIR}/packaging/icons/carvetto.ico)

set(CPACK_NSIS_DISPLAY_NAME "Carvetto")
set(CPACK_NSIS_PACKAGE_NAME "Carvetto")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
set(CPACK_NSIS_MODIFY_PATH OFF)
set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
# Start-menu shortcut (and uninstall shortcut) for bin/carvetto.exe.
set(CPACK_PACKAGE_EXECUTABLES "carvetto" "Carvetto")

# .carv file association (machine-wide; the installer needs admin rights for Program Files anyway).
# The uninstaller deletes exactly the keys written here, and only if they still point at us.
# CPack copies these variables into CPackConfig.cmake inside a quoted string without escaping, so
# carvetto_nsis_escape() escapes backslashes and double quotes once; CPack's re-read restores the
# plain NSIS text written below.
function(carvetto_nsis_escape out text)
  string(REPLACE "\\" "\\\\" text "${text}")
  string(REPLACE "\"" "\\\"" text "${text}")
  set(${out} "${text}" PARENT_SCOPE)
endfunction()

carvetto_nsis_escape(_install_cmds [=[
  WriteRegStr HKLM "Software\Classes\.carv" "" "Carvetto.Document"
  WriteRegStr HKLM "Software\Classes\Carvetto.Document" "" "Carvetto document"
  WriteRegStr HKLM "Software\Classes\Carvetto.Document\DefaultIcon" "" "$INSTDIR\share\carvetto\carvetto.ico"
  WriteRegStr HKLM "Software\Classes\Carvetto.Document\shell\open\command" "" '"$INSTDIR\bin\carvetto.exe" "%1"'
  System::Call 'shell32::SHChangeNotify(i 0x08000000, i 0, i 0, i 0)'
]=])
carvetto_nsis_escape(_uninstall_cmds [=[
  DeleteRegKey HKLM "Software\Classes\Carvetto.Document"
  ReadRegStr $0 HKLM "Software\Classes\.carv" ""
  StrCmp $0 "Carvetto.Document" 0 +2
  DeleteRegKey HKLM "Software\Classes\.carv"
  System::Call 'shell32::SHChangeNotify(i 0x08000000, i 0, i 0, i 0)'
]=])
set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${_install_cmds}")
set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "${_uninstall_cmds}")

include(CPack)
