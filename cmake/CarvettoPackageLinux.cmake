# T-0353: Linux desktop integration and packages (CPack DEB and RPM; AppImage via packaging/linux/make-appimage.sh;
# ADR-0124). Layout and shared rules: cmake/CarvettoInstall.cmake. Included on Linux/BSD only.

# --- desktop integration (installed by every `cmake --install`, packaged or not) ---------------------------------
set(_lin ${PROJECT_SOURCE_DIR}/packaging/linux)
install(FILES ${_lin}/carvetto.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
install(FILES ${_lin}/carvetto.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/mime/packages)
# T-0363: the icon set (artwork and provenance: packaging/icons/README.md). 16 and 24 px use the simplified variant.
set(_ico ${PROJECT_SOURCE_DIR}/packaging/icons)
install(FILES ${_ico}/carvetto.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps)
foreach(_s 32 48 64 128 256)
  install(FILES ${_ico}/carvetto-${_s}.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/${_s}x${_s}/apps
          RENAME carvetto.png)
endforeach()
foreach(_s 16 24)
  install(FILES ${_ico}/carvetto-small-${_s}.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/${_s}x${_s}/apps
          RENAME carvetto.png)
endforeach()
# The same artwork is the document icon of the .carv MIME type.
install(FILES ${_ico}/carvetto.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/mimetypes
        RENAME application-x-carvetto-project.svg)

if(NOT TARGET carvetto)
  return()
endif()

# --- bundled Qt (off by default: a system install uses the distribution's Qt) -------------------------------------
# Layout, relative to the prefix:  lib/carvetto/lib/libQt6*.so, lib/carvetto/plugins/<kind>/*.so, bin/qt.conf.
# The executable gets RUNPATH $ORIGIN/../lib/carvetto/lib; qt.conf points Qt at the plugins. Qt's own plugins
# already look for their libraries in $ORIGIN/../../lib, which is exactly lib/carvetto/lib in this layout.
# Only Qt and its private ICU are bundled; every other library comes from the distribution (package Depends).
option(CARVETTO_BUNDLE_QT "Install Qt libraries and plugins beside the application (packages, AppImage)" OFF)
if(CARVETTO_BUNDLE_QT)
  set(_qtroot ${CMAKE_INSTALL_LIBDIR}/carvetto)
  set_target_properties(carvetto PROPERTIES INSTALL_RPATH "\$ORIGIN/../${_qtroot}/lib" BUILD_WITH_INSTALL_RPATH OFF)
  # Plugins: the platform plugins (xcb for real use, offscreen for --self-test), image formats, icon engines and
  # the xcb GL integrations. Found beside the xcb plugin's location in the Qt tree.
  get_target_property(_xcb Qt6::QXcbIntegrationPlugin LOCATION)
  if(NOT _xcb)
    get_target_property(_xcb Qt6::QXcbIntegrationPlugin IMPORTED_LOCATION_RELEASE)
  endif()
  if(NOT _xcb)
    message(FATAL_ERROR "CARVETTO_BUNDLE_QT: the Qt xcb platform plugin was not found (Qt6::QXcbIntegrationPlugin)")
  endif()
  cmake_path(GET _xcb PARENT_PATH _xcb_dir)
  cmake_path(GET _xcb_dir PARENT_PATH _qt_plugins)
  set(CARVETTO_QT_PLUGIN_DIR ${_qt_plugins} CACHE INTERNAL "")
  install(CODE "set(CARVETTO_EXE \"$<TARGET_FILE:carvetto>\")
set(CARVETTO_QT_PLUGINS \"${_qt_plugins}\")
set(CARVETTO_QT_ROOT \"${_qtroot}\")")
  install(CODE [[
    # file(INSTALL) prepends DESTDIR itself; file(GLOB/WRITE) and external tools need it spelled out.
    set(_dest "${CMAKE_INSTALL_PREFIX}/${CARVETTO_QT_ROOT}")
    set(_root "$ENV{DESTDIR}${_dest}")
    set(_plugin_files)
    foreach(_kind platforms imageformats iconengines xcbglintegrations platforminputcontexts)
      file(GLOB _found "${CARVETTO_QT_PLUGINS}/${_kind}/*.so")
      foreach(_p ${_found})
        get_filename_component(_n "${_p}" NAME)
        # Only the plugins the app can use: xcb, offscreen, and every non-platform plugin.
        if(NOT _kind STREQUAL "platforms" OR _n MATCHES "^libq(xcb|offscreen)\\.so$")
          file(INSTALL "${_p}" DESTINATION "${_dest}/plugins/${_kind}")
          list(APPEND _plugin_files "${_p}")
        endif()
      endforeach()
    endforeach()
    file(GET_RUNTIME_DEPENDENCIES
      EXECUTABLES "${CARVETTO_EXE}"
      LIBRARIES ${_plugin_files}
      RESOLVED_DEPENDENCIES_VAR _deps
      UNRESOLVED_DEPENDENCIES_VAR _unresolved
      PRE_INCLUDE_REGEXES "^libQt6" "^libicu"
      PRE_EXCLUDE_REGEXES ".*")
    file(INSTALL ${_deps} DESTINATION "${_dest}/lib" FOLLOW_SYMLINK_CHAIN)
    # Unresolved Qt/ICU names mean the bundle would be broken; anything else is a distribution library.
    foreach(_u ${_unresolved})
      if(_u MATCHES "^libQt6|^libicu")
        message(FATAL_ERROR "bundled Qt: unresolved dependency ${_u}")
      endif()
    endforeach()
    # Qt plugins from the Qt installer already search $ORIGIN/../../lib; other builds (e.g. conda) search elsewhere,
    # so pin it when patchelf is available (CI installs it, and checks where the plugin resolves Qt from).
    find_program(CARVETTO_PATCHELF patchelf)
    if(CARVETTO_PATCHELF)
      file(GLOB_RECURSE _installed_plugins "${_root}/plugins/*.so")
      foreach(_p ${_installed_plugins})
        execute_process(COMMAND "${CARVETTO_PATCHELF}" --set-rpath "$ORIGIN/../../lib" "${_p}"
                        RESULT_VARIABLE _rc)
        if(NOT _rc EQUAL 0)
          message(FATAL_ERROR "patchelf failed on ${_p}")
        endif()
      endforeach()
    endif()
    file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/qt.conf"
         "[Paths]\nPrefix = ../${CARVETTO_QT_ROOT}\nPlugins = plugins\nLibraries = lib\n")
  ]])
endif()

# --- packages ------------------------------------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB;RPM")
set(CPACK_PACKAGING_INSTALL_PREFIX /usr)
set(CPACK_PACKAGE_NAME carvetto)
set(CPACK_PACKAGE_VENDOR Carvetto)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Open-source CNC CAD/CAM")
set(CPACK_PACKAGE_DESCRIPTION "Carvetto draws and edits 2D/3D designs, plans toolpaths, simulates them and posts G-code.")
set(CPACK_PACKAGE_CONTACT "Carvetto contributors <noreply@carvetto.invalid>")
set(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE)
set(CPACK_STRIP_FILES ON)

set(CPACK_DEBIAN_FILE_NAME "carvetto_${PROJECT_VERSION}_amd64.deb")
set(CPACK_DEBIAN_PACKAGE_SECTION graphics)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/thecloudix/Carvetto")
# Libraries Qt's xcb/GL plugins load that the bundle does not carry. Listed by hand because dpkg-shlibdeps cannot
# resolve the bundled Qt libraries. Alternatives cover the t64 renames of Ubuntu 24.04 / Debian 13.
set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "libc6, libstdc++6, libgcc-s1, libgl1, libegl1, libxkbcommon0, libxkbcommon-x11-0, libxcb-cursor0, libxcb-icccm4, libxcb-image0, libxcb-keysyms1, libxcb-randr0, libxcb-render-util0, libxcb-shape0, libxcb-xkb1, libxcb-glx0, libxcb-sync1, libxcb-xfixes0, libxcb-xinerama0, libxcb1, libx11-6, libx11-xcb1, libfontconfig1, libfreetype6, libdbus-1-3, libglib2.0-0t64 | libglib2.0-0, libdouble-conversion3, libpcre2-16-0, libzstd1, zlib1g")
set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "shared-mime-info")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${_lin}/deb-postinst ${_lin}/deb-postrm)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)

set(CPACK_RPM_FILE_NAME "carvetto-${PROJECT_VERSION}-1.x86_64.rpm")
set(CPACK_RPM_PACKAGE_LICENSE "see /usr/share/carvetto/licenses/LICENSE")
set(CPACK_RPM_PACKAGE_GROUP "Applications/Engineering")
# rpmbuild's automatic dependency scan would require the bundled libQt6*.so as separate packages.
set(CPACK_RPM_PACKAGE_AUTOREQ OFF)
set(CPACK_RPM_PACKAGE_AUTOPROV OFF)
set(CPACK_RPM_PACKAGE_REQUIRES
    "mesa-libGL, mesa-libEGL, libxkbcommon, libxkbcommon-x11, xcb-util-cursor, xcb-util-wm, xcb-util-image, xcb-util-keysyms, xcb-util-renderutil, fontconfig, freetype, dbus-libs, glib2, double-conversion, pcre2-utf16, libzstd, zlib")
set(CPACK_RPM_POST_INSTALL_SCRIPT_FILE ${_lin}/rpm-post.sh)
set(CPACK_RPM_POST_UNINSTALL_SCRIPT_FILE ${_lin}/rpm-postun.sh)
# The prefix directories are owned by the filesystem package; do not claim them.
set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION /usr/share/applications /usr/share/mime /usr/share/mime/packages
    /usr/share/icons /usr/share/icons/hicolor /usr/share/icons/hicolor/scalable
    /usr/share/icons/hicolor/scalable/apps /usr/share/icons/hicolor/scalable/mimetypes
    /usr/share/icons/hicolor/16x16 /usr/share/icons/hicolor/16x16/apps
    /usr/share/icons/hicolor/24x24 /usr/share/icons/hicolor/24x24/apps
    /usr/share/icons/hicolor/32x32 /usr/share/icons/hicolor/32x32/apps
    /usr/share/icons/hicolor/48x48 /usr/share/icons/hicolor/48x48/apps
    /usr/share/icons/hicolor/64x64 /usr/share/icons/hicolor/64x64/apps
    /usr/share/icons/hicolor/128x128 /usr/share/icons/hicolor/128x128/apps
    /usr/share/icons/hicolor/256x256 /usr/share/icons/hicolor/256x256/apps)

include(CPack)
