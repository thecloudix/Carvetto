# ADR-0118: Windows installer (CPack NSIS)

Date: 2026-11-17 · Status: accepted (T-0352)

## Context
T-0352 asks for a Windows installer built in CI (Qt bundled, shipped data, .carv association, Start menu, clean
uninstall). T-0353 will add Linux packages from the same install rules. The author could not run Windows locally, so the
choices favour the smallest, best documented mechanism and a CI job that proves it end to end.

## Decision
1. **CPack NSIS**, not WiX. NSIS is preinstalled on the GitHub `windows-latest` image (the job installs it via
   choco if missing), needs no extra toolchain, supports `/S` silent install and `/S _?=dir` synchronous silent
   uninstall, and file association is a few registry lines. WiX remains an option if MSI (policy deployment) is wanted.
2. **Platform-neutral install layout** in `cmake/CarvettoInstall.cmake`, shared with Linux packaging:
   `bin/` (carvetto, carvetto-cli) and `share/carvetto/{posts,gadgets,tooldb,fonts,translations,licenses}`. The app
   already searches `../share/carvetto/{posts,gadgets}` from its own directory, so no compile-time prefix is needed.
   Only `cmake/CarvettoPackageWindows.cmake` (included when `WIN32` and the app target exists) is Windows specific.
3. **Qt deployment** with `qt_generate_deploy_app_script` (runs `windeployqt` at install time, so it lands in the CPack
   staging tree) plus `InstallRequiredSystemLibraries` for the MSVC/UCRT runtime beside the executables.
4. **File association** machine-wide in `HKLM\Software\Classes` (`.carv` -> `Carvetto.Document`), because the installer
   installs to `Program Files` and needs admin anyway. The uninstaller removes the document key and removes `.carv`
   only if it still names Carvetto. Per-user settings under AppData are deliberately left (user data, not installed files).
5. NSIS snippets are passed through `carvetto_nsis_escape()` because CPack writes variables into `CPackConfig.cmake`
   without escaping quotes and backslashes (verified on Linux by round-tripping the config).
6. **CI**: a separate `windows-installer` job (main, `v*` tags, manual dispatch, `task/T-0352*` PRs and PRs labelled
   `packaging`) builds, runs CPack, uploads the artifact, installs silently, runs `carvetto --self-test` from the
   installed copy with Qt removed from `PATH`, uninstalls, and fails if any file, registry key or Start-menu folder remains.

## Limits
No application icon or installer artwork yet (none exists; default icons are used). Code signing is not done, so
SmartScreen may warn. The installer path was not run locally (no Windows); the CI job is its first execution.
