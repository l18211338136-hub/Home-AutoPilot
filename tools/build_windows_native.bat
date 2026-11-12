@echo off
rem Convenience wrapper for hosts without a Unix toolchain: load the MSVC
rem environment, then use the project's standard CMake + Ninja build.
rem Override VSINSTALL if your Visual Studio is installed elsewhere.
setlocal
if "%VSINSTALL%"=="" set "VSINSTALL=C:\Program Files\Microsoft Visual Studio\18\Enterprise"
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1

rem Some sandboxed / minimal VS setups skip the Windows SDK tools in PATH,
rem which breaks the manifest (rc.exe / mt.exe) step of CMake's compiler check.
rem Use a literal path: %ProgramFiles(x86)% cannot be expanded inside a block.
set "SDKROOT=C:\Program Files (x86)\Windows Kits\10"
set "MSVC_VER="
set "SDK_VER="
for /d %%i in ("%VSINSTALL%\VC\Tools\MSVC\*") do set "MSVC_VER=%%~nxi"
for /d %%i in ("%SDKROOT%\Include\*") do set "SDK_VER=%%~nxi"
set "VCTOOLS=%VSINSTALL%\VC\Tools\MSVC\%MSVC_VER%"
set "SDK_BIN=%SDKROOT%\bin\%SDK_VER%\x64"

rem Explicit INCLUDE/LIB keep this working even when registry queries are
rem unavailable (hardened / sandboxed hosts).
set "INCLUDE=%VCTOOLS%\include;%SDKROOT%\Include\%SDK_VER%\ucrt;%SDKROOT%\Include\%SDK_VER%\um;%SDKROOT%\Include\%SDK_VER%\shared;%INCLUDE%"
set "LIB=%VCTOOLS%\lib\x64;%SDKROOT%\Lib\%SDK_VER%\ucrt\x64;%SDKROOT%\Lib\%SDK_VER%\um\x64;%LIB%"
if exist "%SDK_BIN%\rc.exe" set "PATH=%PATH%;%SDK_BIN%"

set "CMAKE=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
set "CTEST=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe"
set "ROOT=%~dp0.."
if "%HOMEFLY_BUILD_DIR%"=="" set "HOMEFLY_BUILD_DIR=%ROOT%\build\linux"
if "%HOMEFLY_CMAKE_ARGS%"=="" set "HOMEFLY_CMAKE_ARGS="

rem Optional overrides:
rem   set HOMEFLY_BUILD_DIR=...\build\filtered
rem   set HOMEFLY_CMAKE_ARGS=-DHF_LOG_COMPILE_MIN_LEVEL=2
"%CMAKE%" -S "%ROOT%" -B "%HOMEFLY_BUILD_DIR%" -G Ninja ^
    -DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl ^
    -DCMAKE_MAKE_PROGRAM="%NINJA%" -DHOMEFLY_PLATFORM=linux %HOMEFLY_CMAKE_ARGS% || exit /b 1
"%CMAKE%" --build "%HOMEFLY_BUILD_DIR%" || exit /b 1
"%CTEST%" --test-dir "%HOMEFLY_BUILD_DIR%" --output-on-failure || exit /b 1
endlocal
