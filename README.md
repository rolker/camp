# AutonomousMissionPlanner
Planning software for autonomous surface and underwater vehicles.

Building on Windows 10 

These were the steps followed using the lastest versions at the time. May also work with different versions.

- Installed Visual Studio Community 2017
- Installed QT 5.9.1 (offline installer) (selecting VS 2017 64bit libraries)
- Installed CMake 3.9.1
- Built GDAL 2.2.1 from source.

Building GDAL

- edited nmake.opt modifiying GDAL_HOME to desired install location
- opened x64 Native Tools COmmand Prompt for VS 2017
- cd'ed to GDAL source directory
- nmake -f makefile.vc MSVC_VER=1910 WIN64=1
- nmake -f makefile.vc MSVC_VER=1910 WIN64=1 devinstall

Building AutonomousMissionPlanner

- Using CMake GUI, choose source directory and build directory (created ./build/ in source directory)
- Picked Visual Studio 15 2017 Win64 as the generator.
- Help CMake find QT5 by pointing Qt5_DIR at C:\Qt\Qt5.9.1\5.9.1\msvc2017_64\lib\cmake\Qt5
- Specify GDAL header and library locations
- Customized CMAKE_INSTALL_PREFIX. (Default gave access errors trying to install)
- Generate project
- Opened AutonomousMissionPlanner.sln in build directory.
- Built INSTALL target.
