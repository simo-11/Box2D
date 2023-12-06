rem Use this batch file to build ep for Visual Studio
rmdir /s /q build
mkdir build
cd build
cmake ..
cmake --build .
start ep.sln
