$EIGEN_VERSION="3.2.8"
$EIGEN_HASH="07105f7124f9"

cd $Env:SOURCE_FOLDER
appveyor DownloadFile "http://bitbucket.org/eigen/eigen/get/$EIGEN_VERSION.zip"
7z x "${EIGEN_VERSION}.zip" -o"${Env:SOURCE_FOLDER}\eigen" -r
cd "${Env:SOURCE_FOLDER}\eigen\eigen-eigen-$EIGEN_HASH"
md build
cd build

# Build, make and install Eigen
cmake -G "Visual Studio 14 2015 Win64" -DCMAKE_INSTALL_PREFIX="${Env:CMAKE_INSTALL_PREFIX}" ../
msbuild INSTALL.vcxproj

# Generate eigen3.pc
$EIGEN3_PC_FILE="${Env:PKG_CONFIG_PATH}/eigen3.pc"
echo "Name: Eigen3" | Out-File -Encoding ascii -FilePath $EIGEN3_PC_FILE
echo "Description: A C++ template library for linear algebra: vectors, matrices, and related algorithms" | Out-File -Append -Encoding ascii -FilePath $EIGEN3_PC_FILE
echo "Requires:" | Out-File -Append -Encoding ascii -FilePath $EIGEN3_PC_FILE
echo "Version: $EIGEN_VERSION" | Out-File -Append -Encoding ascii -FilePath $EIGEN3_PC_FILE
echo "Libs:" | Out-File -Append -Encoding ascii -FilePath $EIGEN3_PC_FILE
echo "Cflags: -I${Env:CMAKE_INSTALL_PREFIX}/include/eigen3" | Out-File -Append -Encoding ascii -FilePath $EIGEN3_PC_FILE

# Check install
pkg-config --modversion "eigen3 >= ${EIGEN_VERSION}"
pkg-config --cflags "eigen3 >= ${EIGEN_VERSION}"
