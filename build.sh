echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
# BUILD_TESTS=OFF skips Sophus' test binaries; on GCC 14 their -Werror clashes
# with new Eigen alignment warnings. ORB_SLAM3 only needs the headers anyway.
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j

cd ../../Pangolin
echo "Configuring and building Thirdparty/Pangolin ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
# Point find_package(Pangolin) at the in-tree Pangolin build we just produced
# above (Thirdparty/Pangolin/build hosts PangolinConfig.cmake).
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH="$(cd ../Thirdparty/Pangolin/build && pwd)"
make -j
