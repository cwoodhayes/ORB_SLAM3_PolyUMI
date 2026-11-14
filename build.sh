# Build ORB-SLAM3 and the four third-party dependencies it needs, all in-tree.
#
# Order matters. DBoW2 and g2o are linked from Thirdparty/, Sophus is header-only, and Pangolin
# has to exist before ORB-SLAM3 configures: the final cmake points find_package(Pangolin) at the
# build tree produced here rather than at a system install, so nothing extra has to be set up.
# The vocabulary is also unpacked on the way through.
#
# Job count defaults to 2 because an unbounded `make -j` exhausts RAM on g2o's and Pangolin's
# heavier translation units. Raise it on a machine with memory to spare:
#
#   bash build.sh                      # 2 jobs
#   MAKE_JOBS=$(nproc) bash build.sh   # all cores
#
# Safe to re-run; every build directory is created with mkdir -p.

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"${MAKE_JOBS:-2}"

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"${MAKE_JOBS:-2}"

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
# BUILD_TESTS=OFF skips Sophus' test binaries; on GCC 14 their -Werror clashes
# with new Eigen alignment warnings. ORB_SLAM3 only needs the headers anyway.
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j"${MAKE_JOBS:-2}"

cd ../../Pangolin
echo "Configuring and building Thirdparty/Pangolin ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j"${MAKE_JOBS:-2}"

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir -p build
cd build
# Point find_package(Pangolin) at the in-tree Pangolin build we just produced
# above (Thirdparty/Pangolin/build hosts PangolinConfig.cmake).
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH="$(cd ../Thirdparty/Pangolin/build && pwd)"
make -j"${MAKE_JOBS:-2}"
