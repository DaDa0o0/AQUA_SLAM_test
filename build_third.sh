echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
rm build lib -rf
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o
rm build lib -rf
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../octomap
rm build lib -rf
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

