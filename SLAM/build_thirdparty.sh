# echo "-------------Install Pangolin------------------------"
# cd Thirdparty/Pangolin
# mkdir build 
# cd build
# cmake ..
# make
# make install 
# 
# cd ../../../

echo "---------------Install DBow2----------------------"
echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
rm build -rf
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

cd ../../g2o

echo "----------------Install G2o-------------------"
echo "Configuring and building Thirdparty/g2o ..."

rm build -rf
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

cd ../../../

echo "--------------------------------------"
echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz

