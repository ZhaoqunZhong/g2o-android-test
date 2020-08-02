# g2o-android-test

## Platform

Ubuntu18

## Get g2o source code 

https://github.com/RainerKuemmerle/g2o

## Build g2o for Android 10 

	cd build
	rm -r *

	cmake \
	-DANDROID_ABI=arm64-v8a \
	-DANDROID_NDK=/home/zhaoqun/Android/Sdk/ndk/21.1.6352462 \
	-DCMAKE_TOOLCHAIN_FILE=/home/zhaoqun/Android/Sdk/ndk/21.1.6352462/build/cmake/android.toolchain.cmake \
	-DANDROID_NATIVE_API_LEVEL=27 \
	-DCMAKE_BUILD_TYPE=Release \
	-DEIGEN3_INCLUDE_DIR=/home/zhaoqun/Documents/eigen-3.3.7 \
	-DEIGEN3_VERSION_OK=ON ..

	make -j8

Adjust above -D definations according to your case. 

## Find built g2o shared libs in g2o/libs

