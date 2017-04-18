# npy_converter

First install the cnpy library from https://github.com/rogersce/cnpy
Then compile velodyne_reader.cpp with the CMakeLists.txt file provided in this repository.
Before running the executable, manually create three folders named "intensity", "density" and "height" in the same folder where velodyne_reader.cpp was built. Without these folders, a segmentation fault will occur during run time.
