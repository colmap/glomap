We would like to thank [Asadali242](https://github.com/Asadali242) for providing the installation guide for MAC.
Leave your comments at [Issue #62](https://github.com/colmap/glomap/issues/62) if you encounter any problems.

## Installing the COLMAP:

*1. Open the Terminal and install the brew dependencies:*
```
brew install \
cmake \
ninja \
boost \
eigen \
flann \
libomp \ #(Install Libomp as well)
freeimage \
metis \
glog \
googletest \
ceres-solver \
qt@5 \
glew \
cgal \
sqlite3
```

*2. Clone the COLMAP repository:*
```
git clone https://github.com/colmap/colmap.git
cd colmap
```

*3. Ensure Qt5 is in your PATH:*
```
export PATH="/opt/homebrew/opt/qt@5/bin:$PATH"
```

*4. Create a build directory:*
```
mkdir build
cd build
```

*5. After installing, link Qt5 to make sure it’s accessible:*
```
brew link qt@5 --force
```

*6. Run CMake with the specific paths for ARM Mac(M1 and above):*
```
cmake .. -GNinja \
  -DCMAKE_PREFIX_PATH="/opt/homebrew/opt/flann;/opt/homebrew/opt/metis;/opt/homebrew/opt/suite-sparse;/opt/homebrew/opt/qt@5;/opt/homebrew/opt/freeimage"
```

*7. Build and install COLMAP:*
```
ninja
sudo ninja install
```

*8. Confirm COLMAP installation by running:*
```
colmap -h
colmap gui
```

**This is the first part and will install Colmap on your device.**



## Installing the GLOMAP:

*1. Clone the GitHub repository:*
```
git clone https://github.com/colmap/glomap
cd glomap
```

*2. Create a build directory:*
```
mkdir build
cd build
```

*3. Export Environment Variables Again:*
```
export PATH="/opt/homebrew/opt/qt@5/bin:$PATH"
export LDFLAGS="-L/opt/homebrew/opt/libomp/lib"
export CPPFLAGS="-I/opt/homebrew/opt/libomp/include"
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5;/opt/homebrew/opt/libomp"
export PKG_CONFIG_PATH="/opt/homebrew/opt/qt@5/lib/pkgconfig"
```

*4. Run the CMake Command:*
```
cmake -DCMAKE_PREFIX_PATH="/opt/homebrew/Cellar/qt@5/5.15.13_1;/opt/homebrew/opt/libomp" \
-DOpenMP_C_FLAGS="-Xclang -fopenmp" \
-DOpenMP_C_LIB_NAMES="libomp" \
-DOpenMP_CXX_FLAGS="-Xclang -fopenmp" \
-DOpenMP_CXX_LIB_NAMES="libomp" \
-DOpenMP_C_INCLUDE_DIRS="/opt/homebrew/opt/libomp/include" \
-DOpenMP_CXX_INCLUDE_DIRS="/opt/homebrew/opt/libomp/include" \
-DOpenMP_libomp_LIBRARY=/opt/homebrew/opt/libomp/lib/libomp.dylib \
-DOpenMP_INCLUDE_DIR=/opt/homebrew/opt/libomp/include \
.. -GNinja
```

*5. Build the Project:*
```
ninja
```

**NOTE: If at this point, there are build errors related to ‘cholmod.h’ or ‘omp.h’, clean the build and then re-run the make with the following commands:**
```
cmake -DCMAKE_PREFIX_PATH="/opt/homebrew/Cellar/qt@5/5.15.13_1;/opt/homebrew/opt/libomp" \
-DOpenMP_C_FLAGS="-Xclang -fopenmp -I/opt/homebrew/opt/libomp/include" \
-DOpenMP_C_LIB_NAMES="libomp" \
-DOpenMP_CXX_FLAGS="-Xclang -fopenmp -I/opt/homebrew/opt/libomp/include" \
-DOpenMP_CXX_LIB_NAMES="libomp" \
-DOpenMP_libomp_LIBRARY=/opt/homebrew/opt/libomp/lib/libomp.dylib \
.. -GNinja
```

**After the build is successful:**

*6. Install the Built Project:*
```
sudo ninja install
```

*7. Test the Installation:*
```
glomap -h
```

**It should display something like:**
```
GLOMAP -- Global Structure-from-Motion
Usage:
glomap mapper --database_path DATABASE --output_path
MODEL
glomap mapper_resume --input_path MODEL_INPUT --output_path MODEL_OUTPUT
Available commands:
help
mapper
mapper_resume
```


## Testing with the end-to-end examples provided:

*1. Open the end-to-end example database link provided:*
https://lpanaf.github.io/eccv24_glomap/

*2. Download one of the datasets provided and extract the zip file.*

*3. Create a new directory named ‘data’ in the root directory ‘glomap’.*

*4. Place the extracted dataset in the directory ‘data’.*

*5. Now navigate back to the project directory ‘glomap’.*

**NOTE: Following commands are assuming the dataset to be south-building:**

*6. Extract Features with COLMAP:*
```
colmap feature_extractor \
--image_path ./data/south-building/images \
--database_path ./data/south-building/database.db
```

*7. Match Features with COLMAP:*
```
colmap exhaustive_matcher \
--database_path ./data/south-building/database.db
```

*8. Run GLOMAP Mapper:*
```
glomap mapper \
--database_path ./data/south-building/database.db \
--image_path ./data/south-building/images \
--output_path ./output/south-building/sparse
```

**This should generate a new directory named ‘output’ in the project directory.**

*9. Visualize the Results:*
```
colmap gui \
--database_path ./data/south-building/database.db \
--image_path ./data/south-building/images \
--import_path ./output/south-building/sparse/0
```
