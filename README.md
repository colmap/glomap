# About
GLOMAP is a general purpose structure-from-motion pipeline for building reconstruction efficiently. This library takes COLMAP database as input and directly output reconstruction (in COLMAP data format).

# Run the GLOMAP

To run the GLOMAP, one can follow this steps

```
mkdir build
cd build
cmake ..
make -j8
./debug DATABASE_PATH OUTPUT_PATH 
```

Ensure that SuiteSparse, COLMAP, PoseLib are installed before running the code