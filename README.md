# About

GLOMAP is a general purpose global structure-from-motion pipeline for
image-based reconstruction. GLOMAP requires a COLMAP database as input and
outputs a COLMAP sparse reconstruction.

# Run the GLOMAP

To run the GLOMAP, one can follow this steps

```
mkdir build
cd build
cmake ..
make -j8
./debug DATABASE_PATH OUTPUT_PATH 
```
