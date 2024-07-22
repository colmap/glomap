# GLOMAP: Global Structure-from-Motion Revisited

## About
GLOMAP is a general purpose global structure-from-motion pipeline for
image-based reconstruction. GLOMAP requires a COLMAP database as input and
outputs a COLMAP sparse reconstruction.
The goal of this project is to achieve both efficiency and robustness.
If you use this project for your research, please cite
```

```

## Getting Started 
To install GLOMAP, one can follow this steps
```
mkdir build
cd build
cmake ..
make install
```
After installation, one can run GLOMAP by (starting from a database)
```
glomap mapper --database_path DATABASE_PATH --output_path OUTPUT_PATH
```
For more details on the Command Line Interface, one can type `glomap -h" or 'glomap mapper -h" for help.
To obtain a colored reconstruction, it is recommended to call
```
colmap color_extractor --image_path IMAGE  --input_path MODEL --output_path OUTPUT
```

Note:
- GLOMAP depends on two external libraries - [COLMAP](https://github.com/colmap/colmap) and [PoseLib](https://github.com/PoseLib/PoseLib).
  With the default setting, the library will build them via `FetchContent`.
  However, if a self-installed version is prefered, one can also set `FETCH_COLMAP` or `FETCH_POSELIB` in `CMakeLists.txt` to be `OFF`.
- To use `FetchContent` the minimum required version of `cmake` is 3.28. If a self-installed version is used, `cmake` can be downgraded to 3.10.

## End-to-End example
To obtain a reconstruction from Images, one can follow these steps
```
colmap feature_extractor --image_path IMAGE --database_path DATABASE
colmap vocab_tree_matcher --database_path DATABASE --VocabTreeMatching.vocab_tree_path VOCAB_TREE
glomap mapper --database_path DATABASE_PATH --output_path OUTPUT_PATH

```



## Acknowledgement
We are highly inspired by COlMAP, PoseLib, Theia.

## Support
Please, use GitHub Discussions at https://github.com/colmap/glomap/discussions for questions and the GitHub issue tracker at https://github.com/colmap/glomap for bug reports, feature requests/additions, etc.

## Contribution
Contributions (bug reports, bug fixes, improvements, etc.) are very welcome and should be submitted in the form of new issues and/or pull requests on GitHub.

## License

<!-- 

