# GLOMAP: Global Structure-from-Motion Revisited

## About
GLOMAP is a general purpose global structure-from-motion pipeline for
image-based reconstruction. GLOMAP requires a COLMAP database as input and
outputs a COLMAP sparse reconstruction.
The goal of this project is to achieve both efficiency and robustness, and can scale up to >10k images.
If you use this project for your research, please cite
```
@inproceedings{pan2024glomap,
    author={Pan, Linfei and Baráth, Dániel and Pollefeys, Marc and Sch\"{o}nberger, Johannes Lutz},
    title={Global Structure-from-Motion Revisited},
    booktitle={European Conference on Computer Vision (ECCV)},
    year={2024},
}
```

## Getting Started 
To install GLOMAP, one can follow this steps
```
mkdir build
cd build
cmake ..
make install -j8
```
After installation, one can run GLOMAP by (starting from a database)
```
glomap mapper --database_path DATABASE_PATH --output_path OUTPUT_PATH
```
For more details on the Command Line Interface, one can type `glomap -h" or 'glomap mapper -h" for help.
To obtain a colored reconstruction, it is recommended to call
```
colmap color_extractor --image_path IMAGE  --input_path MODEL_INPUT --output_path MODEL_OUTPUT
```

Note:
- GLOMAP depends on two external libraries - [COLMAP](https://github.com/colmap/colmap) and [PoseLib](https://github.com/PoseLib/PoseLib).
  With the default setting, the library will build them via `FetchContent`.
  However, if a self-installed version is prefered, one can also set `FETCH_COLMAP` or `FETCH_POSELIB` in `CMakeLists.txt` to be `OFF`.
- To use `FetchContent` the minimum required version of `cmake` is 3.28. If a self-installed version is used, `cmake` can be downgraded to 3.10.

## End-to-End Example
In this section, we will use datasets from [this link](demuc.de/colmap/datasets) as example.
Download the datasets, and put them under `data` folder.
### From database
If a database is already extracted, GLOMAP can be directly called to perform mapping
```
glomap mapper \
    --database_path ./data/person-hall/database.db \
    --output_path ./output/person-hall/sparse
colmap color_extractor \
    --image_path ./data/person-hall/images \
    --input_path ./output/person-hall/sparse/0 \
    --output_path ./output/person-hall/sparse/0
```
### From images
To obtain a reconstruction from images, the database needs to be established first. Here, we utilize the functions from COLMAP to achieve this (for installation, )
```
colmap feature_extractor \
    --image_path ./data/south-building/images/ \
    --database_path ./data/south-building/database.db
colmap exhaustive_matcher \
    --database_path ./data/south-building/database.db 
glomap mapper \
    --database_path ./data/south-building/database.db \
    --output_path ./output/south-building/sparse
colmap color_extractor
    --image_path ./data/south-building/images \
    --input_path ./output/south-building/sparse/0 \
    --output_path ./output/south-building/sparse/0
```
### Notes
- For larger scale datasets, it is recommended to use `sequential_matcher` or `vocab_tree_matcher` from `COLMAP`. Please refer to [COLMAP](https://github.com/colmap/colmap) for proper citing.
```
colmap sequential_matcher --database_path DATABASE_PATH

colmap vocab_tree_matcher --database_path DATABASE_PATH --VocabTreeMatching.vocab_tree_path VOCAB_TREE_PATH
```
- Alternatively, one can use [hloc](https://github.com/cvg/Hierarchical-Localization/) for image retrival and matching with leraning



## Acknowledgement
We are highly inspired by COLMAP, PoseLib, Theia.

## Support
Please, use GitHub Discussions at https://github.com/colmap/glomap/discussions for questions and the GitHub issue tracker at https://github.com/colmap/glomap for bug reports, feature requests/additions, etc.

## Contribution
Contributions (bug reports, bug fixes, improvements, etc.) are very welcome and should be submitted in the form of new issues and/or pull requests on GitHub.

## License

