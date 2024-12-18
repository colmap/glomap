# Gravity-aligned Rotation Averaging with Circular Regression 

[Project page](https://lpanaf.github.io/eccv24_ra1dof/) | [Paper](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/05651.pdf) | [Supp.](https://lpanaf.github.io/assets/pdf/eccv24_ra1dof_sm.pdf)
---

## About

This project aims at solving the rotation averaging problem with gravity prior. 
To achieve this, circular regression is leveraged.

If you use this project for your research, please cite
```
@inproceedings{pan2024ra1dof,
    author={Pan, Linfei and Pollefeys, Marc and Barath, Daniel},
    title={{Gravity-aligned Rotation Averaging with Circular Regression}},
    booktitle={European Conference on Computer Vision (ECCV)},
    year={2024},
}
```

## Getting Started
Install GLOMAP as instrcucted in [README](../README.md).
Then, call the rotation averager (3 degree-of-freedom) via 
```
glomap rotation_averager --relpose_path RELPOSE_PATH --output_path OUTPUT_PATH
```

If gravity directions are available, call the rotation averager (1 degree-of-freedom) via
```
glomap rotation_averager \
    --relpose_path RELPOSE_PATH \
    --output_path OUTPUT_PATH \
    --gravity_path GRAVTIY PATH 
```
It is recommended to set `--use_stratified=1` if only a subset of images have gravity direction. 
If gravity measurements are subject to i.i.d. noise, they can be refined by setting `--refine_gravity=1`.


## File Formats
### Relative Pose
The relative pose file is expected to be of the following format
```
IMAGE_NAME_1 IMAGE_NAME_2 QW QX QY QZ TX TY TZ
```
Only images contained in at least one relative pose will be included in the following procedure.
The relative pose should be <sub>2</sub>R<sub>1</sub> x<sub>1</sub> + <sub>2</sub>t<sub>1</sub> = x<sub>2</sub>.

### Gravity Direction
The gravity direction file is expected to be of the following format
```
IMAGE_NAME GX GY GZ
```
The gravity direction $g$ should $[0, 1, 0]$ if the image is parallel to the ground plane, and the estimated rotation would have the property that $R_i \cdot [0, 1, 0]^\top = g$.
If is acceptable if only a subset of all images have gravity direciton.
If the specified image name does not match any known image name from relative pose, it is ignored.

### Output
The estimated global rotation will be in the following format
```
IMAGE_NAME QW QX QY QZ
```
Any images that are not within the largest connected component of the view-graph formed by the relative pose will be ignored.
