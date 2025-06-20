<div align="center">

<h1>MapEval: Towards Unified, Robust and Efficient SLAM Map Evaluation Framework</h1>

[**Xiangcheng Hu**](https://github.com/JokerJohn)<sup>1</sup> · [**Jin Wu**](https://zarathustr.github.io/)<sup>1</sup> · [**Mingkai  Jia**](https://github.com/MKJia)<sup>1</sup>· [**Hongyu  Yan**](https://scholar.google.com/citations?user=TeKnXhkAAAAJ&hl=zh-CN)<sup>1</sup>· [**Yi  Jiang**](https://yijiang1992.github.io/)<sup>2</sup>· [**Binqian  Jiang**](https://github.com/lewisjiang/)<sup>1</sup>
<br>
[**Wei Zhang**](https://ece.hkust.edu.hk/eeweiz)<sup>1</sup> · [**Wei  He**](https://sites.google.com/view/drweihecv/home/)<sup>3</sup> · [**Ping Tan**](https://facultyprofiles.hkust.edu.hk/profiles.php?profile=ping-tan-pingtan#publications)<sup>1*&dagger;</sup>

<sup>1</sup>**HKUST&emsp;&emsp;&emsp;<sup>2</sup>CityU&emsp;&emsp;&emsp;<sup>3</sup>USTB**  
<br>
&dagger;Project lead&emsp;*Corresponding Author

<a href="https://ieeexplore.ieee.org/document/10910156"><img src='https://img.shields.io/badge/RAL 2025- MapEval -red' alt='Paper PDF'></a>[![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/Cloud_Map_Evaluation.svg)](https://github.com/JokerJohn/Cloud_Map_Evaluation/stargazers)<a href="https://github.com/JokerJohn/PALoc/network/members">
<img alt="FORK" src="https://img.shields.io/github/forks/JokerJohn/Cloud_Map_Evaluation?color=white" />
</a> <a ><img alt="PRs-Welcome" src="https://img.shields.io/badge/PRs-Welcome-white" /></a>[![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/Cloud_Map_Evaluation.svg)](https://github.com/JokerJohn/Cloud_Map_Evaluation/issues)[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

</div>

![image-20241127080805642](./README/image-20241127080805642.png)


MapEval is a comprehensive framework for evaluating point cloud maps in SLAM systems, addressing two fundamentally distinct aspects of map quality assessment:
1. **Global Geometric Accuracy**: Measures the absolute geometric fidelity of the reconstructed map compared to ground truth. This aspect is crucial as SLAM systems often accumulate drift over long trajectories, leading to global deformation.
2. **Local Structural Consistency**: Evaluates the preservation of local geometric features and structural relationships, which is essential for tasks like obstacle avoidance and local planning, even when global accuracy may be compromised.

These complementary aspects require different evaluation approaches, as global drift may exist despite excellent local reconstruction, or conversely, good global alignment might mask local inconsistencies. Our framework provides a unified solution through both traditional metrics and novel evaluation methods based on optimal transport theory.

## News

- **2025/05/05:** Add new test data and remove the simulation codes.
- **2025/03/05**: [Formally published](https://ieeexplore.ieee.org/document/10910156)!
- **2025/02/25**: Accept!
- **2025/02/12**: Codes released! 
- **2025/02/05**: Resubmit.
- **2024/12/19**: Submitted to **IEEE RAL**! 

## Key Features

**Traditional Metrics Implementation**:

- **Accuracy** (AC): Point-level geometric error assessment
- **Completeness** (COM): Map coverage evaluation.
- **Chamfer Distance** (CD): Bidirectional point cloud difference
- **Mean Map Entropy** (MME): Information-theoretic local consistency metric

**Novel Proposed Metrics**:

- **Average Wasserstein Distance** (AWD): Robust global geometric accuracy assessment
- **Spatial Consistency Score** (SCS): Enhanced local consistency evaluation

| ![image-20250304121753995](./README/image-20250304121753995.png) |
| ------------------------------------------------------------ |

## Results

### Simulated experiments

| Noise Sensitivity                                            | Outlier Robustness                                           |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20241129123446075](./README/image-20241129123446075.png) | ![image-20241129091845196](./README/image-20241129091845196.png) |

![image-20241127083707943](./README/image-20241127083707943.png)

### Real-world experiments

| Map Evaluation via Localization Accuracy                     | Map Evaluation in Diverse Environments                       |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20241127083813797](./README/image-20241127083813797.png) | ![image-20241127083801691](./README/image-20241127083801691.png) |

| ![image-20241129092052634](./README/image-20241129092052634.png) |
| ------------------------------------------------------------ |

##  Efficiency and Parameter Analysis

| ![image-20250322192323830](./README/image-20250322192323830.png) | ![image-20250322192349614](./README/image-20250322192349614.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

## Datasets

| [MS-dataset](https://github.com/JokerJohn/MS-Dataset) | [FusionPortable (FP) and FusionPortableV2 dataset](https://fusionportable.github.io/dataset/fusionportable_v2/) | [Newer College (NC)](https://ori-drs.github.io/newer-college-dataset/) | [ GEODE dataset (GE)](https://github.com/PengYu-Team/GEODE_dataset) |
| ----------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

| ![image-20250322192302315](./README/image-20250322192302315.png) |
| ------------------------------------------------------------ |



## Quickly Run

### Dependencies

- *[Open3d ( >= 0.11)](https://github.com/isl-org/Open3D)* 
- Eigen3
- yaml-cpp
- Ubuntu 20.04

### Test Data(password: 1)

| sequence |                                                              | Test PCD                                                     | GT PCD                                                       |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| MCR_slow | ![image-20250515145431522](./README/image-20250515145431522.png) | [map.pcd](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/ES9eSANEr-9NvkFqMzMFsecBo5r3hBpBnj0c6BMPgsfXnQ?e=aijdPf) | [map_gt.pcd](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/ESfn5EEsiPlCiJcydVc_HqgBDGqy65MHoyu63XE-iKbFBQ?e=dTDon4) |
| PK01     | ![image-20250515144915631](./README/image-20250515144915631.png) | [map.pcd](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/ERPFVJN6CtBKtHlPWyni-jIB0dgLzgF1FGxPTatKoCp02Q?e=TEgfBp) | [gt.pcd](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/EeztnFHwKJlCoW-fmKljaMMBSvNvT5BkTXxoA1iXqeUS5A?e=37evMi) |

### Usage

1. install open3d. (maybe a higer version of CMake is needed)

```bash
git clone https://github.com/isl-org/Open3D.git
cd Open3D && mkdir build && cd build   
cmake ..
make install
```

2. set and read the instruction of some params in [config.yaml](map_eval/config/config.yaml).

```yaml
# accuracy_level, vector5d, we mainly use the result of the first element
# if inlier is very small, we can try to larger the value, e.g. for outdoors, [0.5, 0.3, 0.2, 0.1, 0.05]
accuracy_level: [0.2, 0.1, 0.08, 0.05, 0.01]

# initial_matrix, vector16d, the initial matrix of the registration
# make sure the format is correct, or you will got the error log: YAML::BadSubscript' what():  operator[] call on a scalar
initial_matrix:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  
# vmd voxel size, outdoor: 2.0-4.0; indoor: 2.0-3.0
vmd_voxel_size: 3.0
```


3. complie map_eval

```bash
git clone https://github.com/JokerJohn/Cloud_Map_Evaluation.git
cd Cloud_Map_Evaluation/map_eval && mkdir build
cmake ..
make
```

4. get the final results

```bash
./map_eval
```

we have a point cloud map generated by a pose-slam system, and we have a ground truth point cloud map. Then we caculate related metrics.

![image-20250214100110872](./README/image-20250214100110872.png)

### Visulization

We can also get a rendered raw distance-error map(10cm) and inlier distance-error map(2cm) in this process, the color R->G->B represent for the distance error at a level of 0-10cm.

![image (4)](./README/image%20(4).png)

**if we do not have gt ma**p, we can only evaluate the **Mean Map Entropy (MME)**, smaller means better consistency. just set `evaluate_mme: false` in **[config.yaml](map_eval/config/config.yaml)**.

![image (5)](./README/image%20(5).png)

we can also get a simpe mesh reconstructed from point cloud map.

![image-20230101200651976](README/image-20230101200651976.png)

5. we got the result flies.

![image-20250212202446474](./README/image-20250212202446474.png)

6. if you want to get the visulization of voxel errors, use the  [error-visualization.py](map_eval/scripts/error-visualization.py) 

   ```python
   pip install numpy matplotlib scipy
   
   python3 error-visualization.py
   ```

   | ![image-20250212202920950](./README/image-20250212202920950.png) | ![image-20250212202933255](./README/image-20250212202933255.png) |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![image-20250212203009074](./README/image-20250212203009074.png) | ![image-20250212203025149](./README/image-20250212203025149.png) |

## Issues

### How do you get your initial pose?

we can use [CloudCompare](https://github.com/CloudCompare/CloudCompare) to align LIO map to Gt map .

- Roughly  translate and rotate the LIO point cloud map to the GT map。

- Manually register the moved LIO map (aligned) to the GT map (reference), and get the output of the terminal transfrom `T2`, then the initial pose matrix is the terminal output transform `T`.

| ![image-20230106135937336](README/image-20230106135937336.png) | ![image-20230106140017020](README/image-20230106140017020.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

### What's the difference between raw rendered map and inlier rendered map? 

The primary function of the r**aw rendered map** (left) is to color-code the error of all points in the map estimated by the algorithm. For each point in the estimated map that does not find a corresponding point in the **ground truth (gt) map**, it is defaulted to the maximum error (**20cm**), represented as red. On the other hand, the i**nlier rendered map** (right) excludes the non-overlapping regions of the point cloud and colors only the error of the inlier points after point cloud matching. This map therefore contains only a portion of the points from the original estimated map. (remindered by [John-Henawy](https://github.com/John-Henawy) in [issue 5](https://github.com/JokerJohn/Cloud_Map_Evaluation/issues/5))

![image (6)](./README/image%20(6).png)

### **Applicable Scenarios:**

1. **With a ground truth map:** All metrics are applicable.

2. **Without a ground truth map** (remindered by [@Silentbarber](https://github.com/Silentbarber), [ZOUYIyi](https://github.com/ZOUYIyi) in [issue 4](https://github.com/JokerJohn/Cloud_Map_Evaluation/issues/4) and [issue 7](https://github.com/JokerJohn/Cloud_Map_Evaluation/issues/7)):

    - Only **MME** can be used for evaluation. It is crucial to remember that the maps being evaluated must be on the same scale. 

      > For example, **you cannot compare a LIO map with a LIO SLAM map** that has performed loop closure optimization. This is because loop closure adjusts the local point cloud structure, leading to inaccurate MME evaluation. You can compare the MME of different LIO maps.

## Publications

We recommend to cite [our paper](https://arxiv.org/abs/2411.17928) if you find this library useful:

```latex
@misc{hu2024mapeval,
      title={MapEval: Towards Unified, Robust and Efficient SLAM Map Evaluation Framework}, 
      author={Xiangcheng Hu and Jin Wu and Mingkai Jia and Hongyu Yan and Yi Jiang and Binqian Jiang and Wei Zhang and Wei He and Ping Tan},
      year={2025},
      volume={10},
      number={5},
      pages={4228-4235},
  	  doi={10.1109/LRA.2025.3548441}
}

@article{wei2024fpv2,
  title={Fusionportablev2: A unified multi-sensor dataset for generalized slam across diverse platforms and scalable environments},
  author={Wei, Hexiang and Jiao, Jianhao and Hu, Xiangcheng and Yu, Jingwen and Xie, Xupeng and Wu, Jin and Zhu, Yilong and Liu, Yuxuan and Wang, Lujia and Liu, Ming},
  journal={The International Journal of Robotics Research},
  pages={02783649241303525},
  year={2024},
  publisher={SAGE Publications Sage UK: London, England}
}
```

## Related Package

The folloing works use MapEval for map evalution.

| Work                                                         | Tasks                                    | Date       | Metrics   | Demo                                                         |
| ------------------------------------------------------------ | ---------------------------------------- | ---------- | --------- | ------------------------------------------------------------ |
| [**LEMON-Mapping**](https://arxiv.org/abs/2505.10018)        | Multi-Session Point Cloud mapping        | Arxiv'2025 | MME       | ![image-20250517201051378](./README/image-20250517201051378.png) |
| **[CompSLAM](https://arxiv.org/abs/2505.06483)**             | Multi-Modal Localization<br/>and Mapping | Arxiv'2025 | AWD/SCS   | ![image-20250513202215918](./README/image-20250513202215918.png) |
| [**GEODE**](https://github.com/PengYu-Team/GEODE_dataset)    | SLAM Dataset                             | IJRR'2025  |           | ![image-20250513204209752](./README/image-20250513204209752.png) |
| [**ELite**](https://github.com/dongjae0107/ELite)            | LiDAR-based Lifelong Mapping             | ICRA'2025  | AC/CD     | ![image-20250513202654237](./README/image-20250513202654237.png) |
| [**PALoc**](https://github.com/JokerJohn/PALoc)              | Prior-Assisted 6-DoF Localization        | TMECH'2024 | AC/CD     | ![image-20250513205043100](./README/image-20250513205043100.png) |
| [**MS-Mapping**](https://github.com/JokerJohn/MS-Mapping)    | Multi-session LiDAR mapping              | Arxiv'2024 | AC/CD/MME | ![image-20240730152951528](./README/image-20240730152951528.png) |
| [**FusionPortableV2**](https://journals.sagepub.com/doi/full/10.1177/02783649241303525) | SLAM Dataset                             | IJRR'2024  | COM/CD    | ![img](./README/10.1177_02783649241303525-fig15.jpg)         |

## Contributors

<a href="https://github.com/JokerJohn/Cloud_Map_Evaluation/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=JokerJohn/Cloud_Map_Evaluation" />
</a>

![Star History Chart](https://api.star-history.com/svg?repos=JokerJohn/Cloud_Map_Evaluation&type=Date)
