# Crowd-SLAM

Crowd-SLAM is a visual SLAM system that is robust in crowded scenarios.

Demonstration video: https://www.youtube.com/watch?v=LeS8MEVaR2E
Paper: https://link.springer.com/article/10.1007/s10846-021-01414-1

<img src="images/example.png"
width="773" height="489" /></a>
<!--img src="images/MOT20-01_CYTi.jpg" 
width="960" height="540" /></a!-->

# License

Crowd-SLAM is released under a GPLv3 License.

If you use Crowd-SLAM in an academic work, please cite:

    @article{soaresJINT2021,
      title={Crowd-{SLAM}: Visual {SLAM} Towards Crowded Environments using Object Detection},
      author={Soares, J. C. V., Gattass, M. and Meggiolaro, M. A.},
      journal={Journal of Intelligent & Robotic Systems},
      volume={102},
      number={50},
      doi = {https://doi.org/10.1007/s10846-021-01414-1},
      year={2021}
     }
     
# Building Crowd-SLAM
- Install ORB-SLAM2 prerequisites: C++11 or C++0x Compiler, Pangolin, OpenCV and Eigen3  (https://github.com/raulmur/ORB_SLAM2).
- Clone the repository:
```
git clone https://github.com/virgolinosoares/Crowd-SLAM
```
- Execute:
```
cd Crowd-SLAM
chmod +x build.sh
./build.sh
```

We have tested the library in **Ubuntu 18.04**, with OpenCV 3.4.

# RGB-D Example (TUM Dataset)

- Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

- Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools):

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```
- Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER` to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./rgbd Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# Acknowledgements
Our code builds on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).
