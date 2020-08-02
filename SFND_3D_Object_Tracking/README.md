# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.



## FP.5 : Performance Evaluation 1

This exercise is about conducting tests with the final project code, especially with regard to the Lidar part. Look for several examples where you have the impression that the Lidar-based TTC estimate is way off. Once you have found those, describe your observations and provide a sound argumentation why you think this happened.

| Frame | Prev Closest (m) | Current Closest (m) | TTCLidar (sec) |
| ----- | ---------------- | ------------------- | -------------- |
| 0th   | \-               | 7.974               | \-             |
| 1th   | 7.974            | 7.913               | 12.9722        |
| 2th   | 7.913            | 7.849               | 12.264         |
| 3th   | 7.849            | 7.793               | 13.9161        |
| 4th   | 7.793            | 7.685               | 7.11572        |
| 5th   | 7.685            | 7.638               | 16.2511        |
| 6th   | 7.638            | 7.577               | 12.4213        |
| 7th   | 7.577            | 7.555               | 34.3404        |
| 8th   | 7.555            | 7.475               | 9.34376        |
| 9th   | 7.475            | 7.434               | 18.1318        |
| 10th  | 7.434            | 7.393               | 18.0318        |
| 11th  | 7.393            | 7.205               | 3.83244        |
| 12th  | 7.205            | 7.272               | \-10.8537      |
| 13th  | 7.272            | 7.194               | 9.22307        |
| 14th  | 7.194            | 7.129               | 10.9678        |
| 15th  | 7.129            | 7.042               | 8.09422        |
| 16th  | 7.042            | 6.827               | 3.17535        |
| 17th  | 6.827            | 6.896               | \-9.99424      |
| 18th  | 6.896            | 6.814               | 8.30978        |

### Observation 1:  Indirect speed measurement is too sensitive to data noise

Based on 4th data in the table above, a change of 10 cm distance can causes the TTC jumps from 14 sec to 7sec. Such a minor noise of Lidar measurement can have a big impact to TTC accuracy because this method estimate speed indirectly from Lidar point distance.

---

### Observation 2: 

Lidar points are relatively accurate compared to other sensors, but sometimes the points are slightly off when there is strong reflection, or when the Lidar coverage to the target car is different. These noises is the main reason why the Lidar TTC estimation is not accurate. 



## FP.6 : Performance Evaluation 2

This last exercise is about running the different detector / descriptor combinations and looking at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons. This is the last task in the final project.

---

As of the previous project, I have already chosen top 3 combinations of detector and descriptor based on number of keypoints and detection duration. They are:

**Performance Score Definition**: (1/Average time) * (Average matches)

| TOP3 COMBINATION | AVERAGE TIME (ms) | AVERAGE MATCH | Score |
| ---------------- | ----------------- | ------------- | ----- |
| ORB - ORB        | 14.4              | 98            | 6.8   |
| FAST - BRIEF     | 29.4              | 100.7         | 3.4   |
| FAST - ORB       | 28.9              | 97.7          | 3.4   |

Therefore, I have proceed with these three combinations for this project, the collected data is shown in the table below.

|            | ORB - ORB      |            | FAST - BRIEF   |            | FAST - ORB     |            | Lidar     |
| ---------- | -------------- | ---------- | -------------- | ---------- | -------------- | ---------- | --------- |
| Frame      | No. of Matches | TTC Camera | No. of Matches | TTC Camera | No. of Matches | TTC Camera | TTC Lidar |
| 0th        | \-             | \-         | \-             | \-         | \-             | \-         | \-        |
| 1th        | 19             | 16.7       | 53             | 11.7       | 43             | 11.1       | 13.0      |
| 2th        | 18             | 7.4        | 58             | 12.7       | 49             | 11.7       | 12.3      |
| 3th        | 21             | 32.2       | 61             | 13.2       | 54             | 14.0       | 13.9      |
| 4th        | 11             | \-inf      | 53             | 13.2       | 44             | 14.1       | 7.1       |
| 5th        | 29             | 31.0       | 50             | 17.1       | 39             | \-inf      | 16.3      |
| 6th        | 18             | \-inf      | 64             | 13.5       | 54             | 13.4       | 12.4      |
| 7th        | 34             | \-inf      | 53             | 12.9       | 68             | 11.5       | 34.3      |
| 8th        | 18             | 4.2        | 69             | 10.7       | 55             | 12.0       | 9.3       |
| 9th        | 28             | \-inf      | 70             | 11.6       | 63             | 11.7       | 18.1      |
| 10th       | 26             | \-inf      | 63             | 13.5       | 66             | 13.4       | 18.0      |
| 11th       | 20             | 7.5        | 42             | 13.2       | 49             | 13.1       | 3.8       |
| 12th       | 22             | \-inf      | 68             | 10.9       | 63             | 13.5       | -         |
| 13th       | 18             | \-inf      | 67             | 12.6       | 66             | 12.1       | 9.2       |
| 14th       | 26             | 9.1        | 55             | 12.7       | 64             | 11.8       | 11.0      |
| 15th       | 15             | \-inf      | 52             | 13.1       | 55             | 11.8       | 8.1       |
| 16th       | 32             | 15.8       | 69             | 11.6       | 67             | 11.9       | 3.2       |
| 17th       | 21             | 7.9        | 69             | 9.1        | 57             | 11.3       | -         |
| 18th       | 22             | 82.4       | 57             | 12.1       | 44             | 10.8       | 8.3       |
| **AVG**    | **22.1**       | **21.4**   | **59.6**       | **12.5**   | **55.6**       | **12.3**   | **12.4**  |
| **STDDEV** | -              | **23.59**  | -              | **1.63**   | -              | **1.04**   | **7.33**  |

### Observation 1: TTC camera performs bad when keypoint matches are too little

Based on the table above, **ORB-ORB** fails to estimate a reasonable TTC (-inf) for 8 times and its standard deviation reaches 23.59 sec, hence it performs the worst. The main reason is the number of matches are too little to compute a reliable estimation of TTC. The average number of keypoint matchers by **ORB-ORB** is only 22.1, while **FAST-BRIEF** and **FAST-ORB** are 59.6 and 55.6.

### Observation 2: FAST-BRIEF and FAST-ORB has similar average TTC as Lidar TTC

FAST-BRIEF and FAST-ORB has average of TTC of 12.5 sec and 12.3, while Lidar TTC has an average of 12.4. This implies that their average TTC is most likely reliable.

### Observation 3: FAST-BRIEF and FAST-ORB has smaller TTC standard deviation than Lidar TTC

FAST-BRIEF and FAST-ORB has standard deviation of TTC of 1.63 sec and 1.04, while Lidar TTC is 7.33. This shows the TTC by camera is much better than Lidar TTC when there is enough keypoint matches.



