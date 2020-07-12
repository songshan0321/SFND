# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.



## Performance Evaluation

### TASK MP.7 - the number of keypoints on the preceding vehicle for all 10 images

Your seventh task is to count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

| DETECTOR/IMAGE | 1    | 2    | 3    | 4    | 5    | 6    | 7    | 8    | 9    | 10   | AVERAGE |
| -------------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ------- |
| SHITOMASI      | 127  | 120  | 123  | 120  | 120  | 115  | 114  | 125  | 112  | 113  | 118.9   |
| HARRIS         | 17   | 14   | 18   | 21   | 26   | 43   | 18   | 31   | 26   | 34   | 24.8    |
| FAST           | 149  | 152  | 152  | 157  | 149  | 150  | 157  | 152  | 139  | 144  | 150.1   |
| BRISK          | 254  | 274  | 276  | 275  | 293  | 278  | 289  | 268  | 260  | 250  | 271.7   |
| ORB            | 91   | 102  | 106  | 113  | 109  | 124  | 129  | 127  | 124  | 125  | 115     |
| AKAZE          | 162  | 157  | 159  | 154  | 162  | 163  | 173  | 175  | 175  | 175  | 165.5   |
| SIFT           | 137  | 131  | 122  | 135  | 136  | 141  | 136  | 149  | 156  | 136  | 137.9   |

### TASK MP.8 - the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors

Your eighth task is to count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, use the BF approach with the descriptor distance ratio set to 0.8.

| COMB./IMAGE       | 1-2    | 2-3     | 3-4    | 4-5     | 5-6    | 6-7     | 7-8     | 8-9     | 9-10    | AVERAGE   |
| ----------------- | ------ | ------- | ------ | ------- | ------ | ------- | ------- | ------- | ------- | --------- |
| SHITOMASI - BRISK | 84     | 80      | 74     | 79      | 71     | 70      | 78      | 81      | 74      | 76.8      |
| SHITOMASI - BRIEF | 97     | 94      | 92     | 89      | 95     | 93      | 88      | 92      | 87      | 91.9      |
| SHITOMASI - ORB   | 89     | 86      | 87     | 91      | 89     | 78      | 81      | 86      | 88      | 86.1      |
| SHITOMASI - FREAK | 66     | 67      | 63     | 64      | 63     | 64      | 61      | 67      | 64      | 64.3      |
| SHITOMASI - AKAZE | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | \-        |
| SHITOMASI - SIFT  | 115    | 109     | 104    | 103     | 101    | 101     | 97      | 107     | 99      | 104.0     |
| HARRIS - BRISK    | 11     | 9       | 10     | 11      | 16     | 14      | 12      | 21      | 17      | 13.4      |
| HARRIS - BRIEF    | 12     | 12      | 14     | 17      | 17     | 16      | 12      | 20      | 21      | 15.7      |
| HARRIS - ORB      | 11     | 11      | 14     | 17      | 19     | 19      | 13      | 21      | 20      | 16.1      |
| HARRIS - FREAK    | 11     | 9       | 13     | 14      | 13     | 18      | 10      | 17      | 18      | 13.7      |
| HARRIS- AKAZE     | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | \-        |
| HARRIS - SIFT     | 14     | 11      | 16     | 19      | 22     | 22      | 13      | 24      | 22      | 18.1      |
| FAST - BRISK      | 79     | 92      | 89     | 93      | 73     | 84      | 95      | 86      | 97      | 87.6      |
| **FAST - BRIEF**  | **90** | **103** | **99** | **103** | **96** | **101** | **115** | **107** | **92**  | **100.7** |
| **FAST - ORB**    | **97** | **104** | **91** | **94**  | **92** | **102** | **104** | **95**  | **100** | **97.7**  |
| FAST - FREAK      | 63     | 82      | 67     | 79      | 66     | 77      | 84      | 77      | 84      | 75.4      |
| FAST - AKAZE      | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | -         |
| FAST - SIFT       | 118    | 122     | 114    | 120     | 115    | 120     | 124     | 118     | 104     | 117.2     |
| BRISK - BRISK     | 140    | 142     | 139    | 142     | 141    | 158     | 139     | 148     | 156     | 145.0     |
| BRISK - BRIEF     | 135    | 164     | 134    | 147     | 150    | 159     | 162     | 161     | 151     | 151.4     |
| BRISK - ORB       | 98     | 104     | 84     | 99      | 91     | 114     | 112     | 115     | 120     | 104.1     |
| BRISK - FREAK     | 113    | 118     | 121    | 116     | 105    | 136     | 138     | 129     | 131     | 123.0     |
| BRISK - AKAZE     | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | \-        |
| BRISK - SIFT      | 177    | 189     | 175    | 181     | 169    | 194     | 199     | 180     | 187     | 183.4     |
| ORB - BRISK       | 60     | 65      | 65     | 76      | 72     | 82      | 82      | 72      | 72      | 71.8      |
| ORB - BRIEF       | 37     | 38      | 38     | 53      | 42     | 61      | 60      | 61      | 60      | 50.0      |
| **ORB - ORB**     | **38** | **57**  | **49** | **52**  | **59** | **69**  | **70**  | **66**  | **71**  | **59.0**  |
| ORB - FREAK       | 38     | 33      | 37     | 41      | 33     | 40      | 41      | 40      | 43      | 38.4      |
| ORB - AKAZE       | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | \-        |
| ORB - SIFT        | 66     | 79      | 78     | 79      | 82     | 93      | 94      | 93      | 92      | 84.0      |
| AKAZE - BRISK     | 122    | 113     | 120    | 119     | 117    | 119     | 133     | 139     | 127     | 123.2     |
| AKAZE - BRIEF     | 107    | 115     | 110    | 107     | 117    | 129     | 134     | 136     | 129     | 120.4     |
| AKAZE - ORB       | 98     | 97      | 96     | 84      | 95     | 122     | 106     | 112     | 119     | 103.2     |
| AKAZE - FREAK     | 102    | 105     | 95     | 98      | 100    | 117     | 126     | 117     | 115     | 108.3     |
| AKAZE - AKAZE     | 127    | 128     | 124    | 119     | 125    | 131     | 138     | 139     | 142     | 130.3     |
| AKAZE - SIFT      | 134    | 136     | 129    | 138     | 139    | 149     | 149     | 154     | 150     | 142.0     |
| SIFT - BRISK      | 56     | 62      | 56     | 61      | 57     | 55      | 56      | 63      | 75      | 60.1      |
| SIFT - BRIEF      | 64     | 68      | 63     | 65      | 53     | 59      | 73      | 69      | 84      | 66.4      |
| SIFT - ORB        | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | \-        |
| SIFT - FREAK      | 59     | 61      | 52     | 63      | 52     | 51      | 51      | 53      | 67      | 56.6      |
| SIFT - AKAZE      | \-     | \-      | \-     | \-      | \-     | \-      | \-      | \-      | \-      | \-        |
| SIFT - SIFT       | 81     | 80      | 84     | 95      | 92     | 83      | 83      | 101     | 104     | 89.2      |

## TASK MP.9

Your ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles. Finally, in a short text, please justify your recommendation based on your observations and on the data you collected.
| COMB. /IMAGE      | 1-2      | 2-3      | 3-4      | 4-5      | 5-6      | 6-7      | 7-8      | 8-9      | 9-10     | AVERAGE  |
| ----------------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |
| SHITOMASI - BRISK | 270.1    | 265.7    | 266.9    | 262.8    | 266.1    | 263.3    | 263.2    | 275.8    | 281      | 268.3    |
| SHITOMASI - BRIEF | 27       | 26.7     | 29       | 28.4     | 27.4     | 27.3     | 25.7     | 26       | 24.5     | 26.9     |
| SHITOMASI - ORB   | 29.4     | 29.6     | 29.6     | 29       | 27.2     | 25.7     | 28.3     | 30       | 28.9     | 28.6     |
| SHITOMASI - FREAK | 62.9     | 61.2     | 60.2     | 59.4     | 58.1     | 58.7     | 60.3     | 60.4     | 59.4     | 60.1     |
| SHITOMASI - AKAZE | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| SHITOMASI - SIFT  | 40.7     | 40.5     | 39.7     | 40.7     | 38.6     | 38.7     | 40.8     | 41.6     | 39.4     | 40.1     |
| HARRIS - BRISK    | 252      | 248.8    | 253.5    | 253      | 268.7    | 259.2    | 260.6    | 250.1    | 257.6    | 255.9    |
| HARRIS - BRIEF    | 14.3     | 12.5     | 10.4     | 12.1     | 21.7     | 10.2     | 15.4     | 11.4     | 15.2     | 13.7     |
| HARRIS - ORB      | 14.4     | 12.1     | 14.6     | 15.6     | 26.7     | 15.1     | 16.4     | 16       | 20       | 16.8     |
| HARRIS - FREAK    | 48.2     | 45       | 45.9     | 45.2     | 56.8     | 43.9     | 47.8     | 45.6     | 49.9     | 47.6     |
| HARRIS- AKAZE     | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| HARRIS - SIFT     | 26.2     | 24.9     | 25.3     | 27.6     | 36.8     | 25.8     | 27.4     | 25.9     | 33.1     | 28.1     |
| FAST - BRISK      | 268.7    | 268.4    | 267.8    | 267.8    | 283.3    | 274.7    | 267.5    | 271.5    | 270.5    | 271.1    |
| **FAST - BRIEF**  | **33.3** | **30.4** | **30.2** | **29.2** | **29**   | **29.2** | **26.7** | **28.2** | **28.3** | **29.4** |
| **FAST - ORB**    | **30.5** | **29**   | **30**   | **29.2** | **29.4** | **29**   | **26.2** | **27.8** | **28.6** | **28.9** |
| FAST - FREAK      | 67.1     | 64.5     | 64       | 73.6     | 64.4     | 63.4     | 61       | 65       | 62.3     | 65.0     |
| FAST - AKAZE      | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| FAST - SIFT       | 53.5     | 52.9     | 48.7     | 47       | 48.1     | 48.4     | 49.1     | 51.9     | 46.8     | 49.6     |
| BRISK - BRISK     | 592.4    | 589.4    | 589.9    | 573.7    | 577.5    | 571.4    | 568.4    | 568.6    | 577.6    | 578.8    |
| BRISK - BRIEF     | 345.5    | 352.3    | 350.9    | 341.1    | 336.1    | 341.9    | 347.5    | 360.5    | 347.7    | 347.1    |
| BRISK - ORB       | 339.7    | 335.7    | 336.2    | 351.4    | 352.4    | 341      | 331.4    | 339.7    | 360.5    | 343.1    |
| BRISK - FREAK     | 372.7    | 399.4    | 377.3    | 369.7    | 382      | 390.3    | 383.8    | 375.3    | 377.7    | 380.9    |
| BRISK - AKAZE     | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| BRISK - SIFT      | 376.9    | 369.7    | 370.7    | 408.5    | 375.4    | 373.9    | 363.2    | 372.3    | 382.6    | 377.0    |
| ORB - BRISK       | 247.9    | 248.6    | 247.8    | 249.4    | 268.1    | 274.2    | 252.3    | 249.2    | 250.2    | 254.2    |
| ORB - BRIEF       | 10.3     | 10.7     | 9.7      | 9.7      | 9.4      | 9.6      | 10.5     | 9.8      | 10.1     | 10.0     |
| **ORB - ORB**     | **14.1** | **15.2** | **13.8** | **14.5** | **14**   | **13.9** | **15.4** | **14.8** | **13.6** | **14.4** |
| ORB - FREAK       | 47.3     | 44.3     | 45.5     | 43.1     | 43.5     | 43.3     | 44.3     | 44.9     | 43.7     | 44.4     |
| ORB - AKAZE       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| ORB - SIFT        | 47.8     | 51.6     | 50.4     | 48.5     | 49       | 52.7     | 53.7     | 56.8     | 53.5     | 51.6     |
| AKAZE - BRISK     | 311.1    | 315.7    | 315.5    | 341.5    | 320.5    | 318.2    | 320.3    | 318.7    | 314.3    | 319.5    |
| AKAZE - BRIEF     | 71.9     | 72.2     | 74.9     | 74.4     | 74.3     | 75.2     | 75.6     | 96.2     | 85.7     | 77.8     |
| AKAZE - ORB       | 78.9     | 75.9     | 77.6     | 73.4     | 79.1     | 91.9     | 80.2     | 87.9     | 84.1     | 81.0     |
| AKAZE - FREAK     | 107.8    | 99.1     | 98.8     | 108.5    | 108.5    | 102.7    | 108.9    | 108.2    | 111.8    | 106.0    |
| AKAZE - AKAZE     | 126.1    | 120.5    | 127.3    | 125.6    | 120.6    | 122.1    | 122.9    | 124.4    | 122      | 123.5    |
| AKAZE - SIFT      | 92.1     | 95.5     | 94.5     | 95.5     | 99.2     | 98.2     | 101.2    | 110.3    | 99.5     | 98.4     |
| SIFT - BRISK      | 351.5    | 321.6    | 319.9    | 319.5    | 324.1    | 321.2    | 320.2    | 326.6    | 323.4    | 325.3    |
| SIFT - BRIEF      | 117.7    | 100.5    | 100.3    | 115.4    | 101.5    | 102.9    | 100.8    | 104.9    | 106.1    | 105.6    |
| SIFT - ORB        | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| SIFT - FREAK      | 149.2    | 133.7    | 132.1    | 137.7    | 132.8    | 137.7    | 157      | 146.5    | 143.8    | 141.2    |
| SIFT - AKAZE      | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       | \-       |
| SIFT - SIFT       | 173.3    | 171.7    | 171.2    | 171      | 189.5    | 174.5    | 172.3    | 215.6    | 201.2    | 182.3    |

### TOP3 Detector / Descriptor Combinations

**Constraint due to requirements for real-time collision detection:**

1. Real-Time: > 30 fps (< 33.3 ms)
2. Number of matched points: 50

**Only 5 combinations satisfy the constraints above, they are:**

- SHITOMASI - BRIEF
- SHITOMASI - ORB
- FAST - BRIEF
- FAST - ORB
- ORB - ORB

**Performance Score Definition**: (1/Average time) * (Average matches)

| TOP3 COMBINATION | AVERAGE TIME (ms) | AVERAGE MATCH | Score |
| ---------------- | ----------------- | ------------- | ----- |
| ORB - ORB        | 14.4              | 98            | 6.8   |
| FAST - BRIEF     | 29.4              | 100.7         | 3.4   |
| FAST - ORB       | 28.9              | 97.7          | 3.4   |