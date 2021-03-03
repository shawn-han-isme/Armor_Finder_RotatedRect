# Armor_Finder

| Author   |  Responsible part |
| ------ | -------------- |
| Xuyuan Han | Armor Plate Recognition and Classifier |
| Xiankun Zeng | PNP Ranging |

## 1. Environment

|Operating System|Runtime Library|
|-------|--------|
|Ubuntu16.04<br />Windows WSL|OpenCV 3.4.7<br />cmake 3.15.4|

- The images with size of **640×480** are used for image processing.

## 2. Program Compilation and Running

Ubuntu16.04（under the project folder）

```shell
mkdir build
cd build
cmake ..
make
sudo ./main
```

## 3. Files Structure

``` Files Structure
.
├── armor                   // main code of auto-aiming
│   ├── include             // auto-aiming header files
│   └── src                 // auto-aiming source code
├── CMakeLists.txt          // cmake project files
├── image                   // the image used by the test code
│   ├── dst                 // the original image
│   └── src                 // the image after process
├── main.cpp                // main.cpp
├── other                   // some other codes, such as timer, drawText
│   └── include             // other header files
├── README.md               // 
└── Video                   // the video files used for debugging the code and the screenshots of the classifier output
```

## 4. Operation Process of the Armor Plate Identification Program

- First perform hsv binarization on the image: assign the pixels that match the hsv color of the armor plate light bar to white, and assign other pixels to black, and use median filtering to make the image smooth
- Use edge extraction to obtain a rotating rectangular area that may be a light bar
- Filter out the light bar rectangle according to the aspect ratio of the rotated rectangle and the hsv brightness of the pixels of the rotated rectangle corresponding to the original image area
- Perform pairwise matching of all possible light bars, and filter according to the angle between the two light bars, the height ratio of the two light bars, the height difference between the centers of the two light bars, and the ratio of the distance between the centers of the two light bars aiiind the height of the light bars. Then we get the qualified light bar pairs.
- Extend the four outer vertices of the light bar pair up and down to the edge of the armor plate in proportion to get the armor plate quadrilateral
- Perform affine transformation on the screenshot of the quadrilateral area of the armor plate to obtain the armor plate image and submit it to the classifier to determine the armor plate and its ID
- Finally, put the framed armor plate quadrilateral into PNP to calculate the distance and angle

![avatar](./自瞄流程图.png)

## 6. code naming conventions

- Function name: use the camel case nomenclature with the first letter lowercase
- Type name: use the camel case nomenclature with the first letter capitalized
- Variable name: use underscore  separating  nomenclature
