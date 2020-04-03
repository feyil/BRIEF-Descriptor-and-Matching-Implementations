# Harris Corner Detection Algorithm Implementation

### Introduction

* This repo includes my solution of the given homework(5/4) in the scope of the Introduction to Image Understanding(CENG391) course which is given as a technical elective in 2019-2020 Fall semester by Computer Engineering Department at Izmir Institute of Technology.

* (*)README.md file uses some parts of the official Homework Doc to better express the purpose of the Homework.

* All solutions implemented on top of the base code.

### Problem*

#### Exercise 1

* Write a new member function Image::compute_brief that takes a reference to a std::vector of keypoints. It should then compute the BRIEF descriptor for each keypoint if possible and return a std::vector of the following structure:

```C++
struct Descriptor {
    int key_id;
    uchar desc[32];
};
```

#### Exercise 2

* Write a new static member function Image::match_brief that takes two references to std::vector of Descriptor. It should then compute the BRIEF descriptor matches from the first one to the second and return a std::vector of the following structure:

```C++
struct Match {
    int key_id0;
    int key_id1;
    int distance;
};
```

### Implementation and Result Showcase

* I have prepared a sh script to add small automation to my compile process. Therefore you can compile given files with this script. Compilation process uses valgrind to check and find any memory leak my occur and this check takes time to complete.If you want you can disable it inside of the compile.sh file.

* I have implemented methods in the problem statement and some others to help me to solve the problems more effectively.

#### Setup

* When you compile with compile.sh it runs the program once with valgrind. After that you can also run it yourself.

```bash
$ sh compile.sh
$ cd build
$ ./image-test
```

* You can play with image_test.cc to get different results. However don't forget to compile whenever you made a change :)

```C++
int main(int argc, char** argv)
{
     Image img(4, 4, 1);
     img.read_pnm("../small_city.pgm");

     Image img_rotated(img.w() * 2,img.h() * 2,1);
     img_rotated.read_pnm("../small_city.pgm");
     double theta = 90 * 3.1415926 / 180;
     img.rotate_centered(&img_rotated, theta);

     float threshold = 1000.0f;
     float k = 0.06f;
     float sigma = 2.5f;
     
     // Keypoint calculation for img
     vector<Keypoint> keys = img.harris_corners(threshold, k, sigma);
     cout << "Detected " << keys.size()
          << " keypoints on small_city.pgm" << endl;
     Image *key_image = make_keypoint_image(&img, &keys);
     key_image->write_pnm("/tmp/keys");
     //end
      
     // Keypoint calculation for img_rotated
     vector<Keypoint> keys_rotated = img_rotated.harris_corners(threshold, k, sigma);
     cout << "Detected " << keys_rotated.size()
          << " rotated "<<theta<<" keypoints on small_city.pgm" << endl;
     Image *key_image_rotated = make_keypoint_image(&img_rotated, &keys_rotated);
     key_image_rotated->write_pnm("/tmp/keys_rotated");
     //end

     // Calculating descriptor for img
     vector<Descriptor> descriptorVec =  img.compute_brief(keys);

     // Calculating descriptor for img_rotated
     vector<Descriptor> descriptorVec2 =  img.compute_brief(keys_rotated);

     // Matching
     vector<Match> matches =  Image::match_brief(descriptorVec, descriptorVec2);

     // Printing matches to the console
     for(int i = 0; i < matches.size(); i++) {
          cout<<"Match "<<i<<"\n\tkey_id0:"<<matches[i].key_id0
          <<"\n\tkey_id1:"<<matches[i].key_id1
          <<"\n\tdistance:"<<matches[i].distance<<endl;
     }

     delete key_image;
     delete key_image_rotated;
     
     return EXIT_SUCCESS;
}

```

![alt text](https://github.com/feyil/Harris-Corner-Detection-Implementation/blob/master/screenshots/brief.png "brief")