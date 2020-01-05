// 230201057

// ------------------------------
// Written by Mustafa Ozuysal
// Contact <mustafaozuysal@iyte.edu.tr> for comments and bug reports
// ------------------------------
// Copyright (c) 2018, Mustafa Ozuysal
// All rights reserved.
// ------------------------------
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holders nor the
//       names of his/its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// ------------------------------
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------
#include <cstdlib>
#include <iostream>

#include "image.hpp"
#include "brief_descriptor.hpp"

using ceng391::Image;
using ceng391::short_to_image;
using ceng391::Keypoint;
using ceng391::Descriptor;
using ceng391::Match;
using std::vector;
using std::cout;
using std::endl;

int main(int argc, char** argv)
{
     Image img(4, 4, 1);
     img.read_pnm("../small_city.pgm");

     Image img_rotated(img.w() * 2,img.h() * 2,1);
     img_rotated.read_pnm("../small_city.pgm");
     double theta = 0 * 3.1415926 / 180;
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

