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
#include "image.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <ostream>
#include <memory>

using std::cerr;
using std::clog;
using std::cos;
using std::min;
using std::endl;
using std::exp;
using std::ifstream;
using std::ios;
using std::memset;
using std::ofstream;
using std::sin;
using std::string;
using std::unique_ptr;

namespace ceng391 {

Image::Image(int width, int height, int n_channels, int step)
{
        m_width = width;
        m_height = height;
        m_n_channels = n_channels;

        m_step = m_width*m_n_channels;
        if (m_step < step)
                m_step = step;
        m_data = new uchar[m_step*height];
}

Image::~Image()
{
        delete [] m_data;
}

Image* Image::new_gray(int width, int height)
{
        return new Image(width, height, 1);
}

Image* Image::new_rgb(int width, int height)
{
        return new Image(width, height, 3);
}

Image *Image::new_copy(Image *img)
{
        Image *cpy = new Image(img->w(), img->h(), img->n_ch());
        for (int y = 0; y < img->h(); ++y)
                memcpy(cpy->data(y), img->data(y), img->w() * img->n_ch());
        return cpy;
}

void Image::set_rect(int x, int y, int width, int height, uchar red, uchar green, uchar blue)
{
        if (x < 0) {
                width += x;
                x = 0;
        }

        if (y < 0) {
                height += y;
                y = 0;
        }

        if (m_n_channels == 1) {
                int value = 0.3*red + 0.59*green + 0.11*blue;
                if (value > 255)
                        value = 255;
                for (int j = y; j < y+height; ++j) {
                        if (j >= m_height)
                                break;
                        uchar* row_data = data(j);
                        for (int i = x; i < x+width; ++i) {
                                if (i >= m_width)
                                        break;
                                row_data[i] = value;
                        }
                }
        } else if (m_n_channels == 3) {
                for (int j = y; j < y+height; ++j) {
                        if (j >= m_height)
                                break;
                        uchar* row_data = data(j);
                        for (int i = x; i < x+width; ++i) {
                                if (i >= m_width)
                                        break;
                                row_data[i*3]     = red;
                                row_data[i*3 + 1] = green;
                                row_data[i*3 + 2] = blue;
                        }
                }
        }
}

void Image::set_rect(int x, int y, int width, int height, uchar value)
{
        if (x < 0) {
                width += x;
                x = 0;
        }

        if (y < 0) {
                height += y;
                y = 0;
        }

        for (int j = y; j < y+height; ++j) {
                if (j >= m_height)
                        break;
                uchar* row_data = data(j);
                for (int i = x; i < x+width; ++i) {
                        if (i >= m_width)
                                break;
                        for (int c = 0; c < m_n_channels; ++c)
                                row_data[i*m_n_channels + c] = value;
                }
        }
}

void Image::to_grayscale()
{
        if (m_n_channels == 1) {
                return;
        } else if (m_n_channels == 3) {
                int new_step = m_width;
                uchar *new_data = new uchar[new_step * m_height];
                for (int y = 0; y < m_height; ++y) {
                        uchar *row_old = m_data + m_step * y;
                        uchar *row_new = new_data + new_step * y;
                        for (int x = 0; x < m_width; ++x) {
                                uchar red = row_old[3*x];
                                uchar green = row_old[3*x + 1];
                                uchar blue = row_old[3*x + 2];
                                int value = 0.3*red + 0.59*green + 0.11*blue;
                                if (value > 255)
                                        value = 255;
                                row_new[x] = value;
                        }
                }

                delete [] m_data;
                m_data = new_data;
                m_step = new_step;
                m_n_channels = 1;
        }
}

void Image::to_rgb()
{
        if (m_n_channels == 3) {
                return;
        } else if (m_n_channels == 1) {
                int new_step = m_width * 3;
                uchar *new_data = new uchar[new_step * m_height];
                for (int y = 0; y < m_height; ++y) {
                        uchar *row_old = m_data + m_step * y;
                        uchar *row_new = new_data + new_step * y;
                        for (int x = 0; x < m_width; ++x) {
                                uchar value = row_old[x];
                                row_new[3*x]     = value;
                                row_new[3*x + 1] = value;
                                row_new[3*x + 2] = value;
                        }
                }

                delete [] m_data;
                m_data = new_data;
                m_step = new_step;
                m_n_channels = 3;
        }
}

bool Image::write_pnm(const std::string& filename) const
{
        ofstream fout;

        string magic_head = "P5";
        string extended_name = filename + ".pgm";
        if (m_n_channels == 3) {
                magic_head = "P6";
                extended_name = filename + ".ppm";
        }

        fout.open(extended_name.c_str(), ios::out | ios::binary);
        if (!fout.good()) {
                cerr << "Error opening file " << extended_name << " for output!" << endl;
                return false;
        }

        fout << magic_head << "\n";
        fout << m_width << " " << m_height << " 255\n";
        for (int y = 0; y < m_height; ++y) {
                const uchar *row_data = data(y);
                fout.write(reinterpret_cast<const char*>(row_data), m_width*m_n_channels*sizeof(uchar));
        }
        fout.close();

        return true;
}

bool Image::read_pnm(const std::string& filename)
{
        ifstream fin(filename.c_str(), ios::in | ios::binary);
        if (!fin.good()) {
                cerr << "Error opening PNM file " << filename << endl;
                return false;
        }

        int width;
        int height;
        int max_val;
        int n_channels = 1;
        string head = "00";
        head[0] = fin.get();
        head[1] = fin.get();
        if (head == "P5") {
                clog << "Loading PGM Binary" << endl;
                n_channels = 1;
        } else if (head == "P6") {
                clog << "Loading PPM Binary" << endl;
                n_channels = 3;
        } else {
                cerr << "File " << filename << " is not a Binary PGM or PPM!" << endl;
                return false;
        }

        fin >> width;
        fin >> height;
        fin >> max_val;
        if (fin.peek() == '\n')
                fin.get();

        int step = width * n_channels;
        uchar *new_data = new uchar[step*height];
        for (int y = 0; y < height; ++y) {
                fin.read(reinterpret_cast<char*>(new_data + y*step), step*sizeof(uchar));
        }
        fin.close();

        delete [] m_data;
        m_data = new_data;
        m_width = width;
        m_height = height;
        m_step = step;
        m_n_channels = n_channels;

        return true;
}

short *Image::deriv_x() const
{
        if (m_n_channels == 3) {
                cerr << "Image derivatives only implemented for grayscale images!" << endl;
                return nullptr;
        }

        short *dx = new short[m_width * m_height];
        for (int y = 0; y < m_height; ++y) {
                const uchar *row = this->data(y);
                short *drow = dx + y * m_width;
                drow[0] = 0;
                for (int x = 1; x < m_width - 1; ++x) {
                        drow[x] = row[x + 1] - row[x - 1];
                }
                drow[m_width - 1] = 0;
        }

        return dx;
}

short *Image::deriv_y() const
{
        if (m_n_channels == 3) {
                cerr << "Image derivatives only implemented for grayscale images!" << endl;
                return nullptr;
        }

        short *dy = new short[m_width * m_height];

        memset(dy, 0, m_width * sizeof(*dy));
        for (int y = 1; y < m_height - 1; ++y) {
                const uchar *rowm = this->data(y - 1);
                const uchar *rowp = this->data(y + 1);
                short *drow = dy + y * m_width;
                for (int x = 0; x < m_width; ++x) {
                        drow[x] = rowp[x] - rowm[x];
                }
        }
        memset(dy + (m_height - 1) * m_width, 0, m_width * sizeof(*dy));

        return dy;
}

void Image::rotate(Image *rotated, double theta, double tx, double ty) const
{
        if (m_n_channels != 1) {
                cerr << "Rotate only works on grayscale images!" << endl;
                return;
        }
        rotated->to_grayscale();

        double ct = cos(theta);
        double st = sin(theta);
        double tx_inv = -ct * tx + st * ty;
        double ty_inv = -st * tx - ct * ty;

        int wp = rotated->w();
        int hp = rotated->h();

        for (int yp = 0; yp < hp; ++yp) {
                uchar *row_p = rotated->data(yp);
                for (int xp = 0; xp < wp; ++xp) {
                        double x = ct * xp - st * yp + tx_inv;
                        double y = st * xp + ct * yp + ty_inv;

                        int x0 = static_cast<int>(x);
                        int y0 = static_cast<int>(y);

                        int value = 0;
                        if (x0 < 0 || y0 < 0 || x0 >= m_width || y0 >= m_height) {
                                value = 0;
                        } else {
                                const uchar *row = this->data(y0);
                                value = row[x0];
                        }

                        row_p[xp] = value;
                }
        }
}

void Image::rotate_centered(Image *rotated, double theta) const
{
        double ct = cos(theta);
        double st = sin(theta);
        double hw = m_width / 2.0;
        double hh = m_height / 2.0;
        double hwp = rotated->w() / 2.0;
        double hhp = rotated->h() / 2.0;

        double tx_cap = -ct * hw - st * hh + hwp;
        double ty_cap =  st * hw - ct * hh + hhp;
        this->rotate(rotated, theta, tx_cap, ty_cap);
}

void Image::smooth_x(float sigma)
{
        if (m_n_channels != 1) {
                cerr << "Smooth-x only works on grayscale images!" << endl;
                return;
        }

        int k = 0;
        unique_ptr<float []> kernel(gaussian_kernel(sigma, &k));

        int l = k / 2;
        unique_ptr<float []>  buffer(new float[m_width + 2 * l]);

        for (int y = 0; y < m_height - 1; ++y) {
                copy_to_buffer(buffer.get(), this->data(y), m_width, l, 1);
                convolve_buffer(buffer.get(), m_width, kernel.get(), k);
                copy_from_buffer(this->data(y), buffer.get(), m_width, 1);
        }
}

void Image::smooth_y(float sigma)
{
        if (m_n_channels != 1) {
                cerr << "Smooth-x only works on grayscale images!" << endl;
                return;
        }

        int k = 0;
        unique_ptr<float []> kernel(gaussian_kernel(sigma, &k));

        int l = k / 2;
        unique_ptr<float []>  buffer(new float[m_height + 2 * l]);

        for (int x = 0; x < m_width - 1; ++x) {
                copy_to_buffer(buffer.get(), m_data + x, m_height, l, m_step);
                convolve_buffer(buffer.get(), m_height, kernel.get(), k);
                copy_from_buffer(m_data + x, buffer.get(), m_height, m_step);
        }
}

void Image::smooth(float sigma_x, float sigma_y)
{
        smooth_x(sigma_x);
        smooth_y(sigma_y);
}

std::vector<Keypoint> Image::harris_corners(float threshold, float k,
                                            float sigma)
{
        short *Ix = this->deriv_x();
        short *Iy = this->deriv_y();

        int n = m_width * m_height;
        short *Ix2  = vec_mul(n, Ix, Ix);
        short *Iy2  = vec_mul(n, Iy, Iy);
        short *IxIy = vec_mul(n, Ix, Iy);

        smooth_short_buffer(m_width, m_height, Ix2, sigma);
        smooth_short_buffer(m_width, m_height, Iy2, sigma);
        smooth_short_buffer(m_width, m_height, IxIy, sigma);

        const float *score = harris_corner_score(m_width, m_height,
                                           Ix2, Iy2, IxIy, k);

        std::vector<Keypoint> keys;
        int border = min(1, static_cast<int>(2.0f * sigma));
        for (int y = border; y < m_height - border; ++y) {
                const float *Rm = score + (y - 1) * m_width;
                const float *R  = score + y * m_width;
                const float *Rp = score + (y + 1) * m_width;
                for (int x = border; x < m_width - border; ++x) {
                        if (R[x] > threshold && R[x] > R[x - 1]
                            && R[x] > R[x + 1] && R[x] > Rm[x]
                            && R[x] > Rp[x]) {
                                Keypoint key;
                                key.x = x;
                                key.y = y;
                                key.score = R[x];
                                keys.push_back(key);
                        }
                }
        }

        delete [] score;
        delete [] IxIy;
        delete [] Iy2;
        delete [] Ix2;
        delete [] Iy;
        delete [] Ix;

        return keys;
}

std::vector<Descriptor> Image::compute_brief(const std::vector<Keypoint> &keypoints) 
{
        std::vector<Descriptor> descriptors;
        
        if(keypoints.size() == 0) {
                return descriptors;
        }

        Image* copy_img = new_copy(this);
        
        float sigma = 2.5f;
        copy_img->smooth(sigma, sigma);

        for(int i = 0; i < keypoints.size(); i++) {

                if(check_keypoint(keypoints[i], copy_img->w(), copy_img->h())) {

                        Descriptor descriptor;
                        descriptor.key_id = i;

                        for(int j = 0; j < DESCRIPTOR_SIZE; j++) {

                                int* offset = DESCRIPTOR_OFFSETS[j];

                                int x0 = keypoints[i].x + offset[0];
                                int y0 = keypoints[i].y + offset[1];

                                int x1 = keypoints[i].x + offset[2];
                                int y1 = keypoints[i].y + offset[3];
                                
                                uchar I0 = copy_img->data(y0)[x0];
                                uchar I1 = copy_img->data(y1)[x1];
                                
                                int descriptor_index = j / 8;
                      
                                if(I0 > I1) {
                                        descriptor.desc[descriptor_index] += 1;
                                }
                                // shift
                                descriptor.desc[descriptor_index] = descriptor.desc[descriptor_index] << 1;
                        }

                        descriptors.push_back(descriptor);
                }

        }

        cerr<<"BRIEF Descriptors Computed"<<endl;

        delete copy_img;
}

bool Image::check_keypoint(const Keypoint &keypoint, int width, int height) {
        int offset = 8;

        float x = keypoint.x;
        float y = keypoint.y;

        bool x_check = (x + offset) < width && (x - offset) >= 0;
        bool y_check = (y + offset) < height && (y - offset) >= 0;   

        return x_check && y_check;
}

std::vector<Match> Image::match_brief(const std::vector<Descriptor> &desVec1, const std::vector<Descriptor> &desVec2) {

        std::vector<Match> matches;

        for(int i = 0; i < desVec1.size(); i++) {
                int min_distance = 256;
                Descriptor min_match_i;
                Descriptor min_match_j;

                Descriptor des1 = desVec1[i];

                for(int j = 0; j < desVec2.size(); j++) {
                        Descriptor des2 = desVec2[j];

                        int distance = hamming_distance(des1.desc, des2.desc);

                        if(distance < min_distance) {
                                min_distance = distance;
                                min_match_i = des1;
                                min_match_j = des2;
                        }    
                }
               
                Match match;
                match.key_id0 = min_match_i.key_id;
                match.key_id1 = min_match_j.key_id;
                match.distance = min_distance;

                matches.push_back(match);
        }
      
        cerr<<"Matching Completed"<<endl;
        return matches;
}

}
