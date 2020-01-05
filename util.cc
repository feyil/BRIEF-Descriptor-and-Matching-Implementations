// ------------------------------
// Written by Mustafa Ozuysal
// Contact <mustafaozuysal@iyte.edu.tr> for comments and bug reports
// ------------------------------
// Copyright (c) 2019, Mustafa Ozuysal
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
#include "util.hpp"

#include <cstdio>
#include <cmath>
#include <memory>

#include "image.hpp"

using std::printf;
using std::max;
using std::ceil;
using std::exp;
using std::unique_ptr;

namespace ceng391 {

Image *short_to_image(const short *ptr, int width, int height)
{
        Image *img = Image::new_gray(width, height);
        for (int y = 0; y < height; ++y) {
                const short *row = ptr + y * width;
                uchar *irow = img->data(y);
                for (int x = 0; x < width; ++x) {
                        irow[x] = (row[x] + 255) / 2;
                }
        }

        return img;
}

float *gaussian_kernel(float sigma, int *k)
{
        int l = ceil(2.0f * sigma);
        *k = 2 * l + 1;
        float *kernel = new float[*k];
        float sum = 0.0f;
        for (int i = 0; i < *k; ++i) {
                int x = i - l;
                kernel[i] = exp(-0.5f * x * x / sigma / sigma);
                sum += kernel[i];
        }
        for (int i = 0; i < *k; ++i)
                kernel[i] /= sum;

        return kernel;
}

void convolve_buffer(float *buffer, int n, const float *kernel, int k)
{
        for (int i = 0; i < n; ++i) {
                float sum = 0.0f;
                for (int j = 0; j < k; ++j) {
                        sum += kernel[j] * buffer[i + j];
                }
                buffer[i] = sum;
        }
}

short *vec_mul(int n, const short *v0, const short *v1)
{
        short *p = new short[n];
        for (int i = 0; i < n; ++i) {
                p[i] = v0[i] * v1[i];
        }
        return p;
}

void smooth_short_buffer(int w, int h, short *I, float sigma)
{
        int k = 0;
        unique_ptr<float []> kernel(gaussian_kernel(sigma, &k));

        int l = k / 2;
        int max_wh = max(w, h);
        unique_ptr<float []>  buffer(new float[max_wh + 2 * l]);

        for (int y = 0; y < h - 1; ++y) {
                copy_to_buffer<short>(buffer.get(), I + y * w, w, l, 1);
                convolve_buffer(buffer.get(), w, kernel.get(), k);
                copy_from_buffer<short>(I + y * w, buffer.get(), w, 1);
        }

        for (int x = 0; x < w - 1; ++x) {
                copy_to_buffer<short>(buffer.get(), I + x, h, l, w);
                convolve_buffer(buffer.get(), h, kernel.get(), k);
                copy_from_buffer<short>(I + x, buffer.get(), h, w);
        }
}

float *harris_corner_score(int w, int h, const short *Ix2, const short *Iy2,
                           const short *IxIy, float k)
{
        float *score = new float[w * h];
        for (int y = 0; y < h; ++y) {
                const short *A = Ix2 + y * w;
                const short *B = IxIy + y * w;
                const short *C = Iy2 + y * w;
                float *R = score + y * w;
                for (int x = 0; x < w; ++x) {
                        float det = A[x] * C[x] - B[x] * B[x];
                        float tr = A[x] + C[x];
                        R[x] = det - k * tr * tr;
                }
        }

        return score;
}

static inline void paint_pixel(Image *img, int x, int y, uchar *color)
{
        img->data(y)[3*x] = color[0];
        img->data(y)[3*x + 1] = color[1];
        img->data(y)[3*x + 2] = color[2];
}

Image *make_keypoint_image(Image *img, std::vector<Keypoint> *keys)
{
        Image *rgb = Image::new_copy(img);
        rgb->to_rgb();

        uchar color[3] = { 255, 0, 0 };
        for (size_t i = 0; i < keys->size(); ++i) {
                int x = (*keys)[i].x;
                int y = (*keys)[i].y;

                if (x < 2 || x >= rgb->w() - 2
                    || y < 2 || y >= rgb->h()) {
                        continue;
                }

                paint_pixel(rgb, x - 2, y, &color[0]);
                paint_pixel(rgb, x + 2, y, &color[0]);
                paint_pixel(rgb, x - 1, y - 1, &color[0]);
                paint_pixel(rgb, x + 1, y - 1, &color[0]);
                paint_pixel(rgb, x, y - 2, &color[0]);
                paint_pixel(rgb, x - 1, y + 1, &color[0]);
                paint_pixel(rgb, x + 1, y + 1, &color[0]);
                paint_pixel(rgb, x, y + 2, &color[0]);
        }

        return rgb;
}

}
