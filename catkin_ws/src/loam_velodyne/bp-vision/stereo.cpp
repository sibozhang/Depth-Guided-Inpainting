/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include <cstdio>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include <image.h>
#include <misc.h>
#include <pnmfile.h>
#include <filter.h>
#include <imconv.h>

#define ITER 5       // number of BP iterations at each scale
#define LEVELS 5     // number of scales

#define DISC_K 1.7F         // truncation of discontinuity cost
#define DATA_K 15.0F        // truncation of data cost
#define LAMBDA 0.07F        // weighting of data cost

#define INF 1E20     // large cost
#define VALUES 16    // number of possible disparities
#define SCALE 16     // scaling from disparity to graylevel in output

#define SIGMA 0.7    // amount to smooth the input images

// dt of 1d function
static void dt(float f[VALUES]) {
  for (int q = 1; q < VALUES; q++) {
    float prev = f[q-1] + 1.0F;
    if (prev < f[q])
      f[q] = prev;
  }
  for (int q = VALUES-2; q >= 0; q--) {
    float prev = f[q+1] + 1.0F;
    if (prev < f[q])
      f[q] = prev;
  }
}
  
// compute message
void msg(float s1[VALUES], float s2[VALUES], 
	 float s3[VALUES], float s4[VALUES],
	 float dst[VALUES]) {
  float val;

  // aggregate and find min
  float minimum = INF;
  for (int value = 0; value < VALUES; value++) {
    dst[value] = s1[value] + s2[value] + s3[value] + s4[value];
    if (dst[value] < minimum)
      minimum = dst[value];
  }

  // dt
  dt(dst);

  // truncate 
  minimum += DISC_K;
  for (int value = 0; value < VALUES; value++)
    if (minimum < dst[value])
      dst[value] = minimum;

  // normalize
  val = 0;
  for (int value = 0; value < VALUES; value++) 
    val += dst[value];

  val /= VALUES;
  for (int value = 0; value < VALUES; value++) 
    dst[value] -= val;
}

// computation of data costs
image<float[VALUES]> *comp_data(image<uchar> *img1, image<uchar> *img2) {
  int width = img1->width();
  int height = img1->height();
  image<float[VALUES]> *data = new image<float[VALUES]>(width, height);

  image<float> *sm1, *sm2;
  if (SIGMA >= 0.1) {
    sm1 = smooth(img1, SIGMA);
    sm2 = smooth(img2, SIGMA);
  } else {
    sm1 = imageUCHARtoFLOAT(img1);
    sm2 = imageUCHARtoFLOAT(img2);
  } 

  for (int y = 0; y < height; y++) {
    for (int x = VALUES-1; x < width; x++) {
      for (int value = 0; value < VALUES; value++) {
	float val = abs(imRef(sm1, x, y)-imRef(sm2, x-value, y));	
	imRef(data, x, y)[value] = LAMBDA * std::min(val, DATA_K);
      }
    }
  }

  delete sm1;
  delete sm2;
  return data;
}

// generate output from current messages
image<uchar> *output(image<float[VALUES]> *u, image<float[VALUES]> *d, 
		     image<float[VALUES]> *l, image<float[VALUES]> *r, 
		     image<float[VALUES]> *data) {
  int width = data->width();
  int height = data->height();
  image<uchar> *out = new image<uchar>(width, height);

  for (int y = 1; y < height-1; y++) {
    for (int x = 1; x < width-1; x++) {
      // keep track of best value for current pixel
      int best = 0;
      float best_val = INF;
      for (int value = 0; value < VALUES; value++) {
	float val = 
	  imRef(u, x, y+1)[value] +
	  imRef(d, x, y-1)[value] +
	  imRef(l, x+1, y)[value] +
	  imRef(r, x-1, y)[value] +
	  imRef(data, x, y)[value];
	if (val < best_val) {
	  best_val = val;
	  best = value;
	}
      }
      imRef(out, x, y) = best * SCALE;
    }
  }

  return out;
}

// belief propagation using checkerboard update scheme
void bp_cb(image<float[VALUES]> *u, image<float[VALUES]> *d,
	   image<float[VALUES]> *l, image<float[VALUES]> *r,
	   image<float[VALUES]> *data,
	   int iter) {
  int width = data->width();  
  int height = data->height();

  for (int t = 0; t < ITER; t++) {
    std::cout << "iter " << t << "\n";

    for (int y = 1; y < height-1; y++) {
      for (int x = ((y+t) % 2) + 1; x < width-1; x+=2) {

	msg(imRef(u, x, y+1),imRef(l, x+1, y),imRef(r, x-1, y),
	    imRef(data, x, y), imRef(u, x, y));

	msg(imRef(d, x, y-1),imRef(l, x+1, y),imRef(r, x-1, y),
	    imRef(data, x, y), imRef(d, x, y));

	msg(imRef(u, x, y+1),imRef(d, x, y-1),imRef(r, x-1, y),
	    imRef(data, x, y), imRef(r, x, y));

	msg(imRef(u, x, y+1),imRef(d, x, y-1),imRef(l, x+1, y),
	    imRef(data, x, y), imRef(l, x, y));

      }
    }
  }
}

// multiscale belief propagation for image restoration
image<uchar> *stereo_ms(image<uchar> *img1, image<uchar> *img2) {
  image<float[VALUES]> *u[LEVELS];
  image<float[VALUES]> *d[LEVELS];
  image<float[VALUES]> *l[LEVELS];
  image<float[VALUES]> *r[LEVELS];
  image<float[VALUES]> *data[LEVELS];

  // data costs
  data[0] = comp_data(img1, img2);

  // data pyramid
  for (int i = 1; i < LEVELS; i++) {
    int old_width = data[i-1]->width();
    int old_height = data[i-1]->height();
    int new_width = (int)ceil(old_width/2.0);
    int new_height = (int)ceil(old_height/2.0);

    assert(new_width >= 1);
    assert(new_height >= 1);

    data[i] = new image<float[VALUES]>(new_width, new_height);
    for (int y = 0; y < old_height; y++) {
      for (int x = 0; x < old_width; x++) {
	for (int value = 0; value < VALUES; value++) {
	  imRef(data[i], x/2, y/2)[value] += imRef(data[i-1], x, y)[value];
	}
      }
    }
  }

  // run bp from coarse to fine
  for (int i = LEVELS-1; i >= 0; i--) {
    int width = data[i]->width();
    int height = data[i]->height();

    // allocate & init memory for messages
    if (i == LEVELS-1) {
      // in the coarsest level messages are initialized to zero
      u[i] = new image<float[VALUES]>(width, height);
      d[i] = new image<float[VALUES]>(width, height);
      l[i] = new image<float[VALUES]>(width, height);
      r[i] = new image<float[VALUES]>(width, height);
    } else {
      // initialize messages from values of previous level
      u[i] = new image<float[VALUES]>(width, height, false);
      d[i] = new image<float[VALUES]>(width, height, false);
      l[i] = new image<float[VALUES]>(width, height, false);
      r[i] = new image<float[VALUES]>(width, height, false);

      for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++) {
	  for (int value = 0; value < VALUES; value++) {
	    imRef(u[i], x, y)[value] = imRef(u[i+1], x/2, y/2)[value];
	    imRef(d[i], x, y)[value] = imRef(d[i+1], x/2, y/2)[value];
	    imRef(l[i], x, y)[value] = imRef(l[i+1], x/2, y/2)[value];
	    imRef(r[i], x, y)[value] = imRef(r[i+1], x/2, y/2)[value];
	  }
	}
      }      
      // delete old messages and data
      delete u[i+1];
      delete d[i+1];
      delete l[i+1];
      delete r[i+1];
      delete data[i+1];
    } 

    // BP
    bp_cb(u[i], d[i], l[i], r[i], data[i], ITER);    
  }

  image<uchar> *out = output(u[0], d[0], l[0], r[0], data[0]);

  delete u[0];
  delete d[0];
  delete l[0];
  delete r[0];
  delete data[0];

  return out;
}

int main(int argc, char **argv) {
  image<uchar> *img1, *img2, *out, *edges;

  if (argc != 4) {
    std::cerr << "usage: " << argv[0] << " left(pgm) right(pgm) out(pgm)\n";
    exit(1);
  }

  // load input
  img1 = loadPGM(argv[1]);
  img2 = loadPGM(argv[2]);

  // compute disparities
  out = stereo_ms(img1, img2);

  // save output
  savePGM(out, argv[3]);
  
  delete img1;
  delete img2;
  delete out;
  return 0;
}
