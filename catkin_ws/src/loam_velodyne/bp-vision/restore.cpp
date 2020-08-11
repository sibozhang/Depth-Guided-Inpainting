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

#define ITER 5       // number of BP iterations at each scale
#define LEVELS 5     // number of scales

#define DISC_K 200.0F       // truncation of discontinuity cost
#define DATA_K 10000.0F     // truncation of data cost
#define LAMBDA 0.05F         // weighting of data cost

#define INF 1E10     // large cost
#define VALUES 256   // number of possible graylevel values

// dt of 1d function
static float *dt(float *f, int n) {
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;

  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+square(q))-(f[v[k]]+square(v[k]))) / (2*(q-v[k]));
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+square(q))-(f[v[k]]+square(v[k]))) / (2*(q-v[k]));
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }
  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    d[q] = square(q-v[k]) + f[v[k]];
  }
  delete [] v;
  delete [] z;
  return d;
}

// compute message
void msg(float s1[VALUES], float s2[VALUES], 
	 float s3[VALUES], float s4[VALUES],
	 float dst[VALUES]) {
  // aggregate and find min
  float minimum = INF;
  for (int value = 0; value < VALUES; value++) {
    dst[value] = s1[value] + s2[value] + s3[value] + s4[value];
    if (dst[value] < minimum)
      minimum = dst[value];
  }

  // dt
  float *tmp = dt(dst, VALUES);

  // truncate and store in destination vector
  minimum += DISC_K;
  for (int value = 0; value < VALUES; value++)
    dst[value] = std::min(tmp[value], minimum);

  // normalize
  float val = 0;
  for (int value = 0; value < VALUES; value++) 
    val += dst[value];

  val /= VALUES;
  for (int value = 0; value < VALUES; value++) 
    dst[value] -= val;

  delete tmp;
}

// computation of data costs
image<float[VALUES]> *comp_data(image<uchar> *img) {
  int width = img->width();
  int height = img->height();
  image<float[VALUES]> *data = new image<float[VALUES]>(width, height);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      for (int value = 0; value < VALUES; value++) {
	float val = square((float)(imRef(img, x, y)-value));
	imRef(data, x, y)[value] = LAMBDA * std::min(val, DATA_K);
      }
    }
  }

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
      imRef(out, x, y) = best;
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
image<uchar> *restore_ms(image<uchar> *img) {
  image<float[VALUES]> *u[LEVELS];
  image<float[VALUES]> *d[LEVELS];
  image<float[VALUES]> *l[LEVELS];
  image<float[VALUES]> *r[LEVELS];
  image<float[VALUES]> *data[LEVELS];

  // data costs
  data[0] = comp_data(img);

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
  image<uchar> *img, *out, *edges;

  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " in(pgm) out(pgm)\n";
    exit(1);
  }

  // load input
  img = loadPGM(argv[1]);

  // restore
  out = restore_ms(img);

  // save output
  savePGM(out, argv[2]);
  
  delete img;
  delete out;
  return 0;
}
