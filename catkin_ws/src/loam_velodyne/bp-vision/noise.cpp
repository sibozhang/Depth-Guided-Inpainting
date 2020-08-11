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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <misc.h>
#include <image.h>
#include <pnmfile.h>

double gaussian() {
  double x;
  double x1;

  static double x2;
  static int x2_valid = 0;

  if (x2_valid) {
    x2_valid = 0;
    return x2;
  }

  /*
   * Algorithm P (Polar method for normal deviates),
   * Knuth, D., "The Art of Computer Programming", Vol. 2, 3rd Edition, p. 122
   */
  do {
    x1 = 2.0 * drand48() - 1.0;
    x2 = 2.0 * drand48() - 1.0;
    x = x1 * x1 + x2 * x2;
  } while (x >= 1.0);
  x1 *= sqrt((-2.0) * log(x) / x);
  x2 *= sqrt((-2.0) * log(x) / x);

  x2_valid = 1;
  return x1;
}

int main(int argv, char **argc) {  
  if (argv != 4) {
    fprintf(stderr, "usage: %s in(pgm) out(pgm) sigma\n", argc[0]);
    exit(1);
  }

  srand48(time(NULL));
  float sigma = atof(argc[3]);

  image<uchar> *im = loadPGM(argc[1]);
  image<uchar> *out = new image<uchar>(im->width(), im->height());
  for (int y = 0; y < im->height(); y++) {
    for (int x = 0; x < im->width(); x++) {
      double r = gaussian()*sigma;
      double v = imRef(im, x, y);
      imRef(out, x, y) = bound(vlib_round(v + r), 0, 255);
    } 
  }

  savePGM(out, argc[2]);
  return 0;
}
