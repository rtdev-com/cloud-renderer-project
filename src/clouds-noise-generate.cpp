#include "clouds.h"

/*
 * generateNoise, smoothNoise, turbulence from:
 * https://lodev.org/cgtutor/randomnoise.html
 */

std::vector<std::vector<std::vector<float>>> generateNoise(int dim) {
  std::vector<std::vector<std::vector<float>>> noise( dim , std::vector<std::vector<float>>( dim, std::vector<float>( dim, 0 ) ) );
  #pragma omp parallel
  {
    #pragma omp for
    for (int z = 0; z < dim; z++) {
      for (int y = 0; y < dim; y++) {
        for (int x = 0; x < dim; x++) {
          float r = ( Vector2f::Random().x() + 1 ) / 2.f; // [0, 1]
          noise[z][y][x] = r;
        }
      }
    }
  }

  return noise;
}

float smoothNoise(std::vector<std::vector<std::vector<float>>>& noise, int dim, float x, float y, float z) {
   //get fractional part of x and y
   float dx = x - int(x);
   float dy = y - int(y);
   float dz = z - int(z);

   //wrap around
   int x1 = x;
   int y1 = y;
   int z1 = z;

   //neighbor values
   int x2 = (x1 + dim - 1) % dim;
   int y2 = (y1 + dim - 1) % dim;
   int z2 = (z1 + dim - 1) % dim;

   float value = 0.0;
   // x1 y1 z1
   value += dx       * dy       * dz       * noise[z1][y1][x1];
   // x1 y1 z2
   value += dx       * dy       * (1.f-dz) * noise[z2][y1][x1];
   // x1 y2 z1
   value += dx       * (1.f-dy) * dz       * noise[z1][y2][x1];
   // x1 y2 z2
   value += dx       * (1.f-dy) * (1.f-dz) * noise[z2][y2][x1];
   // x2 y1 z1
   value += (1.f-dx) * dy       * dz       * noise[z1][y1][x2];
   // x2 y1 z2
   value += (1-dx)   * dy       * (1-dz)   * noise[z2][y1][x2];
   // x2 y2 z1
   value += (1-dx)   * (1-dy)   * dz       * noise[z1][y2][x2];
   // x2 y2 z2
   value += (1-dx)   * (1-dy)   * (1-dz)   * noise[z2][y2][x2];




   //smooth the noise with bilinear interpolation
   // float value = 0.0;
   // value += dx     * dy     * noise[y1][x1];
   // value += (1 - dx) * dy     * noise[y1][x2];
   // value += dx     * (1 - dy) * noise[y2][x1];
   // value += (1 - dx) * (1 - dy) * noise[y2][x2];

   return value;
}

float turbulence( std::vector<std::vector<std::vector<float>>>& noise, int dim, float x, float y, float z, float size) {
  float value = 0.0, initialSize = size;

  while(size >= 1)
  {
    value += smoothNoise(noise, dim, x / size, y / size, z / size) * size;
    size /= 2.0;
  }

  return (128.0 * value / initialSize);
}

/**********************************************************************************
 *
 *                                 Cloud Generators
 *
 ***********************************************************************************/

/*
 * Loads the texture in the GPU texture unit
 */
void Clouds::loadPackedNoiseTexture() {
  glActiveTexture( GL_TEXTURE0 + packed_noise_unit );
  glBindTexture( GL_TEXTURE_3D, packed_noise_id );

  // set the texture wrapping/filtering options (on the currently bound texture object)
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // Also GL_REPEAT
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // Also GL_REPEAT
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_MIRRORED_REPEAT); // Also GL_REPEAT
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // load and generate the texture
  glTexImage3D(
      GL_TEXTURE_3D,
      0,         // mipmap level ?
      GL_RGBA,    // internal format
      texture_pixels, // width
      texture_pixels, // height
      texture_pixels, // depth
      0,         // border
      GL_RGBA,    // format
      GL_UNSIGNED_BYTE,  // type
      noise_packed->data() );
}

/**
 * Generate Worley Noise
 * Creates an nxn texture with c cells
 * Each cell contains a single random points
 *
 * For each density pt, find its closest worley point
 */
void Clouds::generateWorleyNoise3DTexture( int cs, int pxs ) {
  std::vector<std::vector<std::vector<Vector3f>>> wps( cs, std::vector<std::vector<Vector3f>>( cs, std::vector<Vector3f>( cs, Vector3f() ) ) );
  std::vector<std::vector<std::vector<float>>> dps( pxs, std::vector<std::vector<float>>( pxs, std::vector<float>( pxs, 0 ) ) );

  // Generate random Vector2f
  for ( int z = 0 ; z < cs ; z++ ) {
    for ( int y = 0 ; y < cs ; y++ ) {
      for ( int x = 0 ; x < cs ; x++ ) {
        wps[z][y][x] = ( Vector3f::Random() + Vector3f( 1.f, 1.f, 1.f ) ) / 2.f;
      }
    }
  }

  std::cout << "  generated noise\n";
  // check every density pt
  float max_dist = std::numeric_limits<float>::min();
  #pragma omp parallel
  {
    #pragma omp for
    for ( int z = 0 ; z < pxs ; z++ ) {
      for ( int y = 0 ; y < pxs ; y++ ) {
        for ( int x = 0 ; x < pxs ; x++ ) {
          float cellsize = 1. / (float) cs;

          // need to normalize the sizes
          float normx = (x + 0.5) / (float) pxs; // [0, 1], center of px
          float normy = (y + 0.5) / (float) pxs; // [0, 1], center of px
          float normz = (z + 0.5) / (float) pxs; // [0, 1], center of px

          // Current cell
          int cellx = normx * cs;
          int celly = normy * cs;
          int cellz = normz * cs;

          // Position in cells
          float posx = normx * cs;
          float posy = normy * cs;
          float posz = normz * cs;

          // Check every cell
          float min_dist = std::numeric_limits<float>::max();
          for ( int zz = 0 ; zz < cs ; zz++ ) {
            for ( int yy = 0 ; yy < cs ; yy++ ) {
              for ( int xx = 0 ; xx < cs ; xx++ ) {

                // Check this cell in every block position
                for ( int offz = -1 ; offz < 2 ; offz++ ) {
                  for ( int offy = -1 ; offy < 2 ; offy++ ) {
                    for ( int offx = -1 ; offx < 2 ; offx++ ) {
                      float blockoffx = xx + offx * cs;
                      float blockoffy = yy + offy * cs;
                      float blockoffz = zz + offz * cs;

                      Vector3f const& wp = wps[zz][yy][xx];

                      float wp_posx = blockoffx + wp.x();
                      float wp_posy = blockoffy + wp.y();
                      float wp_posz = blockoffz + wp.z();

                      Vector3f pt = Vector3f( wp_posx, wp_posy, wp_posz );
                      float d = (Vector3f( posx, posy, posz ) - pt).norm();

                      min_dist = d < min_dist ? d : min_dist;
                    }
                  }
                }

              }
            }
          }

          dps[z][y][x] = min_dist;

          #pragma omp critical
          max_dist = min_dist > max_dist ? min_dist : max_dist;
        }
      }
    }

    #pragma omp for
    for ( int z = 0 ; z < pxs ; z++ ) {
      for ( int y = 0 ; y < pxs ; y++ ) {
        for ( int x = 0 ; x < pxs ; x++ ) {
          float d = ( 1 - dps[z][y][x] / max_dist ) * 255;
          // packed_noise[4 * pxs * y + 4 * x + 0] = d;
          (*noise_packed)[4 * pxs * pxs * z + 4 * pxs * y + 4 * x + 1] = d;
          // packed_noise[4 * pxs * y + 4 * x + 2] = d;
          // packed_noise[4 * pxs * y + 4 * x + 3] = 255;
        }
      }
    }
  }
}

/**
 * Generate Cloudy noise
 * Results in a nxn loaded at the perlin_noise_unit
 */
void Clouds::generatePerlinNoise3DTexture( int n ) {
  auto noise = generateNoise( n );

  #pragma omp parallel
  {
    #pragma omp for
    for ( int z = 0 ; z < n ; z++ ) {
      for ( int y = 0 ; y < n ; y++ ) {
        for ( int x = 0 ; x < n ; x++ ) {
          float t = turbulence( noise, n, x, y, z, 64 );
          // std::cout << "t: " << t << "\n";
          (*noise_packed)[4 * n * n * z + 4 * n * y + 4 * x + 0] = t;
          // packed_noise[4 * n * y + 4 * x + 1] = t;
          // packed_noise[4 * n * y + 4 * x + 2] = t;
          // packed_noise[4 * n * y + 4 * x + 3] = 255;
        }
      }
    }
  }
}

/**
 * Generates a set of bounding points (corners) for a 3D box
 * and places them into Clouds::positions.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateBoundingPoints(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( positions != nullptr ) { delete positions; }
  positions =  new MatrixXf( 3, matrixDimension );

  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        Vector3f corner = Vector3f( x, y, z ) * cell_size;
        unsigned long i = z + y * numberOfCells + x * numberOfCells * numberOfCells;
        positions->col( i ) = corner;
      }
    }
  }
}

/**
 * Generates a set of random Worley points for each box in 3D space
 * and places them into Clouds::worley_pts.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateWorleyPoints(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( worley_pts != nullptr ) { delete worley_pts; }
  worley_pts = new MatrixXf( 3, matrixDimension );

  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        Vector3f corner = Vector3f( x, y, z ) * cell_size;

        Vector3f R = Vector3f::Random(3).array().abs();

        unsigned long i = z + y * numberOfCells + x * numberOfCells * numberOfCells;
        Vector3f pt = corner + R * cell_size;
        worley_pts->col( i ) = pt;
      }
    }
  }
}

/**
 * Generates a set of density points and assigns them with a value:
 * For each pixel [density point] -> calculate the distance between
 *                                   that pixel and the closest Worley
 *                                   point [density value]
 * Results are placed into Clouds::density_pts & Clouds::density_vals.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateDensityValues(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( density_pts != nullptr ) { delete density_pts; }
  density_pts = new MatrixXf( 3, matrixDimension * matrixDimension );

  if ( density_vals != nullptr ) { delete density_vals; }
  density_vals = new MatrixXf( 3, matrixDimension * matrixDimension );
  float density_cell_size = cell_size / numberOfCells;

  if ( lines != nullptr ) { delete lines; }
  lines = new MatrixXf( 3, 2 * matrixDimension * matrixDimension );

  // for each cell (a cell contains 1 worley point)
  float max_dist = std::numeric_limits<float>::min();
  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        int i = z + y * numberOfCells + x * numberOfCells * numberOfCells; // cell index
        Vector3f corner = Vector3f( x, y, z ) * cell_size;

        // for each density block inside the cell
        for ( int xx = 0 ; xx < numberOfCells ; xx++ ) {
          for ( int yy = 0 ; yy < numberOfCells ; yy++ ) {
            for ( int zz = 0 ; zz < numberOfCells ; zz++ ) {
              int j = zz + yy * numberOfCells + xx * numberOfCells * numberOfCells; // density cell index
              Vector3f density_corner = corner + Vector3f( xx, yy, zz ) * density_cell_size;
              Vector3f density_center = density_corner + Vector3f( 1., 1., 1. ) * density_cell_size / 2;

              // search 27 cells for closest Worley point
              const Vector3f start = Vector3f( x - 1, y - 1, z - 1 );
              float min_dist = std::numeric_limits<float>::max();
              Vector3f min_pt;

              for ( int sx = start.x() ; sx < start.x() + 3 ; sx++  ) {
                int sxx = (sx < 0 ? numberOfCells + sx : sx) % numberOfCells;
                for ( int sy = start.y() ; sy < start.y() + 3 ; sy++ ) {
                  int syy = (sy < 0 ? numberOfCells + sy : sy) % numberOfCells;
                  for ( int sz = start.z() ; sz < start.z() + 3 ; sz++ ) {
                    int szz = (sz < 0 ? numberOfCells + sz : sz) % numberOfCells;

                    if ( sx != sxx || sy != syy || sz != szz ) {
                      // wrapping

                      // index into cells w/ wrapping
                      int k = szz + syy * numberOfCells + sxx * numberOfCells * numberOfCells;
                      Vector3f v = worley_pts->col( k );
                      float dist = (density_center - v).norm();

                      for ( int tx = 0, offx = -1 ; tx < numberOfCells ; tx++, offx++ ) {
                        for ( int ty = 0 , offy = -1 ; ty < numberOfCells ; ty++, offy++ ) {
                          for ( int tz = 0 , offz = -1 ; tz < numberOfCells ; tz++, offz++ ) {
                            Vector3f V = v + Vector3f( offx, offy, offz );
                            float dist = (density_center - V).norm();

                            if ( dist < min_dist ) {
                              min_pt = V;
                              min_dist = dist;
                            }
                          }
                        }
                      }

                    } else {
                      // not wrapping

                      int k = sz + sy * numberOfCells + sx * numberOfCells * numberOfCells; // index into cells w/ wrapping
                      Vector3f v = worley_pts->col( k );

                      float dist = (density_center - v).norm();

                      if ( dist < min_dist ) {
                        min_pt = v;
                        min_dist = dist;
                      }
                    }

                  }
                }
              }

              float cell_size = 1. / num_cells;

              // Add perlin noise on xz-plane
              // float perlin = perlinNoise( density_center.x() * cell_size, density_center.z() * cell_size );

              int I = i * pow( numberOfCells, 3 ) + j;

              lines->col( 2 * I ) = density_center;
              lines->col( 2 * I + 1 ) = min_pt;

              density_pts->col( I ) = density_center;
              // density_vals->col( I ) = Vector3f( perlin, min_dist, 0 );

              // Find maximum value for nomalizing
              if ( min_dist > max_dist ) {
                max_dist = min_dist;
              }
            }
          }
        } // end for each density block inside the cell

      }
    }
  } // end for each cell

  // Normalize Worley Noise
  for ( int i = 0 ; i < density_vals->cols() ; i++ ) {
    density_vals->col(i).y() /= max_dist;
  }
}

/**
 * Main generate functon for Bounding, worley, and density points
 */
void Clouds::generatePoints() {
  generateBoundingPoints(num_cells + 1);
  generateWorleyPoints(num_cells);
  generateDensityValues(num_cells);
}

void Clouds::generateBoundingBox() {
  // 12 lines to draw cube

  Vector3f& a = bbox_min;
  Vector3f& b = bbox_max;

  // back face
  bbox_pts.col( 0  ) = bbox_min;
  bbox_pts.col( 1  ) = Vector3f( b.x() , a.y() , a.z() );

  bbox_pts.col( 2  ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_pts.col( 3  ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_pts.col( 4  ) = Vector3f( b.x() , b.y() , a.z() );
  bbox_pts.col( 5  ) = Vector3f( a.x() , b.y() , a.z() );

  bbox_pts.col( 6  ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_pts.col( 7  ) = bbox_min;

  // front face
  bbox_pts.col( 8  ) = bbox_max;
  bbox_pts.col( 9  ) = Vector3f( a.x() , b.y() , b.z() );

  bbox_pts.col( 10 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_pts.col( 11 ) = Vector3f( a.x() , a.y() , b.z() );

  bbox_pts.col( 12 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_pts.col( 13 ) = Vector3f( b.x() , a.y() , b.z() );

  bbox_pts.col( 14 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_pts.col( 15 ) = bbox_max;

  // Spokes between squares
  bbox_pts.col( 16 ) = Vector3f( a.x() , a.y() , a.z() );
  bbox_pts.col( 17 ) = Vector3f( a.x() , a.y() , b.z() );

  bbox_pts.col( 18 ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_pts.col( 19 ) = Vector3f( a.x() , b.y() , b.z() );

  bbox_pts.col( 20 ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_pts.col( 21 ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_pts.col( 22 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_pts.col( 23 ) = Vector3f( b.x() , a.y() , a.z() );

  // back face
  bbox_tris.col( 0  ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_tris.col( 1  ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_tris.col( 2  ) = Vector3f( a.x() , a.y() , a.z() );

  bbox_tris.col( 3  ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_tris.col( 4  ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_tris.col( 5  ) = Vector3f( b.x() , b.y() , a.z() );

  // front face
  bbox_tris.col( 6  ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 7  ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 8  ) = Vector3f( a.x() , b.y() , b.z() );

  bbox_tris.col( 9  ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_tris.col( 10 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 11 ) = Vector3f( a.x() , b.y() , b.z() );

  // left side
  bbox_tris.col( 12 ) = Vector3f( a.x() , a.y() , a.z() );
  bbox_tris.col( 13 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 14 ) = Vector3f( a.x() , b.y() , a.z() );

  bbox_tris.col( 15 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_tris.col( 16 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 17 ) = Vector3f( a.x() , b.y() , a.z() );

  // right side
  bbox_tris.col( 18 ) = Vector3f( b.x() , a.y() , a.z() );
  bbox_tris.col( 19 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 20 ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_tris.col( 21 ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_tris.col( 22 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 23 ) = Vector3f( b.x() , b.y() , a.z() );

  // top
  bbox_tris.col( 24 ) = Vector3f( a.x() , b.y() , a.z() );
  bbox_tris.col( 25 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_tris.col( 26 ) = Vector3f( b.x() , b.y() , a.z() );

  bbox_tris.col( 27 ) = Vector3f( b.x() , b.y() , b.z() );
  bbox_tris.col( 28 ) = Vector3f( a.x() , b.y() , b.z() );
  bbox_tris.col( 29 ) = Vector3f( b.x() , b.y() , a.z() );

  // top
  bbox_tris.col( 30 ) = Vector3f( a.x() , a.y() , a.z() );
  bbox_tris.col( 31 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 32 ) = Vector3f( b.x() , a.y() , a.z() );

  bbox_tris.col( 33 ) = Vector3f( b.x() , a.y() , b.z() );
  bbox_tris.col( 34 ) = Vector3f( a.x() , a.y() , b.z() );
  bbox_tris.col( 35 ) = Vector3f( b.x() , a.y() , a.z() );
}

/**
 * Fill the texture with 1 - d
 *   where d is the distance to the nearest Worley point
 */
void Clouds::generateDensityTexture() {
  glActiveTexture( GL_TEXTURE0 + density_tex_unit );
  glBindTexture( GL_TEXTURE_3D, density_tex_id );

  // set the texture wrapping/filtering options (on the currently bound texture object)
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // load and generate the texture
  glTexImage3D(
      GL_TEXTURE_3D,
      0,         // mipmap level ?
      GL_RGB,    // internal format
      num_cells, // width
      num_cells, // height
      num_cells, // depth
      0,         // border
      GL_RGB,    // format
      GL_UNSIGNED_BYTE,  // type
      density_vals->data() );
}

