#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# Create a normalmap for a geotiff.

import sys
import gdal
import numpy
import cv2

def main():
  if len(sys.argv) != 3:
    print('\nusage: %s <geotiff> <normalmap.png>' % sys.argv[0])
    sys.exit(1)

  # Open geotiff
  try:
    geotiff = gdal.Open(sys.argv[1])
  except RuntimeError as e:
    print('Could not open %s' % sys.argv[1])
    sys.exit(1)

  # Get raster georeference info
  gt = geotiff.GetGeoTransform()
  pixelWidth = abs(gt[1])
  pixelHeight = abs(gt[5])  # this is sometimes negative. Huh??
  num_cols = geotiff.RasterXSize
  num_rows = geotiff.RasterYSize

  try:
    band = geotiff.GetRasterBand(1)
  except RuntimeError as e:
    print('No band 1 found')
    sys.exit(1)

  heights = band.ReadAsArray().astype(numpy.float)
  normals = numpy.empty((num_rows, num_cols, 3),dtype=numpy.uint8)

  percent = 0
  for i in range(0, num_rows-1):
    # compute indices
    y0 = numpy.clip(i - 1, 0, num_rows-1)
    y1 = numpy.clip(i + 1, 0, num_rows-1)

    for j in range(0, num_cols-1):
      # compute indices
      x0 = numpy.clip(j - 1, 0, num_cols-1)
      x1 = numpy.clip(j + 1, 0, num_cols-1)

      # compute vectors
      right = [pixelWidth * (x1 - x0), 0.0, heights[i][x1] - heights[i][x0]]
      up = [0.0, pixelHeight * (y1 - y0), heights[y0][j] - heights[y1][j]]
      normal = numpy.cross(right, up)
      # normalize
      normal /= numpy.sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])

      # Put normal into byte array
      normals[i][j] = numpy.clip((normal + 1.0) * 128.0, 0, 255)

    newpercent = int((float(i) / float(num_rows-1)) * 100.0)
    if newpercent > percent:
      percent = newpercent
      sys.stdout.write('%s%% ' % percent)
      sys.stdout.flush()

  # Write png
  # cv2 writes BGR by default, so we must tell it to write RGB
  cv2.imwrite(sys.argv[2], cv2.cvtColor(normals, cv2.COLOR_BGR2RGB))


if __name__=='__main__':
  main()

