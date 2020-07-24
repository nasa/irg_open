#!/usr/bin/python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# Compute the albedo of a sphere mapped with a specified PNG. Texels are
# weighted based on their area when mapped onto the sphere, more weight at
# the equator and less weight at the poles.

# Assumptions:
# 1. Each planet is a sphere, but this computation would be more accurate if
# each were a specific ellipsoid.
# 2. Color channels in an RGB image are weighted equally when computing an
# albedo. Separate albedo values that span a range of the spectrum can be
# found, but I can't find information about how these values are combined to
# produce a single albedo value.

import sys
import os
import math
from PIL import Image


def usage():
  print('usage: ' + os.path.basename(sys.argv[0]) + ' <input.png>')

if len(sys.argv) != 2:
  usage()

img = Image.open(sys.argv[1])
num_bands = len(img.getbands())

if num_bands != 1 and num_bands != 3:
  print('Not a 1-channel or 3-channel image. Exiting.')
  sys.exit()

texels = img.load()

divisor = 0
albedo = 0

for j in range(img.height):
  row_albedo = 0

  for i in range(img.width):
    texel_albedo = 0
    texel = texels[i,j]
    if num_bands == 1:
      texel_albedo = texel / 255.0
    else:
      # Average RGB values to get a single albedo value
      texel_albedo = ((texel[0] + texel[1] + texel[2]) / 3.0) / 255.0

    row_albedo += texel_albedo

  row_albedo /= img.width
  circumference = math.sin(math.pi * (j + 0.5) / img.height)
  albedo += row_albedo * circumference
  divisor += circumference

albedo /= divisor
print("albedo =", albedo)
