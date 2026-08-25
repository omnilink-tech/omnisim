#!/bin/sh
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


#
# Description: Script for superimposing a label ("OmniSim TM") at the bottom left of a PNG image
# Required: ImageMagick (http://www.imagemagick.org) AND ttf-freefont
# Usage: ./superimpose_omnisim_logo.sh yourImage.png
#

if [ $# -ne 1 ]; then
  echo 1>&2 Usage: $0 image_filename
  exit 127
fi

width=`identify -format %w $1`
height=`identify -format %h $1`
env LC_CTYPE=en_US.utf8 printf "OmniSim\u2122" | \
  convert -font /usr/share/fonts/truetype/freefont/FreeSansBold.ttf -background none -gravity South -fill '#00000060' \
  -strokewidth 2 -stroke '#ffffff60' -size ${width}x${height} -resize 25x25% \
  label:@- +size $1 +swap -gravity SouthWest -geometry +5+5 -composite $1
