#!/usr/bin/env bash
# Make the city_traffic supervisor reachable from this project so the cars move
# in city_husky_nav.wbt. Run from repo root after gen_city_nav.py.
cp -r projects/samples/demos/controllers/city_traffic projects/omni_quest/controllers/
