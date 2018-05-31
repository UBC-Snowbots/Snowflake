#!/bin/bash

rosnode kill height_filter hue_filter saturation_filter value_filter nodelet_manager rgb_to_hsv
rosnode cleanup
