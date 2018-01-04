# Vision
- This package contains everything pertaining to the vision stack that we develop in-team (ie. no external code). 
- This includes nodes for filtering and basic camera launch nodes.
- This package may also contain launch files for launch both our nodes, and any external nodes related to vision

# Nodes
## ros_vision
- This is our IPM (Inverse Perspective Mapping - basically translating things from the camera's point of view to a birds-eye point of view) and HSV (Hue, Saturation, Value - used for filtering out white lines) node.
- It contains a fair amount of legacy code (*sigh*) so may be a *little* disorganized

### Params
- Image parameters
  - width (**int**): input image width
  - height (**int**): input image height
- IPM Bounding Box Parameters (`0 <= value <= 1`)
  - ipm_base_width (**double**): the length of the bottom of the box as % of image
  - ipm_top_width (**double**): the length of the top of the box as % of image 
  - ipm_base_displacement (**double**): displacement of the box from the bottom of the image 
  - ipm_top_displacement (**double**): displacement of the box from the top of the image
- update_frequency (**int**): fps update
- display_window_width (**int**): the width of the overview window
- display_window_height (**int**): the height of the overview window
- config_file (**string, absolute path**): the location of the config file
- show_image_window (**boolean**): displays the overview window (Can be toggled by ‘s’)
- show_calibration_window (**boolean**): displays the calibration window (Can be toggled with ‘m’)
