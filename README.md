# Como Image Processing Package

This package contains image processing files for [Project Como](https://github.com/TSummersLab/como) at the University of Texas at Dallas. While the files are mostly generic image processing files, the parameters and operation are tuned for the COMO package.

## Capabilities
The package does the following things in order:
1. Get an image from a webcam/camera and publish the 8-bit color or 8-bit grayscale version of the image to the topic `/cam/raw`
2. Detect a checkerboard in the image and extract its outer most corners and store them in `/config/corners.txt`
3. Extend the outer corners of the checkerboard to the image's horizontal limit
4. Reconstruct the image within the extended corners in a top-down view. This image is published to the topic `/cam/line_img`. It is expected to have a single white line on a black background representing the line that the car must follow.
5. Split the raw image at the top edge of the extended checkerboard corner. The part of the image above the line is published to the topic `/cam/forward_img`. This image is expected to have the camera's forward facing data excluding the part of the line that the car must immediately follow.
6. Process the images:
    1. `/cam/line_image`:
        - The image is processed to find the orientation of the line relative to the body frame of the car. It also finds the position of the center of the detected line in the car's body frame.

## Assumptions and Requirements
For the package to operate, the following assumptions are made:
- The package is used on a Linux platform (Ubuntu, Linux Mate, ...)
- A checkerboard is available.
- Video for Linux is installed. If not installed, run the following command in terminal:
    ```
    sudo apt-get install v4l-utils
    ```
- The camera is mounted on the car such that it is pointing forward (its optical axis makes some angle (e.g. 90 degree) with the gravity vector)
- When using a checkerboard, the checkerboard is centered horizontally in the camera's field of view. The top and bottom edges of the checkerboard must be as parallel as possible with the image top and bottom edges.
- When recalibrating (detecting the corners of the checkerboard), the checkerboard is always placed at the same distance from the image. If that is not the case, refer to the [Usage](#usage) section for the required modifications.

## Outline
The package is organized as follows
- config
  - Contains configuration files.
  - Users should make modifications to those files.
- launch
  - Contains launch files to run files in src.
  - Users need not make any modifications to these files.
- msg
  - Contains the messages specific to this package.
  - Users need not make any modifications to these files.
- src
  - Contains the source files for the package.
  - Users need not make any modifications to these files.

## Usage
To use the package, follow these instructions:

1. Clone the package to the desired workspace (e.g. `~/workspace/src` where `catkin_make` or `catkin build` is invoked in `workspace`). We will refer to that workspace as `<path>`.
    ```
    cd <path>/src
    git clone https://github.com/The-SS/como_image_processing.git
    ```

2. Build the package and source it.
    ```
    cd <path>
    catkin build # or catkin_make
    source devel/setup.bash
    ```

3. Modify `<path>/como_image_processing/config/corners.txt`.
    - The text file contains 5 lines:
       - Line 1 is a check to determine whether to use the current data or not. When first using the package, or if the camera is moved, replace the first and/or the second entry in this line by 0. (i.e. 0. instead of 1.000000000000000000e+00)
       - Lines 2, 3, 4, and 5 contain the coordinates of the outer corners of the checkerboard. These are automatically generated.
    - If the number of lines is not 5, calibration is due, and the data will be overwritten by the newly obtained coordinated of the four outer corners of the checkerboard.

4. Modify  `<path>/como_image_processing/config/cam_bridge_params.yml`.
    - This file contains data parameters for the camera.
    - Make the following modifications:
       - `imgMode`:
         - If set to 'gray', the image published to `/cam/raw` will be a mono8 encoded image.
         - If set to 'color', the image published to `/cam/raw` will be a bgr8 encoded image.
         - Any other value will prevent the data from being published to `/cam/raw`
      - `camPath`:
         - Path to the webcam in /dev.
         - To find the value of path, disconnect your camera, then run the following command in terminal:
          ```
          ls /dev/video*
          ```
         - Connect the camera and run the above command again. The newly added device corresponds to your camera.
         - Replace the value for `camPath` (i.e. "/dev/video6") by the value obtained above using the same format. Essentially, only the number (i.e. 6) should be changed.
       - `resolution`:
          - contains `width` and `height` which are the width and height of the resolution to be used for the image published to `/cam/raw`.
          - To view supported resolutions, use either of the following ways:
            1. using v4l:
                - Run the following command in terminal:
                  ```
                  v4l2-ctl --list-formats-ext -d /dev/video6
                   ```
                  But replace `/dev/video6` by the value found for `camPath`
            2. using lsusb:
                - Remove the camera.
                - Run `lsusb` in a terminal window
                - Plug the camera back and run `lsusb` again.
                - Identify the new device and note the `Bus` and `Device` values (e.g. 001 and 010 respectively)
                - Run the following command in terminal:
                ```
               lsusb -s 001:010 -v | egrep "Width|Height"
               ```
               But first, replace the values `001` and `010` by the values obtained for `Bus` and `Device` respectively.
                - Choose the values for height and width from that list.

5. Modify  `<path>/como_image_processing/config/img_preprocessing_params.yml`.
    - The file contains data about the checkerboard size which are used in preprocessing the image (splitting image).
    - Count the number of internal corners of the checkerboard, both vertically and horizontally. For example, if the checkerboard has 6 horizontal rows of boxes and 8 vertical rows of boxes, the number of internal corners would be 5 horizontally and 7 vertically.
    - Fill in that data in `CheckerboardParams` as explained in the file.

6. Modify `<path>/como_image_processing/config/process_line_params.yml`.
   - The file contains data used when processing the line image.
   - Make the following modifications:
      - `LineSub`:
        - Contains the name of the topic to which the `line_img` is published. Do not modify this unless you want to get the data from a different source
      - `LineParams`:
        - Contains data for processing the line image.
        - `threshold` is the threshold used to binarize the image. There is no need to change it.
        - `width_of_line` is the expected width of the line in pixels. This is just an estimate for the minimum width that the line might have. For a 640x480 image of a 1.5cm line with the camera at 10cm high, 50 pixels is a reasonable value. You may increase it as the resolution increases, or decrease it as the resolution decreases.
        - `width_gain` is a multiplier for the `width_of_line` representing the percentage of the width to be used when estimating the expected area and parameter of the line.
      - `PlottingParams`:
        - `arrow_scale` is a multiplier for the length of the arrow added to the image representing the orientation of the line. Vary it proportionally to the variation in resolution.
      - `CheckerboardParams`:
        - `square_size` is the size of one side of a square of the checkerboard in meters. Measure the value then replace it.
        - `car_body_frame_to_board` is the distance between the chosen origin for the body frame of the car and the horizontal line passing through the two outer corners of the checkerboard that are closest to the car. When placing the checkerboard, measure this value then replace the current value. If the checkerboard is moved in the next calibration, this value should be Modified.

7. Run a launch file:
    - The above modifications need to be done once (unless changes are made to the checkerboard, calibration, line, or similar parameter that are generally unchanged). Once the modifications are made and the package is setup, users can run a launch file to start the package. This is done by running the following in a terminal window:
    ```
    roslaunch como_image_processing <package>
    ```
    - In the above command, replace`<package>` by one of the following options:
      - `cam_bridge.launch`:
        - Starts `cam_bridge.py` which fetches an image and publishes it to `/cam/raw`
      - `img_preprocessing.launch`:
        - Starts `cam_bridge.py` and `img_preprocessing.py`. The latter splits the image into an image of the line and a forward facing image. The resulting images are published to `/cam/line_img` and `/cam/forward_img` respectively.
      - `process_line.launch`:
        - Starts `cam_bridge.py`, `img_preprocessing.py`, and `process_line.py`. In addition to doing what `img_preprocessing.launch` does, it also analyses the line to produce data about how the line is oriented and where it is positioned relative to the body frame of the car. It publishes the line data to `/line_data`. It also publishes a version of `/cam/line_img` containing an arrow indicating the direction of the line relative to the camera frame as defined in `process_line.py`. That image is published to `/cam/orientation_img`.
