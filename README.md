# Depth Recovery from Stereo on Remotely Operated Vehicles

<html>
  <style>
    img {
      width: 40%;
      display: inline-block;
    }
  </style>
</html>

![Unprocessed left image - Katzaa Dataset 01, saved as jpg](images/KatzaaTEST_input.jpg)
![Disparity from left image - Katzaa Dataset 01](images/results_200/disparity_SGBM_Katzaa01.png)

Information from summer '23 research project on depth-recovery from stereo on underwater ROVs. Using ZED camera, and a variety of depth-recovery methods. 

*Top - from Katzaa Dataset . Bottom - image after run through stereo disparity code.*

## What's in this repository? 
#### `images/`
All images used in these files - containing some samples of dataset imagery and code run during the project.

#### `DataCode.md`
Links and descriptions of existing stereo projects that include datasets and code to be implemented - see project description on this page (`README.md`) for which ones and how they were implemented. 

#### `NEWSetup.md`
A description of all the steps to equip the software of the loaner computer for this project. 

#### `README.md`
The file you are looking at right now! A summary of the **resources** contained and a **project summary**. 

#### `Calibration.md`
Calibration resources that could be helpful in future. For ZED-specific calibration information, see `WorkingWithZED2.md`

#### `StereoMatching.md`


#### `Transmission.md`

#### `WorkingWithZED2.md`
Instructions on working with the ZED2 camera in relation to running the **ROS node**, published and subscribed topics, and **calibration** information. Links and references to relevant Stereolabs documentation on the ZED2.

---

## Project Summary

The setup went from the process of using the ZED cameras, which were bought to integrate with the lab's existing BlueROV setup, to be used to produce the stereo images. The ZED camera provides a variety of raw and augmented topics published via ROS (which integrates with the existing setup) and comes with an SDK to further adjust settings and interact with the camera. 
1. Distinguishing onboard ZED processes vs. what needs to be run on the Jetson Nano
2. Running the appropriate commands and utilizing RViz

More specifics can be found in the WorkingWithZED2.md document.

From there, I focused on what algorithms could be used to recover depth information. This began as a literature review and research into other existing underwater robotics setups which utilize stereo vision. 
1. The MARIS Project and work of the RIMLab
2. The SQUID Dataset and work of the Treibitz Lab

There were also distinct methods of utilizing stereo imagery in robotics applications, and further different ways of getting depth information from said methods. Image enhancement often went along hand-in-hand with depth estimation, because underwater video is subject to turbid water conditions, haze, and fogging that make it harder to extract features and perform other standard techniques used in terra firma robotics applications. 
1. Image enhancement - color, dehazing
2. Transmission maps for depth
3. Depth from Disparity
4. Depth from Motion (Parallax)
