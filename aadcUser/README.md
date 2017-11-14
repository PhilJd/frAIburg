# Car - The ADTF code

This folder contains the code that runs on the car.

## SSH
- lan: with static ip
- wifi: findout the car wlan0 ip (inet addr)
`$ ifconfig -a`

connect with:
`$ ssh -2- X -v aadc@<ip>`
or for a faster connection
`$ ssh -XC -c aes256-ctr aadc@<ip>`

start adtf
`$ /opt/adtf/2.14.0/bin/adtf_devenv`
or
`$ /opt/adtf/2.14.0/bin/adtf_devenv --logfile=adtflog.txt`


## Camera Calibration with Kalibr
- Download Kalibr-cde
- Record images with ROS and create a .bag file
- Run Kalibr ./kalibr_calibrate_cameras with options:
	--bag ./bagfile.bag 								// path to bagfile
	--topics /pylon_camera_node/image_raw_crop_mono 	// you get this from ROS
	--models pinhole-equi								// gave the best results
	--target ./april_6x6_80x80cm.yaml 					// the calibration pattern that you used
Possible Error during calibration:
[FATAL] []: No corners could be extracted for camera < param from --topics > Check the calibration target configuration and dataset.
--> can be solved by converting the ros.bag to mono. Reason: basler gives rgb matrix directly

The calibration images were recorded with the cameras from car 1. The calibration params are used for both cars.

Kalibr outputs yaml with intrinsics: fx, fy, cx, cy
Note: the calibration for the rearcam was made for a horizontally and vertically flipped image (which corresponds to the real scene)
You find the calibration files in ADTF/src/configuration/kalibr_* (git)

## IPM
- using undistorted images (calibration from kalibr)
- using an alpha / zoom (scale_to_crop value) of 0.45 for the rearcam and 0.6 for the basler
- you find the image and world points in the frAIburg_configuration.xml file