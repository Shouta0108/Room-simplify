# Room-simplify — with OpenCV

Create a simple overview map of a room using the **LD19 LiDAR** and **OpenCV**.
This project runs on a PC in **C++**, so you do **not** need a Raspberry Pi.

## Overview

This program reads scan data from an LD19 LiDAR connected through a **USB-to-serial converter**, then visualizes and processes the room shape.

## Requirements

Before running the project, install the following:

- **gnuplot 5.4 patchlevel 8**
- **OpenCV 4.11.0**

You also need:

- An **LD19 LiDAR**
- A **USB serial converter module** to connect the LiDAR to your PC

## Result example

<img width="720" height="720" alt="Hough_result" src="https://github.com/user-attachments/assets/107fc825-ca30-4c15-83b3-ab0348787107" />

## Notes

- This repository is written in **C++**.
- Make sure the serial device is recognized by your OS before starting the program.
- If you want, you can later expand this README with build steps, usage instructions, and sample output details.
