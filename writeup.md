## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**

**Training / Calibration**

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

Added lower and upper thresholds to the original threshold function, and created 3 new functions for filtering navigable terrain, obstacles and rocks.

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

Used the same functions for processing camera images.

The terrain and obstacle bitmaps were not complementary, due to all three channels having to be over/below the thresholds. This would turn out to be problematic later on, but I didn't address it in the notebook.

![Screenshot of a processed image][image2]

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

Copied the functions for filtering terrain, obstacles and rocks to the perception file. Left the decision file untouched.

Started in autonomous mode and it looked like it would pass already:

![Image of first run results][image1]

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

##### 2.1 Following walls

###### 2.1.1 Wall following strategy
Since rocks are placed near walls, it looked like the rover was supposed to follow the walls. So I came up with the following strategy (actually, I found it on the Internet, but it looked simple enough to actually work, so I decided to give it a shot):
|is wall on the right|is wall ahead|action|
|-|-|-|
|||turn right, look for wall|
|√||advance|
||√|facing wall, turn left to align the wall on the right|
|√|√|change rover mode to *stop*|

However, using this strategy, the rover stops every time it faces both front and right walls, and it made testing so painful to watch (BTW, for future offerings: a headless mode for the Unity app would be amazing!).

I added another flag for when the rover has a wall in its immediate front as a condition for changing mode to *stop*, and made it turn left in case there's a wall ahead.

|is wall on the right|is wall ahead|is wall in front|action|
|-|-|-|-|
||||turn right, look for wall|
|√|||advance|
|√|√||facing wall but not in immediate front, turn left to align the wall on the right|
|√|√|√|wall too close to advance turning left, change mode to *stop*|



###### 2.1.2 Wall detection

Added a filter function for counting terrain points inside arbitrary angle and distance ranges. This makes it very easy to select, for instance, terrain points located in the right half of the image, at a distance no longer than 18 units.

```py
def mask(dists, dist_min, dist_max, angles, angle_min, angle_max):
    normalized_angles = angles * 180/np.pi
    return (normalized_angles > angle_min) & \
            (normalized_angles < angle_max) & \
            (dists > dist_min) & \
            (dists < dist_max)

def no_points_mask(dists, dist_min, dist_max, angles, angle_min, angle_max):
    masked = mask(dists, dist_min, dist_max, angles, angle_min, angle_max)
    return len(dists[masked])
```

##### 2.2 Improving navigation

Since not all ground is closed properly, the rover still got stuck sometimes by falling into the ground, mostly near black rocks. To address this, I added two parts to the strategy: one *reactive*, that is getting it out after getting stuck and one *preventive*, that tries to make it avoid black rocks that show up in the images.

###### 2.2.1 Escape after getting stuck

For detection, the rover position is saved every 8 seconds. If at one step, the new position is very close to the previous one (less than .2 distance units), then most likely the rover is stuck and the mode is changed to *stuck*.

Once the rover gets into *stuck* mode, the strategy for getting out is:
* turn left
* turn right (spinning usually gets the rover out of the ground clips)
* back off a bit
* switch to *stop* mode and look for navigable ground to progress

###### 2.2.2 Avoid black rocks

However, even after adding the escape algorithm, the rover runs were pretty annoying to watch -- the rover eventually breaks free, but it takes a while to get out.

That is why I also made it avoid black rocks that it detects in the right side of the screen. This also made the runs a bit teetery (since it detects regular walls as being black rocks) so even though it gained some overall speed by not getting stuck, fidelity dropped significantly.

New state on the rover:
```py
# Stuck detection
self.last_pos = None
self.stuck_time = 0

# Black rocks detection
self.black_angles = None
self.black_dists = None

# Wall detection
self.wall_right = False
self.wall_ahead = False
self.wall_front = False
self.black_wall_right = False
```

##### 2.3 Improving fidelity

Some fidelity was lost due to the rover constantly turning left and right, so I added thresholds for pitch and roll angles when updating the world map: ```359.99 < angle < 0.01```.

![alt text][image3]
