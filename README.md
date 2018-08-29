# CarND--PID
Self-Driving Car Engineer Nanodegree Program
James Di Donato
August 2018

---
## Introduction
In this project, I implemented PID controller in C++ to maneuver the vehicle around the simulated lakefront race track. 

The simulator provides the cross track error (CTE) and the velocity (mph) and the script computes the steering angle to control the vehicle using a PID algorithm. A video of vehicle driving around the track with the tuned paramters can be watched in "PID_Final.mp4". 

## Rubric Points


**Describe the effect each of the P, I, D components had in your implementation. Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.**

Starting with the P, for proportional, it represents how agressively the vehicle corrects the steering to the centerline. In general, large P value would lead to lots of oscillations throwing the vehicle off the course while a small value would lead to understeering around corners. This parameter is the simplist to understand, as given in the name, the steering value is directly proportional to the Cross Track Error (CTE), the error between the vehicle position and the tracking line. The second parameter, I for integral, works by accumulating its influence over the steering angle based on the previous errors. As the sum of previous errors, the steering angle is actuated in proportion until the errors become negative and reduce the sum. This parameter is best at counteracting rapid fluctuation around the tracking line and has the oppositive effect of the P term. In this assignment, the I term significantly reduced the 'jitter' of the vehicle on the center line and acted as a low pass filter on the steering rack. Finally, the derivative term operates on the change in CTE from the previous time step and is best at minimizing overshoot caused by the proportional component. The derivative term does not consider any previous errors beyond the last time step. In this project, the derivative term was effective at keeping the vehicle from oscillating out of control while helping the vehicle steer around the sharp corners with speed.

In the implementation, I was suprised at how a small integral parameter of 0.001 was so effective at keeping the vehicle centered.

**Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!**

I started by manually tuning the parameters from a starting point of (P,I,D) = (0.5, 0.0, 0.5). I observed a large oscillation while driving on the straight track the car eventually fell off. To counteract this, I  I increased the derivative parameter and reduced the P term. I was able to navigate around the track succesfully after manually tuning the parameters to (P,I,D) = (0.2, 0.0, 5). At this point there was still room for improvemement, while the vehicled stayed on the tes track it oscillated agressively around slight bends and would not be a comfortable drive.  At this point I starting playing with the integral parameter. I started at 0.5 and slowly worked it down to 0.001 to smooth out the ride.

Tuning History:
- P,I,D = 0.5, 0.0, 0.5 - lots of oscillations, doesnt make it around the 1st corner
- P,I,D = 0.2, 0.0, 1 - still lots of oscillations in the straight aways
- P,I,D = 0.2, 0.0, 2 - still lots of oscillations - much better on the straight
- P,I,D = 0.2, 0.0, 5 - small oscillations in slight curve's - better on straight - Passes
- P,I,D = 0.2, 0.5, 5 - Integral takes over and steers off track immediately
- P,I,D = 0.2, 0.1, 5 - Integral helps PID maintain centerline but reacts violently on turns
- P,I,D = 0.2, 0.001, 5 - Integral helps PID maintain centerline on both straightaway and turn

## Next Steps

At this point I was satisfied with the result and decided to submit the project after manual tuning. I realize that there is plenty of room to improve the algorithm but I was happy with what I had learned and the project met the requirements. Some next steps would be to use a numerical optimizer such as Twiddle to optimize the parameters for the physics of this track. A more robust solution would be to run this optimizer on a larger course with varying turning radii to ensure optimal peformance in the real world. Additionally, one would need to ensure that the PID calibration keeps the vehicle within the bounds of a standard lane width and not an entire street. A final step would be to use a PID algorithm to maximize the speed of the vehicle whislt mainitaning an acceptable level passenger comfort ( desired CTE ).


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

