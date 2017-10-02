# Path Planning
 
For this project, the first goal is to get the car driving forward in its lane.

Then after this is happening succesfully, we move onto a way of detecting vehicles ahead of it and slowing down and changing lanes where appropriate to continue along the road safely and quickly.

To get the car moving along in its lane smoothly, I calculate a few sparse waypoints ahead of it along its lane using the map Frenet 's' and 'd' co-ordinates, and the supplied getXY function to convert these back into world co-ordinates. I then use the spline.h library to create a spline along these sparse waypoints, and interpolate points along this spline to add to the path so that the car travels at the needed speed.

So first, I generate two waypoints: The first is where the car was previously, and the second is where the car is right now. The way I calculate the previous point is simply looking at the car's yaw and moving one unit back from the car's position that way. This helps with the smoothness of the path, by starting the path off with the car's current position and heading. You can see this in lines 260 onwards in main.cpp. And if the simulator gives previous waypoints back from the update, then I start by using two points from this instead.

Then I calculate three more sparse waypoints ahead of the car, from the map waypoints, using Frenet 's' and 'd' co-ordinates. I calculate s+60, s+80, and s+90, meaning the car's current position along the road plus 60 meters, etc. I use the car's current lane (either 0, 1, or 2), to calculate the 'd' element of the Frenet co-ordinate. These are also converted to world X, Y co-ordinates.

I then add these five waypoints to our spline, as shown on line 325 in main.cpp. But before doing so, I translate and rotate them so that the point where the car is is x=0, and the car's current heading means that increasing x results in y=0 to go straight ahead, i.e. rotate so that the angle 0 means head straight ahead. This helps with making the splines have a unique x point for each y value.

To make the car's travels smooth, I load up the new path with what was leftover from the calculated path previously.

