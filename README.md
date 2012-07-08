# DanceTracker

This code is used to track dancers in the [FlockLogic](http://www.princeton.edu/~flocklogic/) project.

# Tracking Algorithm and Parameters

## Segmentation

Each frame (image from the camera) is split into its H, S, and V channels (see http://en.wikipedia.org/wiki/HSL_and_HSV and http://www.yafla.com/yaflaColor/ColorRGBHSL.aspx).  

Roughly,

* H stands for 'hue' and corresponds to the 'color' as in red vs green vs blue
* S stands for 'saturation' and corresponds to how saturated (intense) the
  color is 
* V stands for 'value' and corresponds to how dark/bright the color is. 

The V channel is basically what you would want if you converted the color image to grayscale.  

Each channel is thresholded indepenedently: 

    H_thresholded = 'true' at each pixel where H_min < H < H_max and 'false' otherwise 
    S_thresholded = "" S_min < S < S_max "" 
    V_thresholded = "" V_min < V < V_max ""

The "HSV Thresh" frame is created by ANDing all three of the individual channel thresholds:

    HSV Thresh = H_thresholded AND S_Thresholded AND V_thresholded

The "BG Thresh" frame is created by looking for pixels that are darker than the background by the value of the "BG Thresh" parameter (poor choice of names on my part - DTS).  The brightness/darkness corresponds to the value of the V channel:

    BG Thresh(image) = 'true' at each pixel where the V is less than BG -
    BG_thresh(parameter)

The "Threshold" frame is the logical OR of the "HSV Thresh" frame and the "BG Thresh" frame:

    Threshold = 'true' at each pixel where either HSV Thresh or BG Thresh are true

In other words, we set to 'true' any pixel that is either the right color or sufficiently darker than the background.  

### HSV threshold tuning 

Suggested process:

1. Set the 'BG Thresh' parameter to 255.  This effectively turns off the background thresholding.  I don't think it works that well anyways.

2. Set the S and V thresholds to 0 (min) and 255 (max).  This lets almost all of the pixels through.

3. Find H values that light up the hats in the threshold image, but may also let through some other stuff. 

4. Adjust the S and V values to get rid of the stuff that isn't the hats.  

It may take a few iterations.  You can try playing with it in Matlab or something like Photoshop, but be careful - the values do not map directly.  The conversion in DanceTracker is done using OpenCV's [cvCvtColor](http://opencv.willowgarage.com/documentation/miscellaneous_image_transformations.html#cvtcolor) function.  The values range from 0 to 255 for S and V, and 0 to 180 for H.  Matlab's [rgb2hsv](http://www.mathworks.com/help/techdoc/ref/rgb2hsv.html), for example, has values ranging from 0 to 1 for H, S, and V.  The conversion should just be a scaling, i.e., if H = 0.5 in Matlab, then the equivalent in DanceTracker is H = 90.

In newer versions of MADTraC the user can right click on the image and bring up a dialog showing the pixel values of the pixel that was clicked.  This is pretty handy for finding thresholds!  I.e., view the "H" frame and see what the pixel values are for the hats, etc.


## State estimation 

Parameters related to state estimation:

* Overlap Factor - How close blobs can be (as a fraction of the sum of their radii) before we try to check if they are really a single dancer.  You can play with the value; something around 1.0 should be good.  

* Search Area Padding - How far away from the last position do we allow the new position to be?  There's a trade-off here - small values keep the tracker from "jumping", but make it harder for it to recover when it misses a few steps.  Large values make recovery better but can lead to more jumping.  Units are in pixels.

* Position Disturbance Sigma - The tracker tries to predict the next position of the dancer based on its velocity.  This variable controls how much we trust that prediction.  A high value means we don't trust it.  Units are in pixels.

* Speed Disturbance Sigma - Like the Position Disturbance Sigma, but for the speed (both in X and Y directions).  Units are pixels / frame.  

* Position Measurement Sigma - This is how much we trust the measured position, in units of pixels.  

There's a trade-off between Position Disturbance Sigma - let's call it sigma_d - and Position Measurement Sigma - let's call it sigma_m.  If we set sigma_m > sigma_d, we're telling the tracker that we 'trust' the prediction more than we trust the measurement.  The tracker will be more robust to noise, but it won't track the dancer really well if he/she changes direction suddenly.  The tracker might also have trouble recovering if it loses its lock on a dancer.   On the other hand, if we set sigma_m < sigma_d, then we're telling the tracker that we 'trust' the measurement more than we trust the prediction.  The tracker will be less robust to noise, but it will do a better job of tracking sudden changes in velocity.  It will also snap back more quickly after a loss of tracking.  In a perfect world, our measurements would be perfect, and this is what we would want.  But the world isn't perfect :(  

If you've ever studied LQR control, the disturbance sigmas are a lot like the values in the 'Q' matrix and the measurement sigmas are a lot like the values in the 'R' matrix.  
