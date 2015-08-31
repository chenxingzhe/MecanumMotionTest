#README

This project uses visual odormeter to control robot's movement.
****
We use a very simple method to get robot's  position,a red label and a blue labe by
using two labels' world position and transformation of world ,camera,image coordinate.
This part of code is in the calclandmark.cpp.
***
And the code to record visual odormeter and using the record to navigate is in the MecanumMotion.cpp.
***
The file bwlabel.cpp contains the method to get correct color.

***
**ps**:This code rely on ROS and the code there is not complete.If you are interested in our work please contact me

*my email:chenxingwangzi@gamil.com*
