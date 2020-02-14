## Odometry 

BY- Shivani, Raghav
Email: baldwashivani@gmail.com

Here we use  Avi singh MONO-VO code "https://github.com/avisingh599/mono-vo" as a reference to build our odometry.
Also refered Monocular-Visual-Odometry code by mez "https://github.com/mez/monocular-visual-odometry".

Reference Codes :
1) https://github.com/avisingh599/mono-vo
2) https://github.com/mez/monocular-visual-odometry

Links to clone the above codes : 

1)https://github.com/mez/monocular-visual-odometry.git 

2)https://github.com/avisingh599/mono-vo.git

Problem with te Reference Codes:

Code 1) The code by the Avi Singh is the basic code to develop odometry which only shows the "ground truth trajectory" and do not shows the Visual odometry.

Code 2) The code by the mez is the code what we actually want but the problem with the code is that "it is limited to some sequences which are developed by kittydataset" and not work for our own dataset.

Link to Kitty dataset: "http://www.cvlibs.net/datasets/kitti/eval_odometry.php"
Link to My Dataset: "https://bit.ly/2OAqTuz" 

**Our Solution:**

**To overcome the above problem and use our own dataset to build odometry we merge both the above codes(1 & 2) together and created our own code.  This code is doing exact what we want but there is some bug in the code because the ground truth and the actual position trajectory is going in opposite direction. And we are unable to find the bug where we did wrong.** 

Our code: My-Odometry Mentioned above
<p align="center">
  <img src="https://github.com/Shivani1796/Odometry-/blob/master/AviSingh/1.png">
</p>

Steps to Compile and run:

1) Open Terminal
2) git clone 
3) cd into the cloned project folder 
4) mkdir build & cd build
5) cmake ..
6) make 
7) Run Execuatable file

*Remember: Change img_path and pose_path to correct image sequences and pose file paths. 
	  Ensure focal length and principal point information is correct* 
          
The changes what we made in our program is mentioned in the code with the help of " // ".

