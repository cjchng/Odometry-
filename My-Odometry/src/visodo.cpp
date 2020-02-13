
/*

Monocular Video Odometry
By : Raghav,Shivani
Guided by: Prof. Cjchang
Date: 20th January 2020
Referred by: AviSingh
 */


#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)

//edited line 23
 double getAbsoluteScale(int frame_id, int sequence_id, double z_cal, double *cur_x, double *cur_y, char *POSES)	 {
  
  string line;
  int i = 0;
  ifstream myfile (POSES);
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }
  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
}


 //edited line 60,61
int main( int argc, char** argv )	{
  //edit the image path and the pose path in run configuration
  char *POSES= argv[1];
  char *IMAGES= argv[2];

  // Copy from mez odometry line 65,66,67,68
  //defining renderFeatures and use it below to draw featuresas markers
  bool renderFeatures = false;
  if (argc > 1) {
    renderFeatures = true;
  }

  Mat img_1, img_2;
  Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

  ofstream myfile;
  myfile.open ("result1_1.txt");

  double scale = 1.00;
  char filename1[200];
  char filename2[200];

  //edited line 81,82
  sprintf(filename1,"%s%d.png",IMAGES, 1);
  sprintf(filename2,"%s%d.png",IMAGES, 2);
  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);

  //read the first two frames from the dataset
  Mat img_1_c = imread(filename1);
  Mat img_2_c = imread(filename2);


  if ( !img_1_c.data || !img_2_c.data ) {
    std::cout<< " --(!) Error reading images " << std::endl; return -1;
  }

//edited lien 99-116
  string resultFile ="result1_1.txt";
   //obtain truth for plot comparison
   string posePath =  POSES;
   std::ifstream infile(posePath.c_str());
   std::string line;

    float truthPosition[3] ;
    Mat truthOrientation;

    getline(infile, line);
    getPosition(line, truthPosition);

    // Open a txt file to store the results
    ofstream fout(resultFile.c_str());
    if (!fout) {
        cout << "File not opened!" << endl;
        return 1;
    }
  // we work with grayscale images
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
  featureDetection(img_1, points1);        //detect features in img_1
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

  // WARNING: different sequences in the Dataset have different intrinsic/extrinsic parameters
  double focal = 405.6681;
  cv::Point2d pp(419.4670, 386.9750);
  //recovering the pose and the essential matrix
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;

  char filename[100];

  R_f = R.clone();
  t_f = t.clone();

  clock_t begin = clock();

  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

  // edited to increase the output window size line 157
  Mat traj = Mat::zeros(1200, 1200, CV_8UC3);

  for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)	{

	 //edited line 157,158,159

	  sprintf(filename, "%s%d.png",IMAGES, numFrame);
	  getline(infile, line);
	  getPosition(line, truthPosition);

  	Mat currImage_c = imread(filename);
  	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
  	vector<uchar> status;
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


   for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
  		prevPts.at<double>(0,i) = prevFeatures.at(i).x;
  		prevPts.at<double>(1,i) = prevFeatures.at(i).y;

  		currPts.at<double>(0,i) = currFeatures.at(i).x;
  		currPts.at<double>(1,i) = currFeatures.at(i).y;
    }

   //edited line 183,184,185

   //This is cheating because ideally you'd want to figure out a way to get scale, but without this cheat there is a lot of drift.
    double cur_x ;
    double cur_y ;
  	scale = getAbsoluteScale(numFrame, 0, t.at<double>(2), &cur_x, &cur_y, POSES);

    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

    	 R_f = R*R_f ;
    	 t_f = t_f +scale*(R_f*t);
    }

  //  myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

    //Make sure we have enough features to track
  // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
 	  if (prevFeatures.size() < MIN_NUM_FEAT)	{
 	  featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
      }

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    // edited line 211,212,213

    int xTruth = int(truthPosition[0])+ 300;
    int yTruth = int(truthPosition[2])+ 100;
    circle(traj, Point(xTruth, yTruth), 0.2, CV_RGB(0,255,0), 2);

   rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

     //copy from mez odometry line 221,222,223,224
    //Draw features as markers
    if (renderFeatures){
      for(auto point: currFeatures)
        cv::drawMarker(currImage_c, cv::Point(point.x,point.y), CV_RGB(0,255,0), cv::MARKER_TILTED_CROSS, 2, 1, cv::LINE_AA);
    }

    imshow( "Road facing camera", currImage_c );
    imshow( "Trajectory", traj );

    waitKey(1);

  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

 // edited to stop the output frame and let user press a key to exit the output window
  cv::waitKey(0);
  return 0;
}
