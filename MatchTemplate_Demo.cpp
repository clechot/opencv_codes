/**
 * @file MatchTemplate_Demo.cpp
 * @brief Sample code to use the function MatchTemplate
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <limits>

using namespace std;
using namespace cv;

/// Global Variables
Mat img; Mat templ; Mat result;
Mat mask;
const char* image_window = "Source Image";
const char* result_window = "Result window";

//int match_method = TM_SQDIFF;
int match_method = TM_CCORR_NORMED;
int max_Trackbar = 5;
int thresh_background = 534;
double dNan = numeric_limits<double>::quiet_NaN();

/// Function Headers
void MatchingMethod( int, void* );
void Maskthreshold();
//void TemplateMasking();

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load image and template
  img = imread( argv[1], 1 );
  templ = imread( argv[2], 1 );

  /// Create windows
  namedWindow( image_window, WINDOW_AUTOSIZE );
  namedWindow( result_window, WINDOW_AUTOSIZE );

  /// Create Trackbar
  /*const char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
  createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );*/

  MatchingMethod( 0, 0 );

  waitKey(0);
  return 0;
}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MatchingMethod( int, void* )
{
  /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  /// Create the mask
  Maskthreshold();

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method, mask);
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  cout << "The max value is:  " << maxVal << endl;
  cout << "The min value is:  " << minVal << endl;
  cout << "The max location is:  " << maxLoc << endl;
  cout << "The min location is:  " << minLoc << endl;

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  // control
  int sum_channels = 0;
  for(int j=0;j<templ.cols;j++)
	{
		for(int i=0;i<templ.rows;i++)
		{
			sum_channels = templ.at<Vec3b>(i,j)[0] + templ.at<Vec3b>(i,j)[1] + templ.at<Vec3b>(i,j)[2];
			if(sum_channels >= thresh_background)
			{
				rectangle( img_display, Point(j,i), Point( j+1,i+1 ), Scalar::all(0), 2, 8, 0 );
			}
			else
			{
				rectangle( img_display, Point(j,i), Point( j+1,i+1 ), Scalar::all(255), 2, 8, 0 );
			}
			sum_channels = 0;
		}
	}

  imshow( image_window, img_display );
  imshow( result_window, result );

  return;
}

void Maskthreshold()
{
	int sum_channels = 0;
	mask = Mat(templ.rows,templ.cols,templ.type(),Scalar::all(1));
	
	for(int j=0;j<templ.cols;j++)
	{
		for(int i=0;i<templ.rows;i++)
		{
			sum_channels = templ.at<Vec3b>(i,j)[0] + templ.at<Vec3b>(i,j)[1] + templ.at<Vec3b>(i,j)[2];
			if(sum_channels >= thresh_background)
			{
				mask.at<Vec3b>(i,j) = dNan;
			}
			sum_channels = 0;
		}
	}

	cout << "Masked" << endl;
}

/*void TemplateMasking()
{
	// Create the mask
	int mask_cols = templ.cols;
	int mask_rows = templ.rows;
	Mat mask(mask_rows,mask_cols,templ.type(),Scalar::all(0));

	int i = 0;
	cout << numeric_limits<double>::has_quiet_NaN;

	double dNan = numeric_limits<double>::quiet_NaN();
	for(i=0;i<templ.cols/2;i++)
	{
		mask.col(i) = dNan;
	}

	// Mask the template image
	multiply(templ,mask,templ);
}*/
