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
const char* template_window = "Template";
const char* mask_window = "Mask";

int match_method = TM_SQDIFF;
//int match_method = TM_CCORR_NORMED;
int max_Trackbar = 5;
int thresh_background = 534;
double dNan = numeric_limits<double>::quiet_NaN();

/// Function Headers
void MatchingMethod( int, void* );
void Maskthreshold();

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
  namedWindow( template_window, WINDOW_AUTOSIZE );
  namedWindow( mask_window, WINDOW_AUTOSIZE );

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

  Mat templ_display;
  templ.copyTo( templ_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  /// Conversion RGB to HSV
  cvtColor(img,img,CV_BGR2HSV);
  cvtColor(templ,templ,CV_BGR2HSV);

  cvtColor(img_display,img_display,CV_BGR2HSV);
  cvtColor(templ_display,templ_display,CV_BGR2HSV);

  /// Create the mask in HSV
  Maskthreshold();

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method, mask);
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  // Informations
  /*cout << "The max value is:  " << maxVal << endl;
  cout << "The min value is:  " << minVal << endl;
  cout << "The max location is:  " << maxLoc << endl;
  cout << "The min location is:  " << minLoc << endl;*/

  /*cout << "Template size: " << templ.size() << endl;
  cout << "Template column: " << templ.col(200) << endl;*/

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  imshow( image_window, img_display );
  imshow( result_window, result );
  imshow( template_window, templ_display );
  imshow( mask_window, mask );

  return;
}

void Maskthreshold()
{
	mask = Mat(templ.rows,templ.cols,templ.type(),Scalar::all(1));

	for(int j=0;j<templ.cols;j++)
	{
		for(int i=0;i<templ.rows;i++)
		{
			if(templ.at<Vec3b>(i,j)[0]==0)
			{
				mask.at<Vec3b>(i,j) = dNan;
			}
		}
	}

	/// Reduce the template size
	Point pixel_ul(0,0);
	Point pixel_br(0,0);

	for(int j=0;j<mask.cols;j++)
	{
		for(int i=0;i<mask.rows;i++)
		{
			if(mask.at<Vec3b>(i,j)[0]==1)
			{
				if(pixel_ul.x == 0)
				{
					pixel_ul.x = i;
				}
				if(pixel_ul.y == 0)
				{
					pixel_ul.y = j;
				}
			}
			if(mask.at<Vec3b>(mask.rows-i,mask.cols-j)[0]==1)
			{
				if(pixel_br.x == 0)
				{
					pixel_br.x = mask.rows-i;
				}
				if(pixel_br.y == 0)
				{
					pixel_br.y = mask.cols-j;
				}
			}
		}
	}

	//TOOO resize()


	/*cout << "Pixel upper left x: " << pixel_ul.x << endl;
	cout << "Pixel upper left y: " << pixel_ul.y << endl;
	cout << "Pixel bottom right x: " << pixel_br.x << endl;
	cout << "Pixel bottom right y: " << pixel_br.y << endl;*/
}