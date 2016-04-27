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
Mat img; Mat templ; Mat result; Mat img_depth; Mat templ_depth;
Mat new_img; Mat new_templ;
Mat mask; Mat resized_templ; Mat resized_mask;
const char* image_window = "Source Image";
const char* result_window = "Result window";
const char* template_window = "Template";
const char* mask_window = "Mask";
const char* resized_template_window = "Resized Template";
const char* resized_mask_window = "Resized Mask";

//int match_method = TM_SQDIFF;
int match_method = TM_CCORR_NORMED;
int max_Trackbar = 5;
int white = 254;
double dNan = numeric_limits<double>::quiet_NaN();

/// Function Headers
void MatchingMethod( int, void* );
void Maskthreshold();
void Mergechannels();

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load image and template
  img = imread( argv[1], 1 );
  templ = imread( argv[2], 1 );

  img_depth = imread( argv[3], 1);
  templ_depth = imread( argv[4] ,1);

  /// Create windows
  namedWindow( image_window, WINDOW_AUTOSIZE );
  namedWindow( result_window, WINDOW_AUTOSIZE );
  namedWindow( template_window, WINDOW_AUTOSIZE );
  namedWindow( mask_window, WINDOW_AUTOSIZE );
  namedWindow( resized_template_window, WINDOW_AUTOSIZE );
  namedWindow( resized_mask_window, WINDOW_AUTOSIZE );

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
  //cvtColor(img_display,img_display,CV_BGR2HSV);
  //cvtColor(templ_display,templ_display,CV_BGR2HSV);

  /// Conversion Depth to Gray
  cvtColor(img_depth,img_depth,CV_BGR2GRAY);
  cvtColor(templ_depth,templ_depth,CV_BGR2GRAY);

  /// Merge the 3 channels image and the depth channel image into one 4 channels image
  Mergechannels();

  /// Create the mask in HSV
  Maskthreshold();

  Mat resized_templ_display;
  resized_templ.copyTo( resized_templ_display );

  /// Do the Matching and Normalize
  //matchTemplate( new_img, resized_templ, result, match_method);
  matchTemplate( new_img, resized_templ, result, match_method, resized_mask);
  //normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// Score
  cout << "Match method: " << match_method << endl;
  cout << "Maximum value (for CCORR_NORMED 3): " << maxVal << endl;
  cout << "Minimum value (for SQDIFF 0): " << minVal << endl;

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + resized_templ.cols , matchLoc.y + resized_templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + resized_templ.cols , matchLoc.y + resized_templ.rows ), Scalar::all(0), 2, 8, 0 );

  //rectangle( mask, Point(219,142),Point(478,423),Scalar::all(254), 2, 8, 0);

  imshow( image_window, img_display );
  imshow( result_window, result );
  imshow( template_window, templ_display );
  imshow( mask_window, mask );
  imshow( resized_template_window, resized_templ_display );
  imshow( resized_mask_window, resized_mask );

  return;
}

void Mergechannels()
{
	new_img = Mat(img.rows, img.cols, CV_8UC4);
	new_templ = Mat(templ.rows, templ.cols, CV_8UC4);
	int from_to1[]={0,0,1,1,2,2};
	int from_to2[]={0,3};

	mixChannels(&img,1,&new_img,1,from_to1,3);
	mixChannels(&img_depth,1,&new_img,1,from_to2,1);
	mixChannels(&templ,1,&new_templ,1,from_to1,3);
	mixChannels(&templ_depth,1,&new_templ,1,from_to2,1);

	cout << "Channels mixed" << endl;
}

void Maskthreshold()
{
	mask = Mat(new_templ.rows,new_templ.cols,new_templ.type(),Scalar::all(white));

	for(int j=0;j<new_templ.cols;j++)
	{
		for(int i=0;i<new_templ.rows;i++)
		{
			if(new_templ.at<Vec4b>(i,j)[0]==0)
			{
				mask.at<Vec4b>(i,j) = dNan;
			}
		}
	}

	// Test if the depth channel does something
	// for(int j=0;j<new_templ.cols;j++)
	// {
	// 	for(int i=0;i<new_templ.rows;i++)
	// 	{
	// 		//new_templ.at<Vec4b>(i,j)[0]=0;
	// 		//new_templ.at<Vec4b>(i,j)[1]=0;
	// 		//new_templ.at<Vec4b>(i,j)[2]=0;
	// 		//new_templ.at<Vec4b>(i,j)[3]=0; // no depth channel
	// 	}
	// }

	/// Find the minimum template size (upper left pixel and bottom right pixel)
	Point pixel_ul(mask.rows,mask.cols);
	Point pixel_br(0,0);

	for(int k=0;k<mask.cols;k++)
	{
		for(int l=0;l<mask.rows;l++)
		{
			if(mask.at<Vec4b>(l,k)[0]==white)
			{
				if(k<pixel_ul.x)
				{
					pixel_ul.x=k;
				}
				if(k>pixel_br.x)
				{
					pixel_br.x=k;
				}
				if(l<pixel_ul.y)
				{
					pixel_ul.y=l;
				}
				if(l>pixel_br.y)
				{
					pixel_br.y=l;
				}
			}
		}
	}

	Mat rect_templ = new_templ(Rect(pixel_ul.x,pixel_ul.y,pixel_br.x-pixel_ul.x+1,pixel_br.y-pixel_ul.y+1));
	rect_templ.copyTo(resized_templ);

	Mat rect_mask = mask(Rect(pixel_ul.x,pixel_ul.y,pixel_br.x-pixel_ul.x+1,pixel_br.y-pixel_ul.y+1));
	rect_mask.copyTo(resized_mask);

	/*cout << "Pixel upper left x: " << pixel_ul.x << endl;
	cout << "Pixel upper left y: " << pixel_ul.y << endl;
	cout << "Pixel bottom right x: " << pixel_br.x << endl;
	cout << "Pixel bottom right y: " << pixel_br.y << endl;*/
	cout << "Masked and resized" << endl;
}