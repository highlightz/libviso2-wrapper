// This test file includes two demo mode - online and offline.
// For online demo, bb2_wrapper class is required,
// which can be found at 'https://github.com/highlightz/Bumblebee2-Stereo-Camera-Driver-Class-for-Windows'.
// For offline demo, a series of stereo images are needed, as stated later.

#define ONLINE 1

#if ONLINE
//
//  Documented C++ sample code of stereo visual odometry (modify to your needs)
//

// System Includes
#include <iostream>
#include <fstream>
#include <sstream>

// libviso2_wrapper Includes
#include "libviso2_wrapper.h"

// StereoCamera Includes
#include "bb2_wrapper/bb2_wrapper.h"

//#define S2C38
//#define HIGH_DEF
#ifdef HIGH_DEF
 #define WIDTH 1024
 #define HEIGHT 768
#else
 #define WIDTH 640
 #define HEIGHT 480
#endif

// A class that records odometry data.
#include "log_wrapper.h"

// Camera class relavant declarations
bb2_wrapper m_camera( WIDTH, HEIGHT );
IplImage* pframeL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
IplImage* pframeR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
IplImage* pfL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
IplImage* pfR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
IplImage* pGrayL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
IplImage* pGrayR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );

// Draw odometry curve on this image
cv::Mat odomCurve( 480, 320, CV_8UC3 );

// Note: struct odometry is defined in file 'libviso2_wrapper.h'
log_wrapper< odometry > dataLogger( "D:\\HighlightClark\\dog_data\\" );

// Some tool functions are defined here.
namespace miscellaneous
{
  double Get_sys_time(  )
  {
  	LARGE_INTEGER litmp;
  	long f;
  	long long p;
  	QueryPerformanceFrequency(&litmp);
  	f=(long)litmp.QuadPart;
  	QueryPerformanceCounter(&litmp);
  	p=(long long)litmp.QuadPart;
  
  	return p*(1./f);
  }
}

double miscellaneous::Get_sys_time( );

int main( )
{
	// Calibration parameters for bb2 
	VisualOdometryStereo::parameters param;
	
#ifdef S2C38
	#ifdef HIGH_DEF
		double f  = 786.66; 
		double cu = 515.40; 
		double cv = 393.52; 
		double parambase = 0.12;
	#else
		double f  = 491.66; 
		double cu = 321.94; 
		double cv = 245.76; 
		double parambase = 0.12;
	#endif
#else  // S2C25
	#ifdef HIGH_DEF
		double f  = 445.90; 
		double cu = 507.51; 
		double cv = 395.32; 
		double parambase = 0.12;
	#else
		double f  = 278.69; 
		double cu = 317.00; 
		double cv = 246.89; 
		double parambase = 0.12;
	#endif
#endif

	param.calib.f  = f;  // focal length in pixels
	param.calib.cu = cu;  // principal point (u-coordinate) in pixels
	param.calib.cv = cv;  // principal point (v-coordinate) in pixels
	param.base     = parambase;  // baseline in meters

	libviso2_wrapper libviso2( param );
	
	// Only for validating
	std::cout << "Camera parameters: " << std::endl;
	std::cout << "baseline: " << param.base     << std::endl
		         << "focal: " << param.calib.f  << std::endl
		         << "cu: "    << param.calib.cu << std::endl
		         << "cv: "    << param.calib.cv << std::endl;

	// Start camera
	if ( !m_camera.StartCamera( ) )
	{
		std::cout << "StartCamera failed!" << std::endl;
		return -1;
	}
	
	// Set an initial counter for the start_log_image_pair method, 
	// which is required by class log_wrapper.
	long img_counter = 0;
	
	m_camera.showCameraInfo( );
	m_camera.EnableStereoMatch( );

	// Main processing loop
	while ( 1 )
	{
		if ( m_camera.AcquireFrame( ) && m_camera.StereoMatch( ) )
		{
			pframeL = m_camera.GetRetfImgL( );
			pframeR = m_camera.GetRetfImgR( );
			m_camera.Four2Three( pframeL, pfL );
			m_camera.Four2Three( pframeR, pfR );

			// Start logging image pair
			dataLogger.start_log_image_pair( img_counter, pfL, pfR );

			cvShowImage( "lc", pfL );
			cvShowImage( "rc", pfR );
	
			cvCvtColor( pfL, pGrayL, CV_BGR2GRAY );
			cvCvtColor( pfR, pGrayR, CV_BGR2GRAY );
		}
	
		iplImageWrapper left_image( pGrayL );
		iplImageWrapper right_image( pGrayR );
			
		cvShowImage( "l", left_image.pic );
		cvShowImage( "r", right_image.pic );

		libviso2.run( left_image, right_image );
		
		odometry anOdom = libviso2.getOdometry(  );
		Matrix aPose = libviso2.getPose(  );
		std::cout << aPose << std::endl << std::endl;
		
		// Start the odometry logger
		dataLogger.start_log_odom( anOdom );
		
		libviso2.drawOdometryCurve( odomCurve );
		cv::imshow( "Odometry", odomCurve );

		const double REINITIALIZE_THRESHOLD = 15.0;
		if ( libviso2.computeDurationDistance( ) > REINITIALIZE_THRESHOLD )
		{
			libviso2.reinitializePose( );
		}

		// Update the logger counter
		img_counter++;
		
		if ( cvWaitKey( 20 ) == 27 ) 
		{
			  break;
		}
	}	

	return 0;
}

#else  // OFFLINE
//
//  Documented C++ sample code of stereo visual odometry (modify to your needs)
//  To run this demonstration, download the Karlsruhe dataset sequence
//  '2010_03_09_drive_0019' from: www.cvlibs.net!
//  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
//

// System Includes
#include <iostream>
#include <fstream>
#include <string>

// libviso2_wrapper Includes
#include "libviso2_wrapper.h"

#define DEMO_KITTI

//#define DEMO_DOGDATA
#define S2C38
#define HIGH_DEF

#include "log_wrapper.h"
log_wrapper< odometry > dataLogger( "D:\\HighlightClark\\dog_data\\", static_cast< std::string >( "offline_odom.txt" ) );

int main( int argc, char** argv )
{
	// Sequence directory
#ifdef DEMO_KITTI
	std::string dir = "D:\\HighlightClark\\2010_03_09_drive_0019";
#endif

#ifdef DEMO_DOGDATA
	std::string dir = "D:\\HighlightClark\\dog_data";
#endif

	// Set most important visual odometry parameters
	// for a full parameter list, look at: viso_stereo.h
	VisualOdometryStereo::parameters param;
	
#ifdef DEMO_KITTI
	// calibration parameters for sequence 2010_03_09_drive_0019 
	param.calib.f  = 645.24;  // Focal length in pixels
	param.calib.cu = 635.96;  // Principal point (u-coordinate) in pixels
	param.calib.cv = 194.13;  // Principal point (v-coordinate) in pixels
	param.base     = 0.5707;  // Baseline in meters
#endif

#ifdef DEMO_DOGDATA
	#ifdef S2C38
		#ifdef HIGH_DEF
			double f  = 786.66; 
			double cu = 515.40; 
			double cv = 393.52; 
			double parambase = 0.12;
		#else
			double f  = 491.66; 
			double cu = 321.94; 
			double cv = 245.76; 
			double parambase = 0.12;
		#endif
	#else  // S2C25
		#ifdef HIGH_DEF
			double f  = 445.90; 
			double cu = 507.51; 
			double cv = 395.32; 
			double parambase = 0.12;
		#else
			double f  = 278.69; 
			double cu = 317.00; 
			double cv = 246.89; 
			double parambase = 0.12;
		#endif
	#endif
	
	param.calib.f  = f; // focal length in pixels
	param.calib.cu = cu; // principal point (u-coordinate) in pixels
	param.calib.cv = cv; // principal point (v-coordinate) in pixels
	param.base     = parambase; // baseline in meters
#endif

	libviso2_wrapper libviso2( param );

	// Loop through all frames i=0:372
	for ( int32_t i = 0; i < 373; i++ ) 
	{
		// Input file names from kitti
#ifdef DEMO_KITTI		
		char base_name[256]; sprintf(base_name,"%06d.png",i);
		std::string left_img_file_name  = dir + "/I1_" + base_name;
		std::string right_img_file_name = dir + "/I2_" + base_name;
#endif
		// Input file names from dog_data
#ifdef DEMO_DOGDATA
		char base_name[256]; sprintf( base_name, "%d.jpg", i );
		std::string left_img_file_name = dir + "/L_" + base_name;
		std::string right_img_file_name = dir + "/R_" + base_name;
#endif
		// Load left and right input image
		iplImageWrapper left_image( left_img_file_name );
		iplImageWrapper right_image( right_img_file_name );
		
		cvShowImage( "l", left_image.pic );
		cvWaitKey( 5 );

		libviso2.run( left_image, right_image );

		odometry anOdom = libviso2.getOdometry(  );

		Matrix aPose = libviso2.getPose( );
		
		std::cout << aPose << std::endl << std::endl;

		// Start the odometry logger
		dataLogger.start_log_odom( anOdom );
	}
	return 0;
}

#endif
