#include "libviso2_wrapper.h"

#include "../Eigen/Eigen"

#include <sstream>
using std::stringstream;

#include <string>
using std::string;

#include <math.h>

#include <iomanip>

#include <iostream>
using std::endl;
using std::cout;

libviso2_wrapper::libviso2_wrapper( VisualOdometryStereo::parameters aParam ) 
	: m_viso ( aParam ), 
	  param( aParam ) 
{
	pose = Matrix::eye(4);
}

// The principal driver for running libviso2
// Input: Left gray image, right gray image
void libviso2_wrapper::run( iplImageWrapper& left_img, iplImageWrapper& right_img )
{
	// Image dimensions
	int32_t width  = left_img.get_width( );
	int32_t height = left_img.get_height( );

	// Convert input images to uint8_t buffer
	uint8_t* left_img_data  = ( uint8_t* )malloc( width * height * sizeof( uint8_t ) );
	uint8_t* right_img_data = ( uint8_t* )malloc( width * height * sizeof( uint8_t ) );
	int32_t k = 0;

	for ( int32_t v = 0; v < height; v++ ) 
	{
		for ( int32_t u = 0; u < width; u++ ) 
		{
			left_img_data[k]  = left_img.get_pixel( u, v );
			right_img_data[k] = right_img.get_pixel( u, v );
			k++;
		}
	}

	// Compute visual odometry
	int32_t dims[] = { width, height, width };
	if ( m_viso.process( left_img_data, right_img_data, dims ) )
	{
		// On success, update current pose
		pose = pose * Matrix::inv( m_viso.getMotion( ) );
		
#if 0		
		double Rot[9] = {pose.val[0][0], pose.val[0][1], pose.val[0][2],
						 pose.val[1][0], pose.val[1][1], pose.val[1][2],
						 pose.val[2][0], pose.val[2][1], pose.val[2][2]};
		
		double euler[3] = {0, 0, 0};
		rotmat_to_euler( Rot, euler );

		std::cout<<"*"<<euler[0]<<std::endl<<"*"<<euler[1]<<std::endl<<"*"<<euler[2]<<std::endl<<std::endl;

		odom.yaw_rad = euler[1];
#endif
	}

	// Release uint8_t buffers
	free( left_img_data );
	free( right_img_data );
}

double libviso2_wrapper::getInliers( ) const
{
	return inliers;
}

Matrix libviso2_wrapper::getPose ( ) const
{
	return pose;
}

// Not used.
void libviso2_wrapper::rotmat_to_euler( double R[9], double ola[3] )
{
	// yaw(-pi pi]  pitch[-pi/2 pi/2] roll(-pi pi]
	const double rfpi = 3.1415926;
	
	double cp = sqrt( R[0] * R[0] + R[3] * R[3] );
	if ( cp == 0 )
	{	
		ola[0] = 0; 
	  if( R[6] >= 0 )
	  {
		  ola[1]=-rfpi/2; 
		}
	  else
	  {
	    ola[1]=rfpi/2;
	  }
	  if(R[4]!=0) 
	  {	
	    if ( ola[1] > 0 ) 
	      ola[2]=atan(R[1]/R[4]); 
	    else 
	      ola[2]=-atan(R[1]/R[4]);
	  
	    if(R[4]<0&&ola[2]<=0) 
	      ola[2]+=rfpi;
	    else if(R[4]<0&&ola[2]>0) 
	      ola[2]-=rfpi;	
	  }
	  else 
	  {	
	    if(R[1]>=0) 
	      ola[2]=ola[1];
	    else        
	      ola[2]=-ola[1];	
	  }
	}
	else
	{
		ola[1]=atan(-R[6]/cp);
		if ( R[0] != 0 ) 
		{
			ola[0]=atan(R[3]/R[0]);
			if(R[0]<0&&R[3]>=0) 
			  ola[0]+=rfpi;
			if(R[0]<0&&R[3]<0) 
			  ola[0]-=rfpi;
		}
		else
		{	
		  if ( R[3] >= 0 )  
		    ola[0] = rfpi/2;
		  else        
		    ola[0] = -rfpi/2;
		}
		if ( R[8] != 0 ) 
		{
			ola[2] = atan( R[7] / R[8] );
			if ( R[8] < 0 && R[7] >= 0 ) 
			  ola[2]+=rfpi;
			if ( R[8] < 0 && R[7] < 0 ) 
			  ola[2]-=rfpi;
		}
		else
		{
			if( R[7] >= 0 ) 
			  ola[2]=rfpi/2;
			else        
			  ola[2]=-rfpi/2;
		}
	}
}

// @ adjustableScale 默认为1.5
// @ gridSize 默认为16
void libviso2_wrapper::drawOdometryCurve( cv::Mat& bkground, double adjustableScale, int gridSize );
{
	const int WIDTH = 640;
    const int HEIGHT = 480;    
    bkground.create( cv::Size( WIDTH, HEIGHT ), CV_8UC3 );
	
	// 绘制网格
	drawGrid( bkground, gridSize );

    // Draw axis
    // Horizontal axis
    cv::line( bkground,
	          cv::Point( 10, bkground.rows / 2 ),
	          cv::Point( bkground.cols - 10, bkground.rows / 2 ),
	          cv::Scalar( 0, 255, 0 ), 2 );
	
	// Vertical axis
    cv::line( bkground,
	          cv::Point( bkground.cols / 2, 10 ),
	          cv::Point( bkground.cols / 2, bkground.rows - 10 ),
	          cv::Scalar( 0, 255, 0 ), 2 );
	          
	// Prepare data: coordinate transformation for convenient display
	Matrix cummulatedPose = Matrix::eye( 4 );
	for ( int i = 0; i < historyOdoms.size( ); i++ )
	{
		cummulatedPose = cummulatedPose * historyOdoms[i].hPose;
	}

	cummulatedPose = cummulatedPose * pose;

	double x_display = cummulatedPose.val[0][3];
	double y_display = cummulatedPose.val[2][3];
	
	x_display =  x_display * adjustableScale + bkground.cols / 2;
    y_display = -y_display * adjustableScale + bkground.rows / 2;
	          
    cv::circle( bkground, cv::Point( x_display, y_display ), 1, cv::Scalar( 0, 0, 255 ) );
}

void libviso2_wrapper::drawCurrentHeading( cv::Mat& bkground )
{
	const int WIDTH = 640;
    const int HEIGHT = 480;    
    bkground.create( cv::Size( WIDTH, HEIGHT ), CV_8UC3 );

	// This parameter is used for adjusting the curve's scale
    const double adjustableScale = 80.0;
		
    // Draw axis
    // Horizontal axis
    cv::line( bkground,
	          cv::Point( 10, bkground.rows / 2 ),
	          cv::Point( bkground.cols - 10, bkground.rows / 2 ),
	          cv::Scalar( 0, 255, 0 ), 2 );
	
	// Vertical axis
    cv::line( bkground,
	          cv::Point( bkground.cols / 2, 10 ),
	          cv::Point( bkground.cols / 2, bkground.rows - 10 ),
	          cv::Scalar( 0, 255, 0 ), 2 );
	          
	// Prepare angle data
	Matrix cummulatedPose = Matrix::eye( 4 );
	double cummulatedAngle = 0;  // The accumulated direction angle from beginning
	for ( int i = 0; i < historyOdoms.size( ); i++ )
	{
		cummulatedAngle += historyOdoms[i].hYaw;
		cummulatedPose = cummulatedPose * historyOdoms[i].hPose;
	}
	
	//cout << "Num of Angle: " << historyOdoms.size( ) << ", Total Angle: " << ( cummulatedAngle + yaw( ) ) * rad_to_deg << endl;

	cummulatedPose = cummulatedPose * pose;
	
	//cout << "Total Pose: " << endl << cummulatedPose << endl << endl;

	// Prepare data: coordinate transformation for convenient display
	double x_display = cos( cummulatedAngle + yaw( ) ) * adjustableScale;
	double y_display = sin( cummulatedAngle + yaw( ) ) * adjustableScale;
	
	x_display = x_display + bkground.cols / 2;
    y_display = y_display + bkground.rows / 2;

	// Draw the vector line
	cv::line( bkground, 
	          cv::Point( x_display, y_display ), 
	          cv::Point( bkground.cols / 2, bkground.rows / 2 ), 
	          cv::Scalar( 255, 0, 0 ), 4 );

	// Draw the end of the line
	cv::circle( bkground,
	            cv::Point( x_display, y_display ),
	            6, 
	            cv::Scalar( 255, 255, 0 ),
	            4 );
	
	// Show the cumulated yaw angle( in degrees )
	stringstream ss;
	ss << ( cummulatedAngle + yaw( ) ) * rad_to_deg;
	string yaw_reading_text;
	ss >> yaw_reading_text;
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 2;
	int thickness = 3;
	cv::Point textOrg( 50, 70 );
	cv::putText( bkground, 
	             yaw_reading_text,
	             textOrg,
	             fontFace,
	             fontScale,
	             cv::Scalar::all( 255 ),
	             thickness, 8 );
	             
	/*cv::circle( bkground,
	            cv::Point( 500, 30 ),
	            6, 
	            cv::Scalar::all( 255 ),
	            2 );*/
}

void libviso2_wrapper::reinitializePose( )
{
	// Record current direction angle
	HistoryOdom currOdom;
	currOdom.hX = pose.val[0][3];
	currOdom.hY = pose.val[1][3];
	currOdom.hZ = pose.val[2][3];
	currOdom.hYaw = yaw( );
	currOdom.hPose = pose;
	historyOdoms.push_back( currOdom );

	// Clear pose
	pose = Matrix::eye(4);
}

// Return yaw to class user
double libviso2_wrapper::yaw( )
{
	// Conversion from rotation matrix to quaternion
	double qw = 0.5 * sqrt( 1 + pose.val[0][0] + pose.val[1][1] + pose.val[2][2] );
	double qx = ( pose.val[2][1] - pose.val[1][2] ) / 4 / qw;
	double qy = ( pose.val[0][2] - pose.val[2][0] ) / 4 / qw;
	double qz = ( pose.val[1][0] - pose.val[0][1] ) / 4 / qw;

	//std::cout << std::fixed << std::setprecision( 3 );
	//std::cout << qw << ", " << qx << ", " << qy << ", " << qz << std::endl;

	//double eu4 = atan2( qy, qw ) * 2;
	//std::cout << eu4 << std::endl;

	Eigen::Matrix3d rot_mat_from_quaternion;
	rot_mat_from_quaternion << 1 - 2 * qy * qy - 2 * qz * qz,     2 * ( qx * qy - qz * qw ), 2 * ( qx * qz + qy * qw ),
		                           2 * ( qx * qy + qz * qw ), 1 - 2 * qx * qx - 2 * qz * qz, 2 * ( qy * qz - qx * qw ),
								   2 * ( qx * qz - qy * qw ),     2 * ( qy * qz + qx * qw ), 1 - 2 * qx * qx - 2 * qy * qy;
	//std::cout << "From quaternion: " << std::endl << rot_mat_from_quaternion << std::endl;


	// Conversion from quaternion to yaw:
	double eu1, eu2, eu3;
	// Prepare data for function threeAxisRot
	double r11 = -2 * ( qy * qz + qw * qx );
	double r12 = qw * qw - qx * qx - qy * qy + qz * qz;
	double r21 = 2 * ( qx * qz + qw * qy );
	double r31 = -2 * ( qx * qy - qw * qz );
	double r32 = qw * qw + qx * qx - qy * qy - qz * qz;
	threeAxisRot( r11, r12, r21, r31, r32, eu1, eu2, eu3 );
	
	//std::cout << eu1 * 180 / 3.14<< ",  " << eu2 * 180 / 3.14<< ", " << eu3 * 180 / 3.14 << std::endl;
	//std::cout << eu2 * 180 / 3.14 << std::endl;

	return eu2;
	
	//Eigen::Matrix3d rot_mat;
	//rot_mat << pose.val[0][0], pose.val[0][1], pose.val[0][2],
	//	         pose.val[1][0], pose.val[1][1], pose.val[1][2],
	//		     pose.val[2][0], pose.val[2][1], pose.val[2][2];

	//Eigen::Vector3d eulers = rot_mat.eulerAngles( 2, 1, 0 );
	//if(eulers(0)>M_PI)
	//	eulers(0) -= 2*M_PI;
	//if(eulers(1)>M_PI)
	//	eulers(1) -= 2*M_PI;
	//if(eulers(1)>M_PI)
	//	eulers(2) -= 2*M_PI;
	////std::cout << eulers << std::endl << std::endl;
	//std::cout << eulers * 180 / 3.14 << std::endl << std::endl;
	//return eulers( 1 );
}

void libviso2_wrapper::threeAxisRot( double r11, double r12, double r21, double r31, double r32,
		               double& r1, double& r2, double& r3 )
{
	r1 = atan2( r11, r12 );
	r2 = asin( r21 );
	r3 = atan2( r31, r32 );
}

void libviso2_wrapper::cumulatePose( Matrix& cp, double& ca )
{
	cp = Matrix::eye( 4 );
	ca = 0;

	for ( int i = 0; i < historyOdoms.size( ); i++ )
	{
		cp = cp * historyOdoms[i].hPose;
		ca += historyOdoms[i].hYaw;
	}

	cp = cp * pose;
	ca += yaw( );

	// 重初始化视觉里程计
	// **********************************************************************************
	// 重初始化导航结果信息的时机,Unit: deg
	const double REINITIALIZE_ANGLE_THRESHOLD = 85;  
	
	// 重初始化的条件是自上次初始化以来，
	// 转向角已经达到转向阈值（这里预设置为85度）
	// （Reinitialize visual odometry, 
	// when accumulated turning angle reach pre-defined threshold）																		   
	// 														   
	if (  rad_to_deg * yaw( ) > REINITIALIZE_ANGLE_THRESHOLD 	  															   
	   || rad_to_deg * yaw( ) < -REINITIALIZE_ANGLE_THRESHOLD )  	                                                           
	{																	  															   		  															   																  															   
		// Reinitialize odometry										  															   
		reinitializePose( );	
	}																	  															   
	// **********************************************************************************
}

void libviso2_wrapper::drawGrid( cv::Mat& bkground, int gridSize )
{
	const int WIDTH = 640;
	const int HEIGHT = 480;

	bkground.create( HEIGHT, WIDTH, CV_8UC3 );

	// 绘制水平线阵
	for ( int i = gridSize; i < HEIGHT; i += gridSize )
	{
		cv::line( bkground, cv::Point( 0, i ), cv::Point( WIDTH, i ), cv::Scalar( 255, 255, 255 ) );
	}

	// 绘制垂直线阵
	for ( int j = gridSize; j < WIDTH; j += gridSize )
	{
		cv::line( bkground, cv::Point( j, 0 ), cv::Point( j, HEIGHT ), cv::Scalar( 255, 255, 255 ) );
	}
}
