// To use this wrapper class, put all libviso2 .h and .cpp files in directory 'libviso2'.
// Refer to 'http://www.cvlibs.net/software/libviso/'

#ifndef LIBVISO2_WRAPPER_H
#define LIBVISO2_WRAPPER_H

// System Includes
#include <string>

#include <vector>
using std::vector;

// OpenCV Includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "structures.h"

// libviso2 Includes
#include "viso_stereo.h"

class libviso2_wrapper
{
public:
	// Construction
	libviso2_wrapper( VisualOdometryStereo::parameters aParam );
	
	// The principal driver for running libviso2
	void run( iplImageWrapper& left_img, iplImageWrapper& right_img );
	
	// For displaying odometry computating statistics
	double getInliers( ) const;

	// Return the [R, t] matrix
	Matrix getPose ( ) const;

	// Return yaw to class user
	double yaw( );

	// @adjustableScale is used for adjusting the curve's scale
	// when running on a large-scale area, this param should be set small.
	void drawOdometryCurve( cv::Mat& bkground, double adjustableScale = 1.5, int gridSize = 16 );
	
	void drawCurrentHeading( cv::Mat& bkground );

	void cumulatePose( Matrix& cp, double& ca );
private:
	// Set most important visual odometry parameters,
	// for a full parameter list, look at: viso_stereo.h
	VisualOdometryStereo::parameters param;

	VisualOdometryStereo m_viso;

	// Current pose (this matrix transforms a point from the current
	// frame's camera coordinates to the first frame's camera coordinates)
	Matrix pose;

	// Used to compute euler angles
	void threeAxisRot( double r11, double r12, double r21, double r31, double r32,
		               double& r1, double& r2, double& r3 );

	double inliers;

	// This struct stores the odom info of a node,
	// where reinitialization happens.
	struct HistoryOdom
	{
		double hX;
		double hY;
		double hZ;

		double hYaw;

		Matrix hPose;
	};
	// A series of odom info nodes
	vector< HistoryOdom > historyOdoms;	

	// Whenever an angle limit is reached, reinitialize the pose
	void reinitializePose( );	
private:
	// 用于在里程曲线画布上绘制规则的网格，以标定里程值和图像像素距离的关系
	// 参数：
	// @gridSize 设定网格单元的尺寸，默认值是16
	void drawGrid( cv::Mat& bkground, int gridSize = 16 );
private:
	void rotmat_to_euler( double R[9], double ola[3] );  // To be validated
};

#endif
