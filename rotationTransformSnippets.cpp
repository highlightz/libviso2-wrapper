#include "libviso2_wrapper.h"

#include <Eigen/Eigen>

#include <iostream>

namespace RotationTransform
{
  // Refer to quat2euler.m in Matlab
  // The rotating order param is 'xyz'
  void threeAxisRot( double r11, double r12, double r21, double r31, double r32,
		     double& r1, double& r2, double& r3 )
  {
	  r1 = atan2( r11, r12 );
	  r2 = asin( r21 );
	  r3 = atan2( r31, r32 );
  }
  
  void rot2euler( Matrix pose, 
                  double& eu1, double& eu2, double& eu3 )
  {
    // Conversion from rotation matrix to quaternion
	  double qw = 0.5 * sqrt( 1 + pose.val[0][0] + pose.val[1][1] + pose.val[2][2] );
	  double qx = ( pose.val[2][1] - pose.val[1][2] ) / 4 / qw;
	  double qy = ( pose.val[0][2] - pose.val[2][0] ) / 4 / qw;
	  double qz = ( pose.val[1][0] - pose.val[0][1] ) / 4 / qw;
	  
	  // Only to validate that the mapping between quaternion and rotation matrix is one-to-one.
	  Eigen::Matrix3d rot_mat_from_quaternion;
	  rot_mat_from_quaternion << 1 - 2 * qy * qy - 2 * qz * qz, 2 * ( qx * qy - qz * qw ), 2 * ( qx * qz + qy * qw ),
		                     2 * ( qx * qy + qz * qw ), 1 - 2 * qx * qx - 2 * qz * qz, 2 * ( qy * qz - qx * qw ),
				     2 * ( qx * qz - qy * qw ), 2 * ( qy * qz + qx * qw ), 1 - 2 * qx * qx - 2 * qy * qy;
    
    	// Display them if you want to.
    	//std::cout << "Raw pose: " << std::endl << pose << std::endl;
    	//std::cout << "From quaternion: " << std::endl << rot_mat_from_quaternion << std::endl;								               
	  
	// Prepare data for function threeAxisRot
	double r11 = -2 * ( qy * qz + qw * qx );
	double r12 = qw * qw - qx * qx - qy * qy + qz * qz;
	double r21 = 2 * ( qx * qz + qw * qy );
	double r31 = -2 * ( qx * qy - qw * qz );
	double r32 = qw * qw + qx * qx - qy * qy - qz * qz;
	  
	threeAxisRot( r11, r12, r21, r31, r32, eu1, eu2, eu3 );
  }
}
