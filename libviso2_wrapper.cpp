#include "libviso2_wrapper.h"

libviso2_wrapper::libviso2_wrapper( VisualOdometryStereo::parameters aParam ) : m_viso ( aParam ), param( aParam ) 
{
	pose = Matrix::eye(4);

	odom.x = 0;
	odom.z = 0;
	odom.y = 0;
	odom.yaw_rad = 0;
}

void libviso2_wrapper::run( iplImageWrapper& left_img, iplImageWrapper& right_img )
{
	// Image dimensions
	int32_t width  = left_img.get_width(  );
	int32_t height = left_img.get_height(  );

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
		
		// Updata current valid odom
		odom.x = pose.val[0][3];
		odom.y = pose.val[1][3];
		odom.z = pose.val[2][3];

		double Rot[9] = {pose.val[0][0], pose.val[0][1], pose.val[0][2],
						 pose.val[1][0], pose.val[1][1], pose.val[1][2],
						 pose.val[2][0], pose.val[2][1], pose.val[2][2]};

		double euler[3] = {0, 0, 0};
		rotmat_to_euler( Rot, euler );
		odom.yaw_rad = euler[0];
	}

	// Release uint8_t buffers
	free( left_img_data );
	free( right_img_data );
}

double libviso2_wrapper::getInliers( ) const
{
	return inliers;
}

odometry libviso2_wrapper::getOdometry( ) const
{
	return odom;
}

Matrix libviso2_wrapper::getPose ( ) const
{
	return pose;
}

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
