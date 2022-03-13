#include<vector>
#include"kerian_bringup/kerian.h"

#define HEADER 0xdd
#define ENDER  0xee


#define ENCODER_SAMPLE_TIME 10
#define WHEEL_RADIUS  0.09
#define LINE_COUNT 579
#define ENCODER_TO_VELOCITY WHEEL_RADIUS*ENCODER_SAMPLE_TIME/1000/LINE_COUNT
#define VELOCITY_TO_ENCODER 1000*LINE_COUNT/WHEEL_RADIUS/ENCODER_SAMPLE_TIME

//V= encoder *WHEEL_RADIUS*ENCODER_SAMPLE_TIME/1000/LINE_COUNT
//encoder = V*1000*LINE_COUNT/WHEEL_RADIUS/ENCODER_SAMPLE_TIME


#define ALPHA   0.5
#define ADCVALUE 32755

namespace Robot
{
	
	boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
	
	boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
	
	boost::asio::io_service iosev;
	boost::asio::serial_port sp(iosev,"/dev/ttyUSB0");
	
	Kerian::Kerian():Expect_Pos_X(0),Expect_Pos_Y(0),Expect_Pos_A(0),Expect_Vec_X(0),
	Expect_Vec_Y(0),Expect_Vec_A(0),Expect_Vec_Left(0),Expect_Vec_Right(0),Expect_Encoder_Left(0),
	Expect_Encoder_Right(0),Measure_Pos_X(0),Measure_Pos_Y(0),Measure_Pos_A(0),Measure_Pos_Last_A(0),
	Measure_Vec_X(0),Measure_Vec_A(0),Measure_Vec_Left(0),Measure_Vec_Right(0),Measure_Encoder_Left(0),
	Measure_Encoder_Right(0),Optinal_Pos_X(0),Optinal_Pos_Y(0),Optinal_Pos_A(0)
	{
		
	}
	
	Kerian::~Kerian()
	{
		
	}
	
	void Kerian::SetExceptVec(double Vec_X,double Vec_A)
	{
		Expect_Vec_A = Vec_A;
		Expect_Vec_X = Vec_X;
	}

	bool Kerian::init()
	{
		//set the serial port configure   
		sp.set_option(boost::asio::serial_port::baud_rate(115200));
		sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
		sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
		sp.set_option(boost::asio::serial_port::character_size(8));
	
		ros::Time::init();
		
		odomPub = nh.advertise<nav_msgs::Odometry>("odom",50);
		return true;
	}
	
	/*
		read data consist:
		header 				 8bit
		left wheel encoder 	 8bit
		right wheel encoder  8bit
		IMU yaw data         16bit
		cheke data           8bit
		ender 				 8bit
	*/
	
	bool Kerian::ReadSpeed()
	{
		int i=0;
		char buf[50];
		
		boost::asio::read(sp,boost::asio::buffer(buf));
		
		//chek the message header 
		if (buf[0] != HEADER)
		{
			ROS_ERROR("Received message header error!");
			return false;
		}
		
		if (buf[6]!=ENDER)
		{
			ROS_ERROR("Received message ender error!");
			return false;
		}
		
		if (!dataCheck(buf)==false)
		{
			ROS_ERROR("Received message bit error!");
			return 	false;
		}
		
		Measure_Encoder_Left = buf[1];
		Measure_Encoder_Right = buf[2];
		
		Measure_Pos_Last_A = Measure_Pos_A;
		Measure_Pos_A = (double)(  ((buf[3]<<8) || buf[4] )  / ADCVALUE);
		
		return true;
	}
	
	void Kerian::CalcDynamicWidth(double vecLeft,double vecRight)
	{
		//DynamicWidth=f(vecLeft,vecRight)
	}
	
	void Kerian::InverseCalcDynamicWidth(double Vec_x,double Vec_A)
	{
		//DynamicWidth=g(Vec_x,Vec_A)
	}
	
	void Kerian::MeasureEncoderToVelocity()
	{
		Measure_Vec_Left = Measure_Encoder_Left*ENCODER_TO_VELOCITY;
		Measure_Vec_Right = Measure_Encoder_Right*ENCODER_TO_VELOCITY;
	}
	
	/*  caculate the odemetry  
		if don't use the calarman fliter:
		calculate the odemetry by encoder value and IMU value
		
		if use the carman fliter:
		we will have a expedition speed of left wheel and right wheel
		then we calculate the expedition position in world axis by intergral
		acording to positive kinematics 
		and we use the measure value include encoder value and IMU value to calculate the measure positon
		then let this value as input value of calarman filter
	*/
	
	void Kerian::ExpectPositiveKinematics()
	{
		CalcDynamicWidth(Expect_Vec_Left,Expect_Vec_Right);
		Expect_Vec_X = (Expect_Vec_Left + Expect_Vec_Right)/2;
		Expect_Vec_A = (Expect_Vec_Right - Expect_Vec_Left)/2/DynamicWidth;
		/* calarman  fliter   */
	}
	
	
	void Kerian::MeasurePositiveKinematics()
	{
		MeasureEncoderToVelocity();
		CalcDynamicWidth(Measure_Vec_Left,Measure_Vec_Right);
		Measure_Vec_X = (Measure_Vec_Left + Measure_Vec_Right)/2;
		//Measure_Vec_A = (Measure_Vec_Right - Measure_Vec_Left)/2/DynamicWidth;
	}
	
	/* calculate the Vec_x and speed of yaw according to  write encoder value */
	
	
	
	void Kerian::InverseKinematics()
	{
		InverseCalcDynamicWidth(Expect_Vec_X,Expect_Vec_A);
		Expect_Vec_Left = Expect_Vec_X - Expect_Vec_A*DynamicWidth;
		Expect_Vec_Right = Expect_Vec_X + Expect_Vec_A*DynamicWidth;
		Expect_Encoder_Left  = (char)(Expect_Vec_Left*VELOCITY_TO_ENCODER);
		Expect_Encoder_Right = (char)(Expect_Vec_Right*VELOCITY_TO_ENCODER);
	}
	
	void Kerian::ExpectPositionEstimation()
	{
		ExpectPositiveKinematics();
		double dt,delta_x,delta_y,delta_th;
		
		Expect_CurrentTime = ros::Time::now();
		dt = (Expect_CurrentTime - Expect_LastTime).toSec();
		
		Expect_Pos_A = Optinal_Pos_A + Expect_Vec_A*dt;
		
		delta_x = Expect_Vec_X * cos(Expect_Pos_A) * dt;
		delta_y = Expect_Vec_X * sin(Expect_Pos_A) * dt;
		
		Expect_Pos_X = Optinal_Pos_X + delta_x;
		Expect_Pos_Y = Optinal_Pos_Y + delta_y;
		
		Expect_LastTime = Expect_CurrentTime;
	}
	
	void Kerian::MeasurePositionEstimation()
	{
		MeasurePositiveKinematics();
		double dt,delta_x,delta_y,delta_th;
		
		Measure_CurrentTime = ros::Time::now();
		dt = (Measure_CurrentTime - Measure_LastTime).toSec();
		
		delta_x = Measure_Vec_X * cos(Measure_Pos_A) * dt;
		delta_y = Measure_Vec_X * sin(Measure_Pos_A) * dt;
		
		Measure_Pos_X = Optinal_Pos_X + delta_x;
		Measure_Pos_Y = Optinal_Pos_Y + delta_y;
		
		Measure_LastTime = Measure_CurrentTime;
	}
	
	
	void Kerian::ComplementaryOptionalPositionEstimation()
	{
		ExpectPositionEstimation();
		MeasurePositionEstimation();
		Optinal_Pos_X = (1-ALPHA)*Expect_Pos_X + ALPHA*Measure_Pos_X;
		Optinal_Pos_Y = (1-ALPHA)*Expect_Pos_Y + ALPHA*Measure_Pos_Y;
		Optinal_Pos_A = (1-ALPHA)*Expect_Pos_A + ALPHA*Measure_Pos_A;
	}
	
	/*publish left wheel and right wheel encoder velocity */
	void Kerian::WriteSpeed()
	{
		InverseKinematics();
		char buf[5] = {0};
		char check=0;
		buf[0] = HEADER;
		buf[1] = Expect_Encoder_Left;
		buf[2] = Expect_Encoder_Right;
		buf[4] = ENDER;
		
		check ^= buf[0];
		check ^= buf[1];
		check ^= buf[2];
		check ^= buf[4];
		buf[3] = check;
		
		boost::asio::write(sp, boost::asio::buffer(buf));
	}
	
	
	bool Kerian::dataCheck(char* ptr)
	{
		int i=0;
		int check=0;
		for(i;i<7;i++)
		{
			check ^= *ptr;
			ptr++;
		}
		return check;
	}
	
	bool Kerian::spinOnce()
	{
		
		ReadSpeed();
		WriteSpeed();
		ComplementaryOptionalPositionEstimation();
		
		ros::Time currentTime;
		currentTime = ros::Time::now();
		
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = currentTime;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";
		
		geometry_msgs::Quaternion odom_quat;
		odom_quat = tf::createQuaternionMsgFromYaw(Optinal_Pos_A);
		odom_trans.transform.translation.x = Optinal_Pos_X;
		odom_trans.transform.translation.y = Optinal_Pos_Y;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = odom_quat;
		
		odom_broadcaster.sendTransform(odom_trans);
		
		nav_msgs::Odometry msgl;
		msgl.header.stamp = currentTime;
		msgl.header.frame_id = "odom";
		msgl.pose.pose.position.x = Optinal_Pos_X;
		msgl.pose.pose.position.y = Optinal_Pos_Y;
		msgl.pose.pose.position.z = 0;
		msgl.pose.pose.orientation = odom_quat;
		msgl.pose.covariance = odom_pose_covariance;
		
		msgl.child_frame_id = "base_footprint";
		msgl.twist.twist.linear.x = (1-ALPHA)*Expect_Vec_X + ALPHA*Measure_Vec_X;
		msgl.twist.twist.linear.y = 0;
		msgl.twist.twist.angular.z = (1-ALPHA)*Expect_Vec_A + ALPHA*Measure_Vec_A;
		msgl.twist.covariance = odom_twist_covariance;
		
		odomPub.publish(msgl);
	}
	
}



