#ifndef KERIAN_H
#define KERIAN_H


#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include </opt/ros/melodic/include/tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>

namespace Robot
{
	class Kerian
	{
		public:
			Kerian();
			~Kerian();
			bool init();
			bool spinOnce();
			void SetExceptVec(double Vec_X,double Vec_A);
		
		private:
			bool ReadSpeed();
			void WriteSpeed();
			bool dataCheck(char* ptr);
			void ExpectPositiveKinematics();//正向运动学计算里程计
			void MeasurePositiveKinematics();
			void InverseKinematics();//逆向运动学计算左右轮转速
			void ExpectPositionEstimation();//期望位姿估计
			void MeasurePositionEstimation();//测量位姿估计
			void ComplementaryOptionalPositionEstimation();
			
			void CalcDynamicWidth(double vecLeft,double vecRight);//计算轮间宽度
			void InverseCalcDynamicWidth(double Vec_x,double Vec_A);
			void MeasureEncoderToVelocity();
			
		private:
			ros::Time Expect_CurrentTime,Expect_LastTime;
			ros::Time Measure_CurrentTime,Measure_LastTime;
			
			double Optinal_Pos_X;
			double Optinal_Pos_Y;
			double Optinal_Pos_A;
			
			double Expect_Pos_X;
			double Expect_Pos_Y;
			double Expect_Pos_A;
			
			double Expect_Vec_X;
			double Expect_Vec_Y;
			double Expect_Vec_A;
			
			double Expect_Vec_Left;
			double Expect_Vec_Right;
			
			char   Expect_Encoder_Left;
			char   Expect_Encoder_Right;
			
			
			double Measure_Pos_X;
			double Measure_Pos_Y;
			double Measure_Pos_A;
			double Measure_Pos_Last_A;
			
			double Measure_Vec_X;
			double Measure_Vec_A;
			
			double Measure_Vec_Left;
			double Measure_Vec_Right;
			
			char   Measure_Encoder_Left;
			char   Measure_Encoder_Right;
			
			
			/*
				  
				下发机器人期望速度 
						Expect_Vec_X
						Expect_Vec_A
				下发左右轮期望速度
						Expect_Vec_Left
						Expect_Vec_Right
				下发左右轮期望增量编码值
						Expect_Encoder_Left
						Expect_Encoder_Right
				
				测量左右轮编码值
						Measure_Encoder_Left
						Measure_Encoder_Right
				测量左右轮速度
						Measure_Vec_Left
						Measure_Vec_Right
				测量机器人速度
						Measure_Vec_X
						Measure_Vec_A
				世界坐标系机器人期望位置
						Expect_Pos_X
						Expect_Pos_Y
						Expect_Pos_A
				世界坐标系机器人测量位置
						Measure_Pos_X
						Measure_Pos_Y
						Measure_Pos_A
			*/
			
		double DynamicWidth;
			
		ros::NodeHandle nh;
		ros::Publisher odomPub;//publish the odometry messages
		//tf::TransformBroadcaster odom_broadcaster_;
		tf2_ros::TransformBroadcaster odom_broadcaster; //broadcast the tf messages
	};
}

#endif
