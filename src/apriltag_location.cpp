#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>  
#include <mavros_msgs/PositionTarget.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/RCIn.h>

#define HIGHT	5		//初始飞行高度 m
#define HIGHT_LIT 60 //最大高度限制 m
#define vel_z 0.5 //上升速度

using namespace std;

bool marker_found = false,flag_move = false;
int  move_mode = 0,channle1 = 5, channle2 = 6,value1 = 0,value2 = 1024,value3 = 1000,value4 = 1500,value5 = 1900;
std::vector<int> current_target_id (1);
float detec_x = 0, detec_y = 0, detec_z = 0;
float init_x_take_off =0, init_y_take_off =0, init_z_take_off =0;
double angle1 = 0, cam_angle = 0;
int x_err = 0,y_err = 0;

typedef struct
{	
	float kp = 2.80;              //比例系数
	float ki = 0.0002;              //积分系数
	
	float err_I_lim = 2500;		//积分限幅值
	
	float errx_Now,errx_old_Last,errx_old_LLast;           //当前偏差,上一次偏差,上上次偏差
	float erry_Now,erry_old_Last,erry_old_LLast;
	float errax_Now,errax_old_Last,errax_old_LLast;           //当前偏差,上一次偏差,上上次偏差
	float erray_Now,erray_old_Last,erray_old_LLast;
	
	float errx_p,errx_i,errx_d;
	float erry_p,erry_i,erry_d;
	float errax_p,errax_i,errax_d;
	float erray_p,erray_i,erray_d;
	
	float CtrOutx,CtrOuty;          //控制增量输出
}PID;
PID H;

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint; // 位置速度控制消息类
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}
std_msgs::Float64 hdg;
void hdg_cb(std_msgs::Float64 msg)
{
	hdg.data = msg.data;
	//ROS_INFO("hdg:%f",hdg.data);
	if(hdg.data < 90)
	{
		angle1 = 90 - hdg.data;
	}else{
		angle1 = 450 - hdg.data;
	}
	//ROS_INFO("angle:%f",angle1);
}
mavros_msgs::RCIn rcin;
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	// for(int i = 0;i<=12;i++)
	// {
	// 	rcin.channels[i] = msg->channels[i];
	// 	ROS_INFO("channel[%d]:%d",i,rcin.channels[i]);
	// }
	if(msg->channels[channle1] > value1 - 50 && msg->channels[channle1] < value1 + 50)
	{
		flag_move = 1;
		ROS_INFO("开启手动");
	}else if(msg->channels[channle1] > value2 - 50 && msg->channels[channle1] < value2 + 50)
	{
		flag_move = 0;
		ROS_INFO("开启自动");
	}
	if(msg->channels[channle2] > value3 - 50 && msg->channels[channle2] < value3 + 50)
	{
		move_mode = 1;
	}else 	if(msg->channels[channle2] > value4 - 50 && msg->channels[channle2] < value4 + 50)
	{
		move_mode = 2;
	}else 	if(msg->channels[channle2] > value5 - 50 && msg->channels[channle2] < value5 + 50)
	{
		move_mode = 0;
	}
}
apriltag_ros::AprilTagDetection marker;
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    int count = msg->detections.size();
	if(count!=0)
	{
		for(int i = 0; i<count; i++)
		{
			marker = msg->detections[i];			
			if(marker.id == current_target_id)
			{
				marker_found = true;
				detec_x = marker.pose.pose.pose.position.x;
        		detec_y = marker.pose.pose.pose.position.y;
        		detec_z = marker.pose.pose.pose.position.z; 
			}
		}
	//ROS_INFO("detec_x=%.2f,detec_y=%.2f,detec_z=%.2f",detec_x,detec_y,detec_z);
	}
	else
	{
		marker_found = false;
	}

}

void vel_xz(float vel_x,float vel_y)
{
	double rotation_angle = (angle1 ) * (M_PI / 180.0);
    tf::Quaternion q;
	q.setRPY(0, 0, rotation_angle);

	tf::Vector3 point(vel_x,vel_y,0);
	tf::Matrix3x3 rotation_matrix(q);
	tf::Vector3 rotated_point = rotation_matrix * point;

	ROS_INFO("vel_x:%f,vel_y:%f",rotated_point.x(),rotated_point.y());
		setpoint.velocity.x = rotated_point.x();
		setpoint.velocity.y = rotated_point.y();
}
void vel_pi(float x,float y)
{
	H.errx_Now =  x;
	H.errx_p = H.errx_Now;
	H.errx_i = H.errx_Now + H.errx_i;
	H.erry_Now = y;
	H.erry_p = H.erry_Now;
	H.erry_i = H.erry_Now + H.erry_i;
	
	H.errx_old_LLast = H.errx_old_Last;
	H.errx_old_Last = H.errx_Now;
	H.erry_old_LLast = H.erry_old_Last;
	H.erry_old_Last = H.erry_Now;	

	//积分限幅
	if(H.errx_i > H.err_I_lim)	H.errx_i = H.err_I_lim;		
	if(H.errx_i < -H.err_I_lim)	H.errx_i = -H.err_I_lim;
	if(H.erry_i > H.err_I_lim)	H.erry_i = H.err_I_lim;		
	if(H.erry_i < -H.err_I_lim)	H.erry_i = -H.err_I_lim;

	H.CtrOutx = H.errx_p*H.kp + H.errx_i*H.ki ;
	H.CtrOuty = H.erry_p*H.kp + H.erry_i*H.ki ;

	vel_xz(H.CtrOutx,H.CtrOuty);
}

//读取参数模板
template<typename T>
T getParam(const std::string& name,const T& defaultValue)//This name must be namespace+parameter_name
{
    T v;
    if(ros::param::get(name,v))//get parameter by name depend on ROS.
    {
        ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }
    else 
        ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
    return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
}

int main(int argc, char *argv[])
{
	setlocale(LC_ALL, "");

	ros::init(argc,argv,"apriltag");
	ros::NodeHandle nh;
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);//订阅无人机状态话题
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);//订阅位置信息
	ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, apriltag_cb);//订阅识别信息
	ros::Subscriber hdg_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 10, hdg_cb);// 订阅磁罗盘角度
	ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc_in", 10, rcin_cb);// 订阅摇杆杆量

	ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);//控制话题,可以发布位置速度加速度同时控制
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::Rate rate(20);

	x_err = getParam<int>("cam/x_err",0);
	y_err = getParam<int>("cam/y_err",0);
   cam_angle = getParam<int>("cam/R",0);

	channle1 = getParam<int>("mode/channle1",0);
	channle2 = getParam<int>("mode/channle2",0);


	while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
	}
	setpoint.header.stamp = ros::Time::now();
	setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	setpoint.type_mask =			//使用位置控制
	//mavros_msgs::PositionTarget::IGNORE_PX |
	//mavros_msgs::PositionTarget::IGNORE_PY |
	//mavros_msgs::PositionTarget::IGNORE_PZ |
	mavros_msgs::PositionTarget::IGNORE_VX |
	mavros_msgs::PositionTarget::IGNORE_VY |
	mavros_msgs::PositionTarget::IGNORE_VZ |
	mavros_msgs::PositionTarget::IGNORE_AFX |
	mavros_msgs::PositionTarget::IGNORE_AFY |
	mavros_msgs::PositionTarget::IGNORE_AFZ |
	mavros_msgs::PositionTarget::FORCE |
	mavros_msgs::PositionTarget::IGNORE_YAW |
	mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	setpoint.position.x = 0;
	setpoint.position.y = 0;
	setpoint.position.z = 0;

	for(int i = 100; ros::ok() && i > 0; --i){
			setpoint.header.stamp = ros::Time::now();
		setpoint_pub.publish(setpoint);
			ros::spinOnce();
			rate.sleep();
		}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	//设定无人机保护模式 POSTION
	mavros_msgs::SetMode offb_setPS_mode;
	offb_setPS_mode.request.custom_mode = "POSCTL";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	
	ros::Time last_request = ros::Time::now();

	init_x_take_off = local_pos.pose.position.x;
	init_y_take_off = local_pos.pose.position.y;
	init_z_take_off = local_pos.pose.position.z;

	char mode = 't';
	int sametimes = 0;

	
	while(ros::ok())
    {
		if (current_state.mode != "OFFBOARD" &&
		(ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if (set_mode_client.call(offb_setPS_mode) &&
				offb_setPS_mode.response.mode_sent)
			{
				ROS_INFO("POSTION PROTECTED");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if (!current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if (arming_client.call(arm_cmd) &&
					arm_cmd.response.success)
				{
					ROS_INFO("UAV armed");
					init_x_take_off = local_pos.pose.position.x;
					init_y_take_off = local_pos.pose.position.y;
					init_z_take_off = local_pos.pose.position.z;
				}
				last_request = ros::Time::now();
			}
			else
			{
				switch(mode)	
				{
					case 't':
					setpoint.position.x = init_x_take_off;
					setpoint.position.y = init_y_take_off;
					setpoint.position.z = init_z_take_off + HIGHT;
					if(local_pos.pose.position.z > init_z_take_off + HIGHT - 0.2 && local_pos.pose.position.z < init_z_take_off + HIGHT + 0.2)
					{	
						if (sametimes > 10)
              			{
         		           mode = 'm';
							last_request = ros::Time::now();
                    	}
            			else sametimes++;
                		}
						else sametimes = 0;
	 	            break;
					case 'm':
							setpoint.type_mask =			//使用位置控制
							mavros_msgs::PositionTarget::IGNORE_PX |
							mavros_msgs::PositionTarget::IGNORE_PY |
							mavros_msgs::PositionTarget::IGNORE_PZ |
							//mavros_msgs::PositionTarget::IGNORE_VX |
							//mavros_msgs::PositionTarget::IGNORE_VY |
							//mavros_msgs::PositionTarget::IGNORE_VZ |
							mavros_msgs::PositionTarget::IGNORE_AFX |
							mavros_msgs::PositionTarget::IGNORE_AFY |
							mavros_msgs::PositionTarget::IGNORE_AFZ |
							mavros_msgs::PositionTarget::FORCE |
							mavros_msgs::PositionTarget::IGNORE_YAW |
							mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
							if(flag_move)	
							{
								if(move_mode == 1)
								{
									setpoint.velocity.z = vel_z;
								}
								else if(move_mode == 0)
								{
									setpoint.velocity.z = 0;
								}
								else if(move_mode == 2)
								{
									setpoint.velocity.z = -vel_z;
								}
								setpoint.velocity.x = 0;
								setpoint.velocity.y = 0;
							}
							else
							{
								if(marker_found)
								{
								
									ROS_INFO("err_x:%f,err_y:%f",detec_x ,detec_y);
									vel_pi(-detec_y * 2,-detec_x); //×2 是因为相机坐标下y比x小一倍
									setpoint.velocity.z = vel_z;
								}
								else
								{
									setpoint.velocity.x = 0;
									setpoint.velocity.y = 0;
									setpoint.velocity.z = 0;
								}
							}
							if(local_pos.pose.position.z >= init_z_take_off + HIGHT_LIT)				
							{
								setpoint.velocity.z = 0;
							}		
							if(local_pos.pose.position.z < init_z_take_off +  0.5)			
							{
								if (sametimes > 10)
								{
									mode = 'l';
									last_request = ros::Time::now();
								}
							else sametimes++;
							}
							else sametimes = 0;
							break;
					case 'l':
							offb_set_mode.request.custom_mode = "AUTO.LAND";
							if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
							{
								if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
								{
									ROS_INFO("AUTO.LAND enabled");
								}
								last_request = ros::Time::now();
							}
							break;
				}
			}
		}
		setpoint_pub.publish(setpoint);
		//ROS_INFO("marker_found=%d",marker_found);

		ros::spinOnce();
		rate.sleep();
	}

return 0;
}

