#include <mainpp.h>
#include <stdio.h>
#include "gy85.h"
#include "motor.h"
#include "encoder.h"
#include "PID.h"
#include "Kinematics.h"

#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

double required_angular_vel = 1.0;
double required_linear_vel = 0.5;
uint32_t previous_command_time = 0;

bool is_first = true;
bool accel, gyro, mag;

/*PID define start*/
PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);
/*PID define end*/

Motor motor1(MOTOR1, 254, 575);
Motor motor2(MOTOR2, 254, 575);

Encoder encoder1(ENCODER1, 0xffff, 0, COUNTS_PER_REV);
Encoder encoder2(ENCODER2, 0xffff, 0, COUNTS_PER_REV);

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
Gy85 imu;

void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;

riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

void pid_callback(const riki_msgs::PID& pid){
	motor1_pid.updateConstants(pid.p, pid.i, pid.d);
  motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
    required_linear_vel = cmd_msg.linear.x;
    required_angular_vel = cmd_msg.angular.z;

    previous_command_time = nh.getHardware()->time();
}

void move_base(){
	Kinematics::rpm req_rpm = kinematics.getRPM(required_linear_vel, 0, required_angular_vel);

	int current_rpm1 = encoder1.getRPM();
	int current_rpm2 = encoder2.getRPM();

	motor1.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
	motor2.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));

	Kinematics::velocities current_vel;
	current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
	
	 //fill in the object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = 0.0;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg object to ROS
    raw_vel_pub.publish(&raw_vel_msg);
} 

void check_imu(){
	gyro = imu.check_gyroscope();
	accel = imu.check_accelerometer();
	mag = imu.check_magnetometer();

	if (!accel){
		nh.logerror("Accelerometer NOT FOUND!");
	}   

	if (!gyro){
		nh.logerror("Gyroscope NOT FOUND!");
	}   

	if (!mag){
		nh.logerror("Magnetometer NOT FOUND!");
	}
	is_first = false;
}

void publish_imu()
{
	//geometry_msgs::Vector3 acceler, gyro, mag;
	//this function publishes raw IMU reading
	//measure accelerometer
	if (accel){
		imu.measure_acceleration();
		raw_imu_msg.linear_acceleration = imu.raw_acceleration;
	}

	//measure gyroscope
	if (gyro){
		imu.measure_gyroscope();
		raw_imu_msg.angular_velocity = imu.raw_rotation;
	}

    //measure magnetometer
	if (mag){
		imu.measure_magnetometer();
		raw_imu_msg.magnetic_field = imu.raw_magnetic_field;
	}

	//publish raw_imu_msg object to ROS
	raw_imu_pub.publish(&raw_imu_msg);
}

void stop_base()
{
    required_linear_vel = 0;
    required_angular_vel = 0;
}

void print_debug()
{
    char buffer[50]; 
    sprintf (buffer, "Encoder Left: %ld", encoder1.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder Right: %ld", encoder2.read());
    nh.loginfo(buffer);
    //sprintf (buffer, "get line speed : %f, pwm: %d", required_linear_vel, pwm);
    //nh->loginfo(buffer);
}
/*
std_msgs::String stm32_to_pc_word;

ros::Subscriber<std_msgs::String> cmd_sub("pc_to_stm32", command_callback);
ros::Publisher stm32_to_pc("stm32_to_pc", &stm32_to_pc_word);

void command_callback( const std_msgs::String& rxbuff)
{
    stm32_to_pc_word = rxbuff;
		stm32_to_pc.publish(&stm32_to_pc_word);
}
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
	motor1.init();
	motor2.init();
	
	encoder1.init();
	encoder2.init();
	imu.init();
  	nh.initNode();
	nh.advertise(raw_vel_pub);
	nh.advertise(raw_imu_pub);
	nh.subscribe(pid_sub);
	nh.subscribe(cmd_sub);

	while (!nh.connected()){
		nh.spinOnce();
	}   
	nh.loginfo("GuideBase Connected!");
}

void loop(void)
{  
	//uint32_t previous_battery_debug_time = 0;
	static uint32_t previous_debug_time = 0;
	static uint32_t previous_imu_time = 0;
	static uint32_t previous_control_time = 0;
	static uint32_t publish_vel_time = 0;
	if ((nh.getHardware()->time() - previous_control_time) >= (1000 / COMMAND_RATE)){
		move_base();
		previous_control_time = nh.getHardware()->time() ;
	}

	if ((nh.getHardware()->time() - previous_command_time) >= 400){
		stop_base();
	}

	if ((nh.getHardware()->time()  - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
		if(is_first){
			//sanity check if the IMU exits
			check_imu();
		} else{
			//publish the IMU data
			publish_imu();
		}
		previous_imu_time = nh.getHardware()->time() ;
	}
	/*
	if( (millis() - previous_battery_debug_time) >= (1000 / BAT_PUBLISH_RATE)){
		if(bat.get_volt() < 11.300000){
			OnOff = !OnOff;
			led.on_off(OnOff);
			nh.logwarn(battery_buffer);			
		}
		publishBAT();
		previous_battery_debug_time = millis();		
	}
	*/
	if(DEBUG){
		if ((nh.getHardware()->time()  - previous_debug_time) >= (1000 / DEBUG_RATE)) {
			print_debug();
			previous_debug_time = nh.getHardware()->time() ;
		}
	}
	nh.spinOnce();
}

