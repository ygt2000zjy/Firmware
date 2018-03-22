// 头文件先按照旋翼的加
#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

extern "C" __EXPORT int xhy_att_controlp_main(int argc, char *argv[]);

#define MAX_GYRO_COUNT 3


class  XhyAttitudeControl
{
public:
	//构造函数
	XhyAttitudeControl();

	//析构函数
	~XhyAttitudeControl();

	//定义任务启动函数
	int start();

private:
	bool	_task_should_exit;  //如果为真，则任务已经存在
	int		_control_task;       //任务句柄

	int		_v_att_sub;          //飞行器当前姿态订阅
	int 	_v_att_sp_sub;       //飞行器姿态设定点订阅
	int 	_v_rates_sp_sub;     //飞行器速率设定点订阅
	int 	_v_control_mode_sub;  //飞行器控制模式订阅
	int 	_params_sub;          //参数更新订阅
  int 	_manual_control_sp_sub; //手动控制设定点订阅
  int 	_vehicle_status_sub;   //飞行器状态订阅
  int 	_motor_limit_sub;     //电机限制订阅，这个好像没啥用
  int 	_battery_status_sub;  //电池状态订阅
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	//陀螺仪数据订阅
	int		_sensor_correction_sub;	//传感器温度补偿订阅
	int		_sensor_bias_sub;	//传感器运行时的有偏补偿

	unsigned _gyro_count;
	int _selected_gyro;

	orb_advert_t	 _v_rates_sp_pub;  //速率设定点发布
	orb_advert_t	 _actuators_0_pub; //姿态作动器控制发布
	orb_advert_t	 _controller_status_pub; //控制器状态发布

	orb_id_t _rates_sp_id;   //主要用于区分纯起飞还是起飞加转换
	orb_id_t _actuators_id;		//主要用于区分纯起飞还是起飞加转换

	//bool 	_actuators_0_circuit_breaker_enable;    //这个好像没啥用，先注释了吧

	struct vehicle_attitude_s 		_v_att; //飞行器姿态结构体
	struct vehicle_attitude_setpoint_s 		_v_att_sp; //飞行器姿态设定点结构体
	struct vehicle_rates_setpoint_s 	_v_rates_sp;   //飞行器速率结构体
	struct manual_control_setpoint_s	_manual_control_sp; //手动控制设定点结构体
	struct vehicle_control_mode_s	_v_control_mode;  //飞行器控制模式结构体
	struct actuator_controls_s 		_actuators;  //作动器控制结构体
	struct vehicle_status_s 	_vehicle_status;  //飞行器状态结构体
	struct battery_status_s     _battery_status;  //电池状态结构体
	struct sensor_gyro_s       _sensor_gyro;   //温度以及偏差补偿前的陀螺仪数据
	struct sensor_correction_s 		_sensor_correction; //传感器温度补偿更正
	struct sensor_bias_s 	_sensor_bias;  //传感器运行时的偏差更正

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t _loop_perf;
	perf_counter_t _controller_latency_perf;

	static constexpr const float initial_update_rate_hz = 250.f; //用于初始化的回路更新速率，这个好像也没啥用
 	float _loop_update_rate_hz;


    math::Vector<3>		_rates_prev; //上一步的角速率
    math::Vector<3>		_rates_prev_filtered; //上一步经过滤波后的角速率，我要不要滤波
    math::Vector<3>		_rates_sp; //角速率设定点
    float		_thrust_sp;  //油门设定点
    math::Vector<3>		_att_control;  //姿态控制向量

    math::Matrix<3,3> 	_I;    //单位矩阵
    math::Matrix<3, 3>	_board_rotation = {};	//飞控板朝向的旋转矩阵

    //下面定义的这几个是L1自适应控制器里面的几个东西，定义在私有变量里面，那应该在类的私有函数中可以任意调用
	 /* math::Vector<3> itahat(0.0f, 0.0f, 0.0f);
		math::Vector<3> Madaptive(0.0f, 0.0f, 0.0f);
		math::Vector<3> Madaptive_prev(0.0f,0.0f,0.0f);
		math::Vector<3> omegahat_prev(0.0f,0.0f,0.0f);
		math::Vector<3> Mbase(0.0f,0.0f,0.0f);*/

		math::Vector<3> itahat={};
		math::Vector<3> itahat_prev={};
		math::Vector<3> Madaptive={};
		math::Vector<3> Madaptive_prev={};
		math::Vector<3> omegahat_prev={};
		math::Vector<3> Mbase={};

    struct {
    	param_t 	rattitude_thres;
    	param_t  	Am1_outer;
    	param_t  	Am2_outer;
    	param_t 	Ixx;
    	param_t 	Iyy;
    	param_t 	Izz;
			param_t 	kg1;
			param_t		kg2;
			param_t		kg3;
    	param_t 	adaptive_gain;
    	param_t 	lowpassfilter_param;
    	//与控制分配相关的系数
    	param_t 	control_allocate1;
    	param_t 	control_allocate2;
    	param_t 	control_allocate3;
    	param_t 	control_allocate4;
    	param_t 	control_allocate5;
    	param_t 	control_allocate6;
    	param_t 	control_allocate7;
    	param_t 	control_allocate8;

    	param_t 	board_rotation;
    	param_t  	board_offset[3];


    }		_params_handles;


	struct {
		float rattitude_thres;
		float Am1_outer;
		float Am2_outer;
		float Ixx;
		float Iyy;
		float Izz;
		float kg1;
		float kg2;
		float kg3;
		float adaptive_gain;
		float lowpassfilter_param;
		//与控制分配相关的系数
		float control_allocate1;
		float control_allocate2;
		float control_allocate3;
		float control_allocate4;
		float control_allocate5;
		float control_allocate6;
		float control_allocate7;
		float control_allocate8;

		int32_t board_rotation;

		float board_offset[3];
	}		_params;

	void parameters_update();  //更新本地参数缓存

	//检查更新并进行处理
	void 	battery_status_poll();
	void	parameter_update_poll();
	void 	sensor_bias_poll();
	void 	sensor_correction_poll();
	void 	vehicle_attitude_poll();
	void 	vehicle_attitude_setpoint_poll();
	void 	vehicle_control_mode_poll();
	void	vehicle_manual_poll();
	void 	vehicle_rates_setpoint_poll();
	void 	vehicle_status_poll();
	void 	vehicle_motor_limits_poll();

	//基准控制器
	void 	baseline_controller(float dt);

	//自适应控制器
	void 	adaptive_control(float dt);

	//控制分配
  void control_allocation();

	static void task_main_trampoline(int argc, char *argv[]);

	void task_main();
};

namespace xhy_att_control
{
	XhyAttitudeControl *g_control; //这个是相当于从堆中进行实例化，最后需要自己手动释放内存
}

XhyAttitudeControl::XhyAttitudeControl():

	_task_should_exit(false),
	_control_task(-1),

	//订阅相关
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_vehicle_status_sub(-1),
	_motor_limit_sub(-1),
	_battery_status_sub(-1),
	_sensor_correction_sub(-1),
	_sensor_bias_sub(-1),

	//陀螺仪选择
	_gyro_count(1),
	_selected_gyro(0),

	//发布相关
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(nullptr),
	_actuators_id(nullptr),

	//_actuators_0_circuit_breaker_enable(false),

	_v_att{},
	_v_att_sp{},
	_v_rates_sp{},
	_manual_control_sp{},
	_v_control_mode{},
	_actuators{},
	_vehicle_status{},
	_battery_status{},
	_sensor_gyro{},
	_sensor_correction{},
	_sensor_bias{},
	_saturation_status{},
   //运行情况计数
	_loop_perf(perf_alloc(PC_ELAPSED, "xhy_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),

 	//这里省略了几个低通滤波器，后续主要看看角速率那边还要不要进行滤波，ins

	_loop_update_rate_hz(initial_update_rate_hz) //这个有关回路速率的貌似最后也没用上
	{
		for (uint8_t i = 0;i < MAX_GYRO_COUNT; i++){
			_sensor_gyro_sub[i] = -1;
		}

		_vehicle_status.is_rotary_wing = true;

		_params.rattitude_thres = 1.0f;
		_params.Am1_outer = 3.0f;
		_params.Am2_outer = 10.0f;
		_params.Ixx = 0.025f;
		_params.Iyy = 0.007f;
		_params.Izz = 0.022f;
		_params.kg1 = 0.2f;
		_params.kg2 = 0.04f;
		_params.kg3 = 0.176f;

		_params.adaptive_gain = -200.0f;
		_params.lowpassfilter_param = 30.0f;
		//尤其注意这里参数的初值，要不要重新给
		_params.control_allocate1 = 0.0f;
		_params.control_allocate2 = 0.0f;
		_params.control_allocate3 = 0.0f;
		_params.control_allocate4 = 0.0f;
		_params.control_allocate5 = 0.0f;
		_params.control_allocate6 = 0.0f;
		_params.control_allocate7 = 0.0f;
		_params.control_allocate8 = 0.0f;

		_params.board_rotation = 0;

		_params.board_offset[0] = 0.0f;
		_params.board_offset[1] = 0.0f;
		_params.board_offset[2] = 0.0f;


		_rates_prev.zero();
		_rates_prev_filtered.zero();
		_rates_sp.zero();
		_thrust_sp = 0.0f;
		_att_control.zero();


		_I.identity();
		_board_rotation.identity();

		_params_handles.rattitude_thres		=	param_find("XHY_RATT_TH");
		_params_handles.Am1_outer		=	param_find("AM1_OUTER");
		_params_handles.Am2_outer		=	param_find("AM2_OUTER");
		_params_handles.Ixx		=	param_find("IXX");
		_params_handles.Iyy		=	param_find("IYY");
		_params_handles.Izz		=	param_find("IZZ");
		_params_handles.kg1 	=	param_find("KG1");
		_params_handles.kg2 	=	param_find("KG2");
		_params_handles.kg3 	=	param_find("KG3");

		_params_handles.adaptive_gain 	=	param_find("ADAPTIVE_GAIN");
		_params_handles.lowpassfilter_param 	=	param_find("LOWPADDFILTER");
		_params_handles.control_allocate1 	=	param_find("CONTROL_AL1");
		_params_handles.control_allocate2 	=	param_find("CONTROL_AL2");
		_params_handles.control_allocate3 	=	param_find("CONTROL_AL3");
		_params_handles.control_allocate4 	=	param_find("CONTROL_AL4");
		_params_handles.control_allocate5 	=	param_find("CONTROL_AL5");
		_params_handles.control_allocate6 	=	param_find("CONTROL_AL6");
		_params_handles.control_allocate7 	=	param_find("CONTROL_AL7");
		_params_handles.control_allocate8 	=	param_find("CONTROL_AL8");

		/* rotations */
		_params_handles.board_rotation		=	param_find("SENS_BOARD_ROT");

		/* rotation offsets */
		_params_handles.board_offset[0]		=	param_find("SENS_BOARD_X_OFF");
		_params_handles.board_offset[1]		=	param_find("SENS_BOARD_Y_OFF");
		_params_handles.board_offset[2]		=	param_find("SENS_BOARD_Z_OFF");

		//取回初始参数值
		parameters_update();
	}

	XhyAttitudeControl::~XhyAttitudeControl()
	{
		if (_control_task != -1){
			//任务每100ms唤醒，或更长
			_task_should_exit = true;

			//等待1s以便完成任务退出请求
			unsigned i = 0;

			do{
				usleep(20000);

				if(++i>50){
					px4_task_delete(_control_task);
					break;
				}
			}while(_control_task != -1);
		}
		xhy_att_control::g_control = nullptr;
	}

	void
	XhyAttitudeControl::parameters_update()
	{
		param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);
		param_get(_params_handles.Am1_outer, &_params.Am1_outer);
		param_get(_params_handles.Am2_outer, &_params.Am2_outer);
		param_get(_params_handles.Ixx, &_params.Ixx);
		param_get(_params_handles.Iyy, &_params.Iyy);
		param_get(_params_handles.Izz, &_params.Izz);
		param_get(_params_handles.kg1, &_params.kg1);
		param_get(_params_handles.kg2, &_params.kg2);
		param_get(_params_handles.kg3, &_params.kg3);
		param_get(_params_handles.adaptive_gain, &_params.adaptive_gain);
		param_get(_params_handles.lowpassfilter_param, &_params.lowpassfilter_param);
		param_get(_params_handles.control_allocate1, &_params.control_allocate1);
		param_get(_params_handles.control_allocate2, &_params.control_allocate2);
		param_get(_params_handles.control_allocate3, &_params.control_allocate3);
		param_get(_params_handles.control_allocate4, &_params.control_allocate4);
		param_get(_params_handles.control_allocate5, &_params.control_allocate5);
		param_get(_params_handles.control_allocate6, &_params.control_allocate6);
		param_get(_params_handles.control_allocate7, &_params.control_allocate7);
		param_get(_params_handles.control_allocate8, &_params.control_allocate8);


		/* stick deflection needed in rattitude mode to control rates not angles */
		param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

		//飞控相对于机体的旋转
		param_get(_params_handles.board_rotation, &(_params.board_rotation));

		// fine adjustment of the rotation，这其中的旋转角度应该是别的地方传递过来的
		param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
		param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
		param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

		/* get transformation matrix from sensor/board to body frame */
		get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

		/* fine tune the rotation */
		math::Matrix<3, 3> board_rotation_offset;
		board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);
		_board_rotation = board_rotation_offset * _board_rotation;

	}

void
XhyAttitudeControl::parameter_update_poll()
{
	bool updated;

	//检测参数是否改变了
	orb_check(_params_sub,&updated);

	if(updated){
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update),_params_sub,&param_update);
		parameters_update();
	}
}

void
XhyAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	//检查是否飞行器的控制模型改变了
	orb_check(_v_control_mode_sub, &updated);

	if (updated){
		orb_copy(ORB_ID(vehicle_control_mode),_v_control_mode_sub, &_v_control_mode);
	}
}

void
XhyAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	//检查是否有杆量输入
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
XhyAttitudeControl::vehicle_attitude_setpoint_poll()
{
	//检查是否有新的设定点
	bool updated;
	orb_check(_v_att_sp_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_attitude_setpoint),_v_att_sp_sub,&_v_att_sp);
	}
}

void
XhyAttitudeControl::vehicle_rates_setpoint_poll()
{
	//检查是否有新的设定点更新
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated)
	{
		orb_copy(ORB_ID(vehicle_rates_setpoint),_v_rates_sp_sub, &_v_rates_sp);
	}

}

void
XhyAttitudeControl::vehicle_status_poll()
{
 	//检测是否有新的状态信息
 	bool vehicle_status_updated;
 	orb_check(_vehicle_status_sub, &vehicle_status_updated);

 	if (vehicle_status_updated){
 		orb_copy(ORB_ID(vehicle_status),_vehicle_status_sub, &_vehicle_status);
 	}
 	if(_rates_sp_id == nullptr){
 		_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
 		_actuators_id = ORB_ID(actuator_controls_0);
 	}
}

void
XhyAttitudeControl::vehicle_motor_limits_poll()
{
}
void
XhyAttitudeControl::battery_status_poll()
{
	//检测是够有电池状态的更新
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(battery_status),_battery_status_sub, &_battery_status);
	}
}

void
XhyAttitudeControl::vehicle_attitude_poll()
{
	//检测是否有新信息
	bool updated;
	orb_check(_v_att_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

void
XhyAttitudeControl::sensor_correction_poll()
{
	//检测是否有新信息
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(sensor_correction),_sensor_correction_sub, &_sensor_correction);
	}
	//更新最近的陀螺仪选择
	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

void
XhyAttitudeControl::sensor_bias_poll()
{
	//检测是否有新信息
	bool updated;
	orb_check(_sensor_bias_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(sensor_bias),_sensor_bias_sub, &_sensor_bias);
	}
}


void
XhyAttitudeControl::baseline_controller(float dt)
{
	//得到非线性动态逆的基准期望力矩，同时得到角速率期望值
	vehicle_attitude_setpoint_poll(); //进行姿态设定点的更新
	vehicle_attitude_poll();  //进行当前飞行器姿态的更新,但是当前姿态只提供了四元数的选择，那么只能通过将四元数转为欧拉角进行获取

	//首先从vehicle_attitude_setpoint主题中，获取欧拉角的期望值，这假设是在外环存在的情况下搞的
	float roll_sp = _v_att_sp.roll_body;
	float pitch_sp = _v_att_sp.pitch_body;
	float yaw_sp = _v_att_sp.yaw_body;
	_thrust_sp = _v_att_sp.thrust;

	//得到期望的三轴欧拉角
	math::Vector<3> Eulerdesire(roll_sp, pitch_sp, yaw_sp);
	/*
	Eulerdesire(0) = roll_sp;
	Eulerdesire(1) = pitch_sp;
	Eulerdesire(2) = yaw_sp;
	 */
	//从四元数获取欧拉角
	math::Matrix<3,3> temp_R;
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	temp_R = q_att.to_dcm();
	math::Vector<3> euler_angles;
	euler_angles = temp_R.to_euler();
	float roll    = euler_angles(0);
	float pitch   = euler_angles(1);
	float yaw     = euler_angles(2);

	//得到三轴欧拉角
	math::Vector<3> Euler(roll, pitch, yaw);
	/*
	Euler(0) = roll;
	Euler(1) = pitch;
	Euler(2) = yaw;
	 */

	math::Matrix<3,3> R;
	R(0,0) = 1;
	R(0,1) = sin(roll)*tan(pitch);
	R(0,2) = cos(roll)*tan(pitch);
	R(1,0) = 0;
	R(1,1) = cos(roll);
	R(1,2) = -sin(roll);
	R(2,0) = 0;
	R(2,1) = sin(roll)/cos(pitch);
	R(2,2) = cos(roll)/cos(pitch);

	math::Matrix<3,3> invR;
	invR(0,0) = 1;
	invR(0,1) = 0;
	invR(0,2) = -sin(pitch);
	invR(1,0) = 0;
	invR(1,1) = cos(roll);
	invR(1,2) = sin(roll)*cos(pitch);
	invR(2,0) = 0;
	invR(2,1) = -sin(roll);
	invR(2,2) =cos(roll)*cos(pitch);

	math::Matrix<3,3> Am1;
	math::Matrix<3,3> Am2;
	Am1(0,0) =_params.Am1_outer;
	Am1(0,1) =0;
	Am1(0,2) =0;
	Am1(1,0) =0;
	Am1(1,1) =_params.Am1_outer;
	Am1(1,2) =0;
	Am1(2,0) =0;
	Am1(2,1) =0;
	Am1(2,2) =_params.Am1_outer;

	Am2(0,0) =_params.Am2_outer;
	Am2(0,1) =0;
	Am2(0,2) =0;
	Am2(1,0) =0;
	Am2(1,1) =_params.Am2_outer;
	Am2(1,2) =0;
	Am2(2,0) =0;
	Am2(2,1) =0;
	Am2(2,2) =_params.Am2_outer;

	math::Matrix<3,3> J;
	J(0,0) = _params.Ixx;
	J(0,1) = 0;
	J(0,2) = 0;
	J(1,0) = 0;
	J(1,1) = _params.Iyy;
	J(1,2) = 0;
	J(2,0) = 0;
	J(2,1) = 0;
	J(2,2) = _params.Izz;


	/*math::Vector<3> rates;
	//进行真实角速率的获取
	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}
	rates = _board_rotation * rates;
	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

	math::Vector<3> omega(rates(0),rates(1),rates(2));*/

	//得到真实的三轴角速率向量,先按照固定翼的思路整
	math::Vector<3> omega;
	omega(0) = _v_att.rollspeed;
	omega(1) =  _v_att.pitchspeed;
	omega(2) = _v_att.yawspeed;

	Mbase = J * Am2 * (invR * (Am1 * Eulerdesire-Euler) - omega) - omega % (J * omega);
	_rates_sp = invR * (Am1 * (Eulerdesire - Euler));
//到此完成了基准控制器的设计
}

//这里进行自适应控制器的设计
void
XhyAttitudeControl::adaptive_control(float dt)
{
	math::Matrix<3,3> h;
	h(0,0) = dt;
	h(0,1) = 0;
	h(0,2) = 0;
	h(1,0) = 0;
	h(1,1) = dt;
	h(1,2) = 0;
	h(2,0) = 0;
	h(2,1) = 0;
	h(2,2) = dt;

	math::Vector<3> omega;
	omega(0) = _v_att.rollspeed;
	omega(1) =  _v_att.pitchspeed;
	omega(2) = _v_att.yawspeed;

	math::Vector<3> omegadesire;
	omegadesire = _rates_sp;

	math::Matrix<3,3> 	invJ;
	invJ(0,0) = 1/_params.Ixx;
	invJ(0,1) = 0;
	invJ(0,2) = 0;
	invJ(1,0) = 0;
	invJ(1,1) = 1/_params.Iyy;
	invJ(1,2) = 0;
	invJ(2,0) = 0;
	invJ(2,1) = 0;
	invJ(2,2) = 1/_params.Izz;

	//这里使用欧拉法进行观测器的离散化
	math::Vector<3> omegahat;

	math::Matrix<3,3> Am2;
	Am2(0,0) =_params.Am2_outer;
	Am2(0,1) =0;
	Am2(0,2) =0;
	Am2(1,0) =0;
	Am2(1,1) =_params.Am2_outer;
	Am2(1,2) =0;
	Am2(2,0) =0;
	Am2(2,1) =0;
	Am2(2,2) =_params.Am2_outer;

	//状态观测器
	omegahat = (Am2 * h + _I)* omegahat_prev + h * itahat_prev + h*invJ * Madaptive_prev; //这里面几个变量的时序一定要搞清
	//从上面的这个来看的话，itahat应该用的是旧的, Madaptive应该用的也是旧的,按照simulink的仿真来看，这两个变量的第一步都是0
	//像这种变量是不是都得写在外面啊我日

	//自适应率
	itahat = (omegahat - omega) * _params.adaptive_gain;

	//控制器
	math::Vector<3> itahat_temp;
	//下面的这三个实数是用与A，B矩阵有关的结果
	itahat_temp(0) = itahat(0) * _params.Ixx;
	itahat_temp(1) = itahat(1) * _params.Iyy;
	itahat_temp(2) = itahat(2) * _params.Izz;

	math::Matrix<3,3> kg;
	kg(0,0) = _params.kg1;
	kg(0,1) = 0;
	kg(0,2) = 0;
	kg(1,0) = 0;
	kg(1,1) = _params.kg2;
	kg(1,2) = 0;
	kg(2,0) = 0;
	kg(2,1) = 0;
	kg(2,2) = _params.kg3;

	math::Matrix<3,3> TEMP1;
	float temp1;
	temp1 = 1+ _params.lowpassfilter_param * dt;
	TEMP1(0,0)=temp1;
	TEMP1(0,1)=0;
	TEMP1(0,2)=0;
	TEMP1(1,0)=0;
	TEMP1(1,1)=temp1;
	TEMP1(1,2)=0;
	TEMP1(2,0)=0;
	TEMP1(2,1)=0;
	TEMP1(2,2)=temp1;

	math::Matrix<3,3> TEMP2;
	float temp2;
	temp2 = _params.lowpassfilter_param * dt;
	TEMP2(0,0)=temp2;
	TEMP2(0,1)=0;
	TEMP2(0,2)=0;
	TEMP2(1,0)=0;
	TEMP2(1,1)=temp2;
	TEMP2(1,2)=0;
	TEMP2(2,0)=0;
	TEMP2(2,1)=0;
	TEMP2(2,2)=temp2;


	//Madaptive = 1/(1+ _params.lowpassfilter_param * dt) * (Madaptive_prev - _params.lowpassfilter_param * dt* itahat_temp + _params.lowpassfilter_param * dt* kg * omegadesire);
 	Madaptive = TEMP1 * Madaptive_prev + TEMP2 * (itahat_temp - kg* omegadesire);
 	//上面算出来的这个结果怎么传出去，是通过之前定义的类的私有变量进行传递吗
	//这里最后转换为-1到1之间的数，然后发布出去，看看怎么整这个事
	omegahat_prev = omegahat;
	Madaptive_prev = Madaptive;
	itahat_prev = itahat;
}
void
XhyAttitudeControl::control_allocation()
{
   //
}

void
XhyAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	xhy_att_control::g_control->task_main();
}

void
XhyAttitudeControl::task_main()
{
	//进行一系列的订阅
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));      //飞行器当前姿态
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));   //飞行器姿态设定点
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));    //飞行器角速率设定点
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));  //飞行器控制模式
	_params_sub = orb_subscribe(ORB_ID(parameter_update));     //参数更新
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));  //手动控制设定点
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));     //飞行器状态
	//_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));  //多旋翼电机限制
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));   //电池状态

	//_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT); //找到陀螺仪的个数

	/*if （_gyro_count == 0）{
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}*/

	//_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));  //温度修正系数
	//_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));  //运行中的传感器有偏修正系数

	//强制进行一波参数更新，主要给参数赋予初值
	parameters_update();

	//唤醒源，通过传感器app选择传感器获取陀螺仪数据
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime last_run = task_start;

	while(!_task_should_exit){
		poll_fds.fd = _v_att_sub;  //这里检测飞行器姿态的句柄

		//为了获取数据，最多等待100ms
		int pret = px4_poll(&poll_fds,1,100);

		if (pret == 0){
			continue;
		}

		if(pret < 0){
			warn("xhy att ctrl poll error %d, %d", pret, errno);
			usleep(100000);
			continue;
		}
		perf_begin(_loop_perf);

		//防止dt出现太小(<2ms)或太大(>20ms)
		if (poll_fds.revents & POLLIN){
			const hrt_abstime now = hrt_absolute_time();
			float dt = (now -last_run) / 1e6f;
			last_run = now;

			if(dt < 0.002f){
				dt = 0.002f;
			}else if (dt > 0.02f){
				dt = 0.02f;
			}

			//copy陀螺仪数据
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			//检查其他主题是否有更新
			parameter_update_poll();  //是否有参数的修改
			vehicle_control_mode_poll();  //飞行器的控制模式是否更新
			vehicle_manual_poll();    //手动控制的杆量是否有更新
			vehicle_status_poll();    //飞行器状态是否有更新
			vehicle_motor_limits_poll();  //电机的限制是否有更新
			battery_status_poll();    //电池的状态是否有更新
			vehicle_attitude_poll();  //飞行器的姿态是否有更新
			sensor_correction_poll();  //传感器的修正是否有更新
			sensor_bias_poll();   //传感器运行时的偏移是否有更新

   				/*检查是否在rattitude模式(角速度控制)，如果飞手的输入超过了设定的阈值，则将其转换成横滚、俯仰、偏航角速度命令传送给自驾仪；
				如果输入没有超过阈值，则将其转换成横滚、俯仰转角度 以及偏航角速度 命令。油门直接输出到混控器。*/
			if(_v_control_mode.flag_control_rattitude_enabled){
				if(fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				   fabsf(_manual_control_sp.x) > _params.rattitude_thres){
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if(_v_control_mode.flag_control_attitude_enabled){

				baseline_controller(dt);
			//发布角速率的设定点
			_v_rates_sp.roll = _rates_sp(0);
			_v_rates_sp.pitch = _rates_sp(1);
			_v_rates_sp.yaw = _rates_sp(2);
			_v_rates_sp.thrust = _thrust_sp;
			_v_rates_sp.timestamp = hrt_absolute_time();

			if (_v_rates_sp_pub != nullptr){
				orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);
			}else if (_rates_sp_id){
				_v_rates_sp_pub = orb_advertise(_rates_sp_id,&_v_rates_sp);
			}
			}else{
				// 没有启用高度控制器，进行角速率设定点的更新

			/* 手动角速率控制 - ACRO 模式,为了程序简洁，先不写 */
			}

			if (_v_control_mode.flag_control_rates_enabled){
				adaptive_control(dt);
				//接下来要发布作动器的控制


				//代码里面还加了一个电池的补偿环节，但是补偿因子是0，感觉默认的是没有起作用

				//下面这一小段主要关心_actuators_id这个玩意，一定要搞对了！！！！！！！！
				if (_actuators_0_pub != nullptr) {
						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}
			}
		}
		perf_end(_loop_perf);

	} //这个对应了上面的_task_should_exsit
	_control_task = -1;
}  //对应了整个task_main()这个函数

int
XhyAttitudeControl::start()
{
	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd("xhy_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL,
					   1700,
					   (px4_main_t)&XhyAttitudeControl::task_main_trampoline,
					   nullptr);
	if(_control_task < 0){
		warn("task start failed");
		return -errno;
	}
	return OK;
}
int xhy_att_controlp_main(int argc, char *argv[])
{
	if (argc < 2){
		warnx("usage: xhy_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (xhy_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		xhy_att_control::g_control = new XhyAttitudeControl;

		if (xhy_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != xhy_att_control::g_control->start()) {
			delete xhy_att_control::g_control;
			xhy_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (xhy_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete xhy_att_control::g_control;
		xhy_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (xhy_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
