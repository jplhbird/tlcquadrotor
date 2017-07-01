// TLC.h: interface for the TLC class.
//
//////////////////////////////////////////////////////////////////////

#ifndef TLC_H
#define TLC_H


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <Eigen/Dense>


#define T_SAMPLE 20

// #define Ixx 0.445f   //
// #define Iyy 0.4465f
// #define Izz 0.4457f
// #define Ixz 0
// 
// #define D 1/(Ixz*Ixz-Ixx*Izz)
// #define Ippq (Ixz*(Iyy-Izz-Ixx))*D
// #define Irqr (0-Ippq)
// #define Ipqr (Izz*Izz-Iyy*Izz+Izz*Izz)*D
// #define Iqpp (0-Ixz/Iyy)
// #define Iqrr (0-Iqpp)
// #define Iqpr (Izz-Ixx)/Iyy
// #define Irpq (Ixx*Iyy-Ixz*Ixz-Ixx*Ixx)*D
// #define gpl (0-Izz*D)
// #define gpn (0-Ixz*D)
// #define grl gpn
// #define gqm 1/Iyy
// #define grn (0-Ixx*D)

// 

// #define D_A 0.00627507
// #define D_B 0.00627507
// #define D_C 0.005143 //0.00013
// #define D_D  0.01143 //0.01143

// #define G 33 


class TLC  
{
public:		
	TLC();
	virtual ~TLC();
	float T_sampling;

        float Ixx;   //inertial parameters
        float Iyy;
        float Izz;
        float Ixz;

        float D;
        float Ippq;
        float Irqr;
        float Ipqr;
        float Iqpp;
        float Iqrr;
        float Iqpr;
        float Irpq;
        float gpl;
        float gpn;
        float grl;
        float gqm;
        float grn;

        float D_A;
        float D_B;
        float D_C; //0.00013
        float D_D; //0.01143

        float G;

        float G_revise_x;
        float h_revise_x;
        float G_revise_y;
        float h_revise_y;

        float ksi_roll_out;
        float ksi_pitch_out;
        float ksi_yaw_out;
        float omega_roll_out;
        float omega_pitch_out;
        float omega_yaw_out;

        float ksi_roll_in;
        float ksi_pitch_in;
        float ksi_yaw_in;
        float omega_roll_in;
        float omega_pitch_in;
        float omega_yaw_in;

	float gamma_nom[3];
	float gamma_sen[3];
	float omega_sen[3];


	float f_z , mg;
	

	float gamma_err[3];//={0,0,0};

	float omega_nom[3];
        float omega_com[3];
	float omega_err[3];//={0,0,0};
	float omega_ctrl[3];

        float M_com[3];
	float M_nom[3];
	float M_ctrl[3];

	float gamma_err_int[3];//={0,0,0};
	float omega_err_int[3];//={0,0,0};

	float phi_nom;//=0;		//gamma_nom[0];
	float theta_nom;//=0;
	float psi_nom;//=0; 
	
	float p_nom;//=0;			//omega_nom[0]
	float q_nom;//=0;
	float r_nom;//=0;
	
	long int N;
 
	int pw[4];

	void Control(void);

        struct uav_state{
            float position[3];
            float velocity[3];
            float attitude[3];
            float anguar_vel[3];
        };

        struct second_order_filter_parameter{
            float m1;
            float m2;
        };

        //void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

        void second_order_filter(uav_state );

        void second_order_filter_e(Eigen::VectorXd* rotor_velocities);

        uav_state state_k;  //the current state of the uav;
        uav_state state_k_1;  //the previous state of the uav;
        uav_state state_k_2;  //the state of the uav at the k-2 sampling time
        uav_state state_filter_m1;   //the middle state of the uav in the 2-order filter
        uav_state state_filter_m2;   //the middle state of the uav in the 2-order filter



private:
        //CSetPa m_setpadlg;  //the time of magnitude of amplitude or small
	void compute_omega_nom(void);
	void compute_M_nom(void);
	void rotate_outerloop_controller(void);
	void rotate_innerloop_controller(void);
};

#endif // #ifndef
