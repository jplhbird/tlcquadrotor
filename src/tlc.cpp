// TLC.cpp: implementation of the TLC class.
//
//////////////////////////////////////////////////////////////////////



#include "tlc.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <Eigen/Dense>


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TLC::TLC()
{
    char i;
    for (i=0;i<4;i++)
    {
        pw[i] = 2400;
    }
	for (i=0;i<3;i++)
	{
            gamma_nom[i] = 0;
            gamma_sen[i] = 0;
            gamma_err[i] = 0;
            gamma_err_int[i] = 0;
		
            omega_nom[i] = 0;
            omega_com[i] = 0;
            omega_ctrl[i] = 0;
            omega_sen[i] = 0;
            omega_err[i] = 0;
            omega_err_int[i] = 0;

            M_nom[i] = 0;
            M_com[i] = 0;
            M_ctrl[i] = 0;
	}

	mg = G;
	f_z = mg; 
	phi_nom = 0;
	theta_nom = 0;
  	psi_nom = 0;
	p_nom = 0;
	q_nom = 0;
	r_nom = 0;
	T_sampling = T_SAMPLE/1000;
}

TLC::~TLC()
{

}

void TLC::Control(void)
{
	int i;
	float b_t=-27.90f;
	float l=0.549f;	

// 	float G_revise_x=m_setpadlg.m_G_revise_x;
// 	float h_revise_x=m_setpadlg.m_h_revise_x;
// 	float G_revise_y=m_setpadlg.m_G_revise_y;
// 	float h_revise_y=m_setpadlg.m_h_revise_y; /

	compute_omega_nom();
	compute_M_nom();
	rotate_outerloop_controller();
	rotate_innerloop_controller();
	//f_z = mg*cos(gamma_sen[0])*cos(gamma_sen[1])-4*b_t;
        f_z = G-3.3*b_t;   //the inverted single effect



	M_com[0]=M_com[0]-G_revise_x*h_revise_x*cos(gamma_sen[1])*sin(gamma_sen[0]);
	M_com[1]=M_com[1]-G_revise_y*h_revise_y*sin(gamma_sen[1]);
        M_com[2]=0;

//	D_A=D_A*1.05;
//	D_B=D_B*1.05;
//	D_C=D_C*1.05;
//	D_D=D_D*1.05;


        pw[0] =0-M_com[0]/2/D_A + M_com[2]/4/D_C + f_z/4/D_D ;
        pw[1] =0+M_com[1]/2/D_B - M_com[2]/4/D_C + f_z/4/D_D ;
        pw[2] =0 + M_com[0]/2/D_A + M_com[2]/4/D_C + f_z/4/D_D ;
        pw[3] =0 -M_com[1]/2/D_B - M_com[2]/4/D_C + f_z/4/D_D ;
//3300; //
		




	for (i=0;i<4;i++)
	{
		if (pw[i]<2300)
		{
			pw[i] = 2300;
		}
		if (pw[i]>3400)
		{
			pw[i] = 3400;
		}
        }  // bounded the output

}

void TLC::compute_omega_nom(void)
{
        //
  
	float d_phi_nom;
	float d_theta_nom;
        float d_psi_nom;

	float phi_nom_last,theta_nom_last,psi_nom_last;
  
	phi_nom_last=phi_nom;
	theta_nom_last=theta_nom;
        psi_nom_last=psi_nom;
  
	phi_nom=gamma_nom[0];
	theta_nom=gamma_nom[1];
        psi_nom=gamma_nom[2];
  
	d_phi_nom=(phi_nom-phi_nom_last)/T_sampling;
	d_theta_nom=(theta_nom-theta_nom_last)/T_sampling;
	d_psi_nom=(psi_nom-psi_nom_last)/T_sampling;
  
	omega_nom[0]=d_phi_nom-d_psi_nom*sin(theta_nom);
	omega_nom[1]=d_theta_nom*cos(phi_nom)+d_psi_nom*sin(phi_nom)*cos(theta_nom);
	omega_nom[2]=-d_theta_nom*sin(phi_nom)+d_psi_nom*cos(phi_nom)*cos(theta_nom);    
}
 
 
void TLC::compute_M_nom(void)
{
        //
  
	float d_p_nom;
	float d_q_nom;
        float d_r_nom;
	float p_nom_last,q_nom_last,r_nom_last;
 
	p_nom_last=p_nom;
	q_nom_last=q_nom;
        r_nom_last=r_nom;
  
	p_nom=omega_nom[0];
	q_nom=omega_nom[1];
        r_nom=omega_nom[2];
  
	d_p_nom=(p_nom-p_nom_last)/T_sampling;
	d_q_nom=(q_nom-q_nom_last)/T_sampling;
	d_r_nom=(r_nom-r_nom_last)/T_sampling;
  
	M_nom[0]=Ixx*d_p_nom+(Izz-Iyy)*q_nom*r_nom-Ixz*(d_r_nom+q_nom*p_nom);
	M_nom[1]=Iyy*d_q_nom+(Ixx-Izz)*p_nom*r_nom+Ixz*(p_nom*p_nom-r_nom*r_nom);
        M_nom[2]=Izz*d_r_nom+(Iyy-Ixx)*q_nom*p_nom+Ixz*(q_nom*r_nom-d_p_nom);
}

void TLC::rotate_outerloop_controller(void)
//outer loop of the TLC attitude controller, feedback controller
{
	int i,j,k;
  
// 	float ksi_roll_out=m_setpadlg.m_zita_phi;
// 	float ksi_pitch_out=m_setpadlg.m_zita_theta;
// 	float ksi_yaw_out=m_setpadlg.m_zita_psi;
// 	float omega_roll_out=m_setpadlg.m_omega_phi;
// 	float omega_pitch_out=m_setpadlg.m_omega_theta;
// 	float omega_yaw_out=m_setpadlg.m_omega_psi;
	
	float alfa_112;
	float alfa_122;
	float alfa_132;
	float alfa_111;
	float alfa_121;
	float alfa_131;
 
	float K_I1[3][3];	   
        float K_p1[3][3];  //gain matrix
 
        float temp1[3]={0,0,0},temp2[3]={0,0,0};  //template variables
 
        for(i=0;i<3;i++)
	{
            gamma_err[i]=gamma_nom[i]-gamma_sen[i];
            gamma_err_int[i]=gamma_err_int[i]+gamma_err[i]*T_sampling;
	}
 
	alfa_112=2*ksi_roll_out*omega_roll_out;
	alfa_122=2*ksi_pitch_out*omega_pitch_out;
	alfa_132=2*ksi_yaw_out*omega_yaw_out;
	alfa_111=omega_roll_out*omega_roll_out;
	alfa_121=omega_pitch_out*omega_pitch_out;
	alfa_131=omega_yaw_out*omega_yaw_out;
 
	K_I1[0][0]=alfa_111;
	K_I1[0][1]=0;
	K_I1[0][2]=-alfa_131*sin(gamma_nom[1]);
	K_I1[1][0]=0;
	K_I1[1][1]=alfa_121*cos(gamma_nom[0]);
	K_I1[1][2]=alfa_131*sin(gamma_nom[0])*cos(gamma_nom[1]);
	K_I1[2][0]=0;
	K_I1[2][1]=-alfa_121*sin(gamma_nom[0]);
	K_I1[2][2]=alfa_131*cos(gamma_nom[0])*cos(gamma_nom[1]);
 
	K_p1[0][0]=alfa_112;
	K_p1[0][1]=omega_nom[1]*sin(gamma_nom[0])+omega_nom[2]*cos(gamma_nom[0]);
	K_p1[0][2]=-alfa_132*sin(gamma_nom[1]);
	K_p1[1][0]=-omega_nom[2];
	K_p1[1][1]=alfa_122*cos(gamma_nom[0])+(omega_nom[1]*sin(gamma_nom[0])+omega_nom[2]*cos(gamma_nom[0]))*sin(gamma_nom[0])*sin(gamma_nom[1])/cos(gamma_nom[1]);
	K_p1[1][2]=alfa_132*sin(gamma_nom[0])*cos(gamma_nom[1]);
	K_p1[2][0]=omega_nom[1];
	K_p1[2][1]=-alfa_122*sin(gamma_nom[0])+(omega_nom[1]*sin(gamma_nom[0])+omega_nom[2]*cos(gamma_nom[0]))*cos(gamma_nom[0])*sin(gamma_nom[1])/cos(gamma_nom[1]);
	K_p1[2][2]=alfa_132*cos(gamma_nom[0])*cos(gamma_nom[1]);

	for(j=0;j<3;j++)
	{
		for(k=0;k<3;k++)
			temp1[j]=temp1[j]+K_I1[j][k]*gamma_err[k];
        }     //matrix multiply
	for(j=0;j<3;j++)
	{
		for(k=0;k<3;k++)
			temp2[j]=temp2[j]+K_p1[j][k]*gamma_err_int[k];
        }     //matrix multiply
	for(i=0;i<3;i++)
		omega_ctrl[i]=temp1[i]+temp2[i];  
			 
	for(i=0;i<3;i++)
                omega_com[i]=omega_ctrl[i]+omega_nom[i];  //commanded angular velocity
}
 
void TLC::rotate_innerloop_controller(void)
//inner loop feedback controller of TLC controller
{
	int i,j,k;

// 	float ksi_roll_in=m_setpadlg.m_zita_p;
// 	float ksi_pitch_in=m_setpadlg.m_zita_q;
// 	float ksi_yaw_in=m_setpadlg.m_zita_r;
// 	float omega_roll_in=m_setpadlg.m_omega_p;
// 	float omega_pitch_in=m_setpadlg.m_omega_q;
// 	float omega_yaw_in=m_setpadlg.m_omega_r;
 
	float alfa_212;
	float alfa_222;
	float alfa_232;
	float alfa_211;
	float alfa_221;
	float alfa_231;
 
	float K_I2[3][3];
        float K_p2[3][3];  //the control gains
 
	float temp1[3]={0,0,0},temp2[3]={0,0,0};
 
	for(i=0;i<3;i++)
	{
            //angular velocity errors:
            omega_err[i]=omega_com[i]-omega_sen[i];
            omega_err_int[i]=omega_err_int[i]+omega_err[i]*T_sampling;
        }
  
	alfa_212=2*ksi_roll_in*omega_roll_in;
	alfa_222=2*ksi_pitch_in*omega_pitch_in;
	alfa_232=2*ksi_yaw_in*omega_yaw_in;
	alfa_211=omega_roll_in*omega_roll_in;
	alfa_221=omega_pitch_in*omega_pitch_in;
	alfa_231=omega_yaw_in*omega_yaw_in;
  
	K_I2[0][0]=Ixx*alfa_211;
	K_I2[0][1]=0;
	K_I2[0][2]=-Ixz*alfa_231;
	K_I2[1][0]=0;
	K_I2[1][1]=Iyy*alfa_221;
	K_I2[1][2]=0;
	K_I2[2][0]=-Ixz*alfa_211;
	K_I2[2][1]=0;
	K_I2[2][2]=Izz*alfa_231;

	K_p2[0][0]=Ixx*(Ippq*omega_nom[1]+alfa_212)-Ixz*Irpq*omega_nom[1];
	K_p2[0][1]=Ixx*(Ippq*omega_nom[0]+Ipqr*omega_nom[2])-Ixz*(Irpq*omega_nom[0]+Irqr*omega_nom[2]);
	K_p2[0][2]=Ixx*Ipqr*omega_nom[1]-Ixz*(Irqr*omega_nom[1]+alfa_232);
	K_p2[1][0]=Iyy*(2*Iqrr*omega_nom[0]+Iqpr*omega_nom[2]);
	K_p2[1][1]=Iyy*alfa_222;
	K_p2[1][2]=Iyy*(2*Iqrr*omega_nom[2]+Iqpr*omega_nom[0]);
	K_p2[2][0]=-Ixz*(Ippq*omega_nom[1]+alfa_212)+Izz*Irpq*omega_nom[1];
	K_p2[2][1]=-Ixz*(Ippq*omega_nom[0]+Ipqr*omega_nom[2])+Izz*(Irpq*omega_nom[0]+Irqr*omega_nom[2]);
	K_p2[2][2]=-Ixz*Ipqr*omega_nom[1]+Izz*(Irqr*omega_nom[1]+alfa_232);
  
	for(j=0;j<3;j++)
	{
		for(k=0;k<3;k++)
			temp1[j]=temp1[j]+K_I2[j][k]*omega_err[k];
        }     //matrix mutiply
	for(j=0;j<3;j++)
	{
		for(k=0;k<3;k++)
			temp2[j]=temp2[j]+K_p2[j][k]*omega_err_int[k];
        }     //matrix mutiply
	for(i=0;i<3;i++)
        {
                M_ctrl[i]=temp1[i]+temp2[i];
        }

	for(i=0;i<3;i++)
        {
                M_com[i]=M_ctrl[i]+M_nom[i];  //the moment command
        }
 
}


void second_order_filter(Eigen::VectorXd* rotor_velocities){

    for(i=0;i<3;i++){
        V_nom_filter_m2[i]=(a_1_in_trans[i]*V_nom[i]-
        a_1_in_trans[i]*V_nom_filter_m1[i]-a_2_in_trans[i]*V_nom_filter_m2[i])*(T_sampling*time_scale_position)+V_nom_filter_m2[i];
        V_nom_filter_m1[i]=V_nom_filter_m2[i]*(T_sampling*time_scale_position)+V_nom_filter_m1[i];

    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tlcquadrotor");
    TLC tlc_ros;
    ros::spin();


     Eigen::Matrix4d K;
        K.setZero();
        K(0, 0) = 3;
        K(1, 1) = 1;
        K(2, 2) = 2;
        K(3, 3) = 1;


}









