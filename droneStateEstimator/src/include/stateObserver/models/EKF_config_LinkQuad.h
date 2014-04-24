/*
 * EKF_model_LinkQuad.h
 *
 *  Created on: May 9, 2012
 *      Author: jespestana
 */

#ifndef EKF_CONFIG_LINKQUAD_H_
#define EKF_CONFIG_LINKQUAD_H_

//
#define MULTIROTOR_MODEL_NUMSTATES		12
#define MULTIROTOR_MODEL_NUMINPUTS		4
#define MULTIROTOR_MODEL_NUMMEASURES	13

// Obervarion model variance
// Obersvation/Measurement variances
#define STD_OBS_EKF_PITCH_DEG 			1.0/3.0  // 5.0/3.0
#define STD_OBS_EKF_ROLL_DEG  			1.0/3.0
#define STD_OBS_EKF_YAW_ODOMETRY_DEG   	20.0/3.0
#define STD_OBS_EKF_YAW_VICON_DEG		0.5/3.0
#define STD_OBS_EKF_Z_M   				0.2/3.0
#define STD_OBS_EKF_VXM_MPS				0.2/3.0
#define STD_OBS_EKF_VYM_MPS				0.2/3.0
#define	STD_OBS_EKF_X_M					0.2/3.0
#define STD_OBS_EKF_Y_M					0.2/3.0
#define STD_OBS_EKF_UNUSED				1e6

// Model variance
// Actuation variance
#define STD_ACT_EKF_PITCH_DEG	1.0/3.0
#define STD_ACT_EKF_ROLL_DEG	1.0/3.0
#define STD_ACT_EKF_DYAW_DEGPS	2.0/3.0
#define STD_ACT_EKF_DZ_MPS		0.001/3.0

// State Model variance
#define STD_STM_EKF_YAW_DEG		1.0/3.0
#define STD_STM_EKF_Z_M			0.05/3.0
#define STD_STM_EKF_X_M			0.05/3.0
#define STD_STM_EKF_Y_M			0.05/3.0
#define STD_STM_EKF_VX_MPS		0.05/3.0
#define STD_STM_EKF_VY_MPS		0.05/3.0

// Initial state variance
#define STD_IST_EKF_PITCH_DEG		2.0/3.0
#define STD_IST_EKF_ROLL_DEG		2.0/3.0
#define STD_IST_EKF_DYAW1_DEGPS		5.0/3.0
#define STD_IST_EKF_DYAW2_DEGPS		5.0/3.0
#define STD_IST_EKF_YAW_DEG			720.0/3.0
#define STD_IST_EKF_DZ1_MPS			0.2/3.0
#define STD_IST_EKF_DZ2_MPS			0.2/3.0
#define STD_IST_EKF_Z_M				5.0/3.0
#define STD_IST_EKF_X_M				20.0/3.0
#define STD_IST_EKF_Y_M				20.0/3.0
#define STD_IST_EKF_VX_MPS			2.0/3.0
#define STD_IST_EKF_VY_MPS			2.0/3.0

// Mahalanobis distance
#define PARROTEKF_MAHALANOBIS_DISTANCE	1000000.0

#endif /* EKF_CONFIG_LINKQUAD_H_ */
