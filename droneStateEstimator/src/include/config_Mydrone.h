/*
 * config_Mydrone.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#ifndef CONFIG_MYDRONE_H_
#define CONFIG_MYDRONE_H_

#define _MULTIROTOR_IS_PARROT_
//#define _MULTIROTOR_IS_PELICAN_
//#define _MULTIROTOR_IS_LINKQUAD_

#ifdef _MULTIROTOR_IS_PARROT_
#define MULTIROTOR_PROXY_HOST_IP			"127.0.0.1"
#include "stateObserver/models/EKF_config_Parrot.h"
#include "stateObserver/models/EKF_model_Parrot.h"
typedef ParrotModel1 Multirotor_Model;
#include "controller/config/parrot/config_controller_Parrot.h"
#endif

#ifdef _MULTIROTOR_IS_PELICAN_
#define MULTIROTOR_PROXY_HOST_IP			"10.0.100.3"
#include "stateObserver/models/EKF_config_Pelican.h"
#include "stateObserver/models/EKF_model_Pelican.h"
typedef PelicanModel1 Multirotor_Model;
#include "controller/config/pelican/config_controller_Pelican.h"
#endif

#ifdef _MULTIROTOR_IS_LINKQUAD_
#define MULTIROTOR_PROXY_HOST_IP			"192.168.0.1"
#include "stateObserver/models/EKF_config_LinkQuad.h"
#include "stateObserver/models/EKF_model_LinkQuad.h"
typedef LinkQuadModel1 Multirotor_Model;
#include "controller/config/linkquad/config_controller_LinkQuad.h"
#endif

#endif /* CONFIG_MYDRONE_H_ */
