#ifndef COMMUNICATION_DEFINITION_H
#define COMMUNICATION_DEFINITION_H

//#define TEST_WITHOUT_AURCOSLAM
#define TEST_WITH_AURCOSLAM

// ***** BEGIN: MODULE NAMES *****
#define MODULE_NAME_ODOMETRY_STATE_ESTIMATOR    "droneStateEstimator"
#define MODULE_NAME_TRAJECTORY_CONTROLLER       "droneController"
#define MODULE_NAME_ARUCO_EYE                   "ArucoRetina"
#define MODULE_NAME_LOCALIZER                   "droneLocalizer"
#define MODULE_NAME_OBSTACLE_PROCESSOR          "droneObstacleProcessor"
#define MODULE_NAME_TRAJECTORY_PLANNER          "droneTrajectoryPlanner"
#define MODULE_NAME_MISSION_PLANNER             "droneMissionPlanner"
#define MODULE_NAME_YAW_PLANNER					"droneYawPlanner"
#define MODULE_NAME_ARCHITECTURE_BRAIN          "droneArchitectureBrain"
namespace ModuleNames {
    enum name { ODOMETRY_STATE_ESTIMATOR = 1,
                TRAJECTORY_CONTROLLER,
                ARUCO_EYE,
                LOCALIZER,
                OBSTACLE_PROCESSOR,
                TRAJECTORY_PLANNER,
                MISSION_PLANNER,
                YAW_PLANNER,
                ARCHITECTURE_BRAIN
              };
}
// ***** END:   MODULE NAMES *****

// droneArucoEye
#define DRONE_ARUCO_EYE_FRONT_IMAGE_RECT    "ardrone/front/image_rect_color"
#define DRONE_ARUCO_EYE_FRONT_IMAGE_RAW     "ardrone/front/image_raw"
#define DRONE_ARUCO_EYE_OBSERVATIONVEC_LIST "arucoObservation"

// droneLocalizer
#define DRONE_LOCALIZER_POSE_SUBSCRIPTION                       "EstimatedPose_wrt_GFF"
#define DRONE_LOCALIZER_ARUCO_OBSERVATIONVEC_LIST               "arucoObservation"
#define DRONE_LOCALIZER_POSE_PUBLICATION                        "ArucoSlam_EstimatedPose"
#define DRONE_LOCALIZER_POSE_PUBLICATION_2ND_YPR_CONVENTION     "ArucoSlam_EstimatedPose_v2" //Caca
#define DRONE_LOCALIZER_LANDMARK_LIST                           "ArucoSlam_LandarmkList"

// droneObstacleProcessor
#define DRONE_OBSTACLE_PROCESSOR_LANDMARK_LIST  "ArucoSlam_LandarmkList"
#define DRONE_OBSTACLE_PROCESSOR_OBSTACLE_LIST  "obstacles"  // TODO_JL

// droneTrajectoryPlanner
#define DRONE_TRAJECTORY_PLANNER_TRAJ_REF_COM       "droneTrajectoryAbsRefCommand"
#define DRONE_TRAJECTORY_PLANNER_MISSION_POINT_REF  "droneMissionPoint"
#define DRONE_TRAJECTORY_PLANNER_OBSTACLE_LIST      "obstacles"
#define DRONE_TRAJECTORY_PLANNER_SOCIETY_POSE       "societyPose"
#ifdef TEST_WITHOUT_AURCOSLAM
#define DRONE_TRAJECTORY_PLANNER_POSE_SUBSCRIPTION  "EstimatedPose_wrt_GFF"
#endif // TEST_WITHOUT_AURCOSLAM
#ifdef TEST_WITH_AURCOSLAM
#define DRONE_TRAJECTORY_PLANNER_POSE_SUBSCRIPTION  "ArucoSlam_EstimatedPose"
#endif // TEST_WITH_AURCOSLAM

// droneStateEstimator
#define DRONE_STATE_ESTIMATOR_POSE_PUBLICATION_LMrT     "SOEstimatedPose"
#define DRONE_STATE_ESTIMATOR_SPEEDS_PUBLICATION_LMrT   "SOEstimatedSpeeds"
#define DRONE_STATE_ESTIMATOR_POSE_SUBSCRIPTION_LMrT    "SOEstimatedPose"
#define DRONE_STATE_ESTIMATOR_SPEEDS_SUBSCRIPTION_LMrT  "SOEstimatedSpeeds"
#define DRONE_STATE_ESTIMATOR_POSE_PUBLICATION_GMR      "EstimatedPose_wrt_GFF"

// droneController
#define DRONE_CONTROLLER_DRONE_NAV_COMMAND_PUBLICATION      "droneNavCommand"
#define DRONE_CONTROLLER_CTRLR_POSITION_REF_SUBSCRIPTION    "dronePositionRefs"
#define DRONE_CONTROLLER_CTRLR_SPEED_REF_SUBSCRIPTION       "droneSpeedsRefs"
#define DRONE_CONTROLLER_DRONE_NAV_COMMAND_SUBSCRIPTION     "droneNavCommand"
#define DRONE_CONTROLLER_CTRLR_POSITION_REF_PUBLICATION     "dronePositionRefs"
#define DRONE_CONTROLLER_CTRLR_SPEED_REF_PUBLICATION        "droneSpeedsRefs"
#define DRONE_CONTROLLER_ABS_TRAJ_REF_COM_SUBSCRIPTION      "droneTrajectoryAbsRefCommand"
#define DRONE_CONTROLLER_REL_TRAJ_REF_COM_SUBSCRIPTION      "droneTrajectoryRefCommand"
#define DRONE_CONTROLLER_YAW_REF_COM_SUBSCRIPTION           "droneControllerYawRefCommand"
#ifdef TEST_WITHOUT_AURCOSLAM
// see parrotControllerNode
#define DRONE_CONTROLLER_POSE_SUBSCRIPTION                  "¿¿??"
#define DRONE_CONTROLLER_SPEEDS_SUBSCRIPTION                "¿¿??"
#endif // TEST_WITHOUT_AURCOSLAM
#ifdef TEST_WITH_AURCOSLAM
#define DRONE_CONTROLLER_POSE_SUBSCRIPTION                  "ArucoSlam_EstimatedPose"
//#define DRONE_CONTROLLER_SPEEDS_SUBSCRIPTION                "ArucoSlam_EstimatedSpeeds"
#endif // TEST_WITH_AURCOSLAM


// droneBrain
#define DRONE_BRAIN_DRONE_TRAJECTORY_ABS_REF_COM	"droneTrajectoryAbsRefCommand"
#ifdef TEST_WITHOUT_AURCOSLAM
#define DRONE_BRAIN_DRONE_POSE_SUBS	"EstimatedPose_wrt_GFF"
#endif // TEST_WITHOUT_AURCOSLAM
#ifdef TEST_WITH_AURCOSLAM
#define DRONE_BRAIN_DRONE_POSE_SUBS	"ArucoSlam_EstimatedPose"
#endif // TEST_WITH_AURCOSLAM


//droneMissionPlanner
#define DRONE_MISSION_PLANNER_POSE_SUBSCRIPTION	"ArucoSlam_EstimatedPose"
#define DRONE_MISSION_PLANNER_HL_COMMAND        "droneMissionHLCommand"
#define DRONE_MISSION_PLANNER_HL_COMMAND_ACK	"droneMissionHLCommandAck"
#define DRONE_MISSION_PLANNER_MISSION_POINT_REF  "droneMissionPoint"
#define DRONE_MISSION_YAW_REF_COM_SUBS           "droneControllerYawRefCommand"
#define DRONE_MISSION_PLANNER_POINT_TO_LOOK_PUB		"dronePointToLook"
#define DRONE_MISSION_PLANNER_YAW_TO_LOOK_PUB		"droneYawToLook"
#define DRONE_MISSION_INFO_PUB           	"droneMissionInfo"
#define DRONE_MISSION_GO_TASK_SUBS           	"droneGoTask"



//droneYawPlanner
#define DRONE_YAW_PLANNER_POSE_SUBS			"ArucoSlam_EstimatedPose"
#define DRONE_YAW_PLANNER_POINT_TO_LOOK_SUB		"dronePointToLook"
#define DRONE_YAW_PLANNER_YAW_TO_LOOK_SUB		"droneYawToLook"
#define DRONE_YAW_PLANNER_YAW_REF_COM_PUB		"droneControllerYawRefCommand"


// droneArchitectureBrain
#define DRONE_ARCHITECTURE_BRAIN_SOCIETY_BROADCAST  "/societyBroadcast"
#define DRONE_ARCHITECTURE_BRAIN_SOCIETY_POSE       "societyPose"


// droneGraphInterface
#define DRONE_GRAPH_INTERFACE_POSE_SUBS "ArucoSlam_EstimatedPose"
#define DRONE_GRAPH_INTERFACE_TRAJECTORY_SUBS "droneTrajectoryAbsRefCommand"
#define DRONE_GRAPH_INTERFACE_MISSION_POINT_SUBS "droneMissionPoint"
#define DRONE_GRAPH_INTERFACE_SOCIETY_POSE_SUBS "societyPose"
#define DRONE_GRAPH_INTERFACE_OBSTACLES_SUBS "obstacles"

#endif // COMMUNICATION_DEFINITION_H
