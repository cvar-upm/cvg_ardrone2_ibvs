#include "ekf_lmrt_pose_publisher.h"

EKF_LMrT_pose_publisher::EKF_LMrT_pose_publisher(bool pose_publisher_in) {
    matHomog_drone_GMR_t0_wrt_GFF       = cv::Mat::eye(4,4,CV_32F);
    matHomog_drone_LMrT_wrt_drone_GMR   = cv::Mat::eye(4,4,CV_32F);
    matHomog_drone_GMR_wrt_drone_LMrT   = cv::Mat::eye(4,4,CV_32F);
    matHomog_EKF_LMrT_wrt_GFF           = cv::Mat::eye(4,4,CV_32F);
    matHomog_GFF_wrt_EKF_LMrT           = cv::Mat::eye(4,4,CV_32F);
    matHomog_EKF_LMrT_wrt_drone_LMrT_t0 = cv::Mat::eye(4,4,CV_32F);
    matHomog_droneLMrT_wrt_EKF_LMrT     = cv::Mat::eye(4,4,CV_32F);
    matHomog_drone_GMR_wrt_GFF                  = cv::Mat::eye(4,4,CV_32F);
    matHomog_droneLMrT_wrt_EKF_LMrT_4controller = cv::Mat::eye(4,4,CV_32F);
    matHomog_droneGMRwrtGFF             = cv::Mat::eye(4,4,CV_32F);
    matHomog_droneLMrTwrtEKF            = cv::Mat::eye(4,4,CV_32F);

    x = 0.0;
    y = 0.0;
    z = 0.0;
    yaw   = 0.0;
    pitch = 0.0;
    roll  = 0.0;

    pose_publisher = pose_publisher_in;
}

int EKF_LMrT_pose_publisher::initComponents(std::string configFile) {
    std::cout<< "EKF_LMrT_pose_publisher, configFile:" << configFile << std::endl;

    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(configFile);
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result) {
        std::cout<<"EKF_LMrT_pose_publisher::initComponents, I cannot open configFile xml" << std::endl;
        return 0;
    }

    std::string readingValue;

    // Rotation, translation aux var
    float x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;

    pugi::xml_node take_off_site = doc.child("take_off_site");
    pugi::xml_node take_off_site_position = take_off_site.child("position");
    pugi::xml_node take_off_site_attitude = take_off_site.child("attitude");

    // take_off_site_position
    readingValue=take_off_site_position.child_value("x");
    {
    std::istringstream string2float_converter(readingValue);
    string2float_converter>>x_aux;
    }
    readingValue=take_off_site_position.child_value("y");
    {
    std::istringstream string2float_converter(readingValue);
    string2float_converter>>y_aux;
    }
    readingValue=take_off_site_position.child_value("z");
    {
    std::istringstream string2float_converter(readingValue);
    string2float_converter>>z_aux;
    }

    // take_off_site_attitude
    readingValue=take_off_site_attitude.child_value("yaw");
    {
    std::istringstream string2float_converter(readingValue);
    string2float_converter>>yaw_aux;
    }
    readingValue=take_off_site_attitude.child_value("pitch");
    {
    std::istringstream string2float_converter(readingValue);
    string2float_converter>>pitch_aux;
    }
    readingValue=take_off_site_attitude.child_value("roll");
    {
    std::istringstream string2float_converter(readingValue);
    string2float_converter>>roll_aux;
    }
    yaw_aux   = (M_PI/180.0)*yaw_aux;
    pitch_aux = (M_PI/180.0)*pitch_aux;
    roll_aux  = (M_PI/180.0)*roll_aux;

    //    // Homogeneous tranforms
    //    cv::Mat matHomog_drone_GMR_t0_wrt_GFF;          // from droneXX.xml configFile
    //    cv::Mat matHomog_drone_LMrT_wrt_drone_GMR;      // constant, see reference frames' definition (see pdf)
    //    cv::Mat matHomog_drone_GMR_wrt_drone_LMrT;      // constant, see reference frames' definition (see pdf)
    //    cv::Mat matHomog_EKF_LMrT_wrt_drone_LMrT_t0;    // from initial yaw
    //    cv::Mat matHomog_EKF_LMrT_wrt_GFF;              // to obtain SLAMs required transform from EKF data
    //    cv::Mat matHomog_GFF_wrt_EKF_LMrT;              // to obtain droneController required transform from TrajectoryPlanner data
    //    cv::Mat matHomog_droneLMrT_wrt_EKF_LMrT;        // from EKF

    // matHomog_drone_LMrT_wrt_drone_GMR
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_GMR_t0_wrt_GFF,      x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux);
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_LMrT_wrt_drone_GMR,    0.0,   0.0,   0.0,     0.0,       0.0,      M_PI);
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_GMR_wrt_drone_LMrT,    0.0,   0.0,   0.0,     0.0,       0.0,     -M_PI);
//  matHomog_EKF_LMrT_wrt_drone_LMrT_t0;  // is calculated calling the service setDroneYawInitSrv
    matHomog_EKF_LMrT_wrt_GFF = matHomog_drone_GMR_t0_wrt_GFF*matHomog_drone_LMrT_wrt_drone_GMR*matHomog_EKF_LMrT_wrt_drone_LMrT_t0;
    cv::invert( matHomog_EKF_LMrT_wrt_GFF, matHomog_GFF_wrt_EKF_LMrT);
    return 1;
}

void EKF_LMrT_pose_publisher::open(ros::NodeHandle & nIn, std::string moduleName) {
    // DONE: open function

    //Services
    setDroneYawInitSrv     = nIn.advertiseService(moduleName+"/setInitDroneYaw", &EKF_LMrT_pose_publisher::setInitDroneYaw,this);

    if (pose_publisher) {
    //Topics
    droneEstimatedPosePubl = nIn.advertise<droneMsgs::dronePose>(DRONE_STATE_ESTIMATOR_POSE_PUBLICATION_GMR, 1);

    }
}

void EKF_LMrT_pose_publisher::setHomogTransform_rad_wYvPuR( float x, float y, float z, float yaw, float pitch, float roll) {
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_droneLMrT_wrt_EKF_LMrT, x, y, z, yaw, pitch, roll);
}

void EKF_LMrT_pose_publisher::publishHomogTransform_rad_wYvPuR() { // publishes matHomog_drone_GMR_wrt_GFF
    matHomog_drone_GMR_wrt_GFF = matHomog_EKF_LMrT_wrt_GFF*matHomog_droneLMrT_wrt_EKF_LMrT*matHomog_drone_GMR_wrt_drone_LMrT;
    double x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;
    /* int error_ocurred = */referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_drone_GMR_wrt_GFF, &x_aux, &y_aux, &z_aux, &yaw_aux, &pitch_aux, &roll_aux);

    // DONE: publish msg
    droneMsgs::dronePose drone_GMR_wrt_GFF_wYvPuR_msg;
    drone_GMR_wrt_GFF_wYvPuR_msg.x = x_aux;
    drone_GMR_wrt_GFF_wYvPuR_msg.y = y_aux;
    drone_GMR_wrt_GFF_wYvPuR_msg.z = z_aux;
    drone_GMR_wrt_GFF_wYvPuR_msg.yaw   = yaw_aux;
    drone_GMR_wrt_GFF_wYvPuR_msg.pitch = pitch_aux;
    drone_GMR_wrt_GFF_wYvPuR_msg.roll  = roll_aux;
    drone_GMR_wrt_GFF_wYvPuR_msg.YPR_system      = "wYvPuR";
    drone_GMR_wrt_GFF_wYvPuR_msg.target_frame    = "drone_GMR";
    drone_GMR_wrt_GFF_wYvPuR_msg.reference_frame = "GFF";
    droneEstimatedPosePubl.publish(drone_GMR_wrt_GFF_wYvPuR_msg);
}

// DONE: define service properly
bool EKF_LMrT_pose_publisher::setInitDroneYaw(droneStateEstimator::setInitDroneYaw_srv_type::Request& request, droneStateEstimator::setInitDroneYaw_srv_type::Response& response) {
    double yaw_droneLMrT_telemetry_rad = request.yaw_droneLMrT_telemetry_rad;
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_EKF_LMrT_wrt_drone_LMrT_t0, 0.0, 0.0, 0.0, -yaw_droneLMrT_telemetry_rad, 0.0, 0.0);
    matHomog_EKF_LMrT_wrt_GFF = matHomog_drone_GMR_t0_wrt_GFF*matHomog_drone_LMrT_wrt_drone_GMR*matHomog_EKF_LMrT_wrt_drone_LMrT_t0;
    cv::invert( matHomog_EKF_LMrT_wrt_GFF, matHomog_GFF_wrt_EKF_LMrT);

    response.ack = true;
    return response.ack;
}

void EKF_LMrT_pose_publisher::passFROM_droneGMRwrtGFF_TO_droneLMrTwrtEKF( double x, double y, double z, double yaw, double pitch, double roll, double *px, double *py, double *pz, double *pyaw, double *ppitch, double *proll) {
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_droneGMRwrtGFF, x, y, z, yaw, pitch, roll);
    matHomog_droneLMrTwrtEKF = matHomog_GFF_wrt_EKF_LMrT*matHomog_droneGMRwrtGFF*matHomog_drone_LMrT_wrt_drone_GMR;
    referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_droneLMrTwrtEKF, px, py, pz, pyaw, ppitch, proll);
}
