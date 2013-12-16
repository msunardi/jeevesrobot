/*
 * guidebot Navigation, spring 2012
 * edited: Fall 2013
 * =============================================================================
 * Tested platform: Linux + mrpt lib, guidebbot (ActivMediaRobot) base + Kinect
 * For initial MRPT library and Kinect setup please see our project report.
 * additional files: guidebotNavConf.ini, floorplan (map), run script(optional)
 * =============================================================================
 * implementing:
 * - main thread: command prompt for
 *   + robot status
 *   + manual drive
 *   + path planning and driving robot to target
 * - display thread: 3D display of current robot movement, also shows sensor
 *   readings(kinect and sonar) and particles distribution
 * - pdf update thread: execute montecarlo localization using particles filter
 *   based on the a current pair of action-observation.
 * - kinect observation grabbing thread
 * - wall detect thread
 * =============================================================================
 * Current status and issues
 * - path planning and 3d display works
 * - driving is smooth, however does not work well with wall detect.
 * - localization works in principle however needed some tuning. Particles are
 *   calculated in background and updated to our display ONLY. We have NOT update
 *   current robot location (robot odometry, where robot thinks it is at) with the
 *   most likely location (calculated from particle filter). We think there is an
 *   issue with the changeOdometry() function from MRPT. A temporary fix for this
 *   (odometryOffset) is implemented, but not thoroughly tested.
 */

//MRPT Libraries
#include <mrpt/hwdrivers.h>
#include <mrpt/maps.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/hwdrivers/CActivMediaRobotBase.h>

//Includes for arduino serial communication
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

//includes for network communication
#include <sys/socket.h>
#include <arpa/inet.h>

//Includes for localization


using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::bayes;
using namespace mrpt::random;
using namespace std;

#define CONFIG_FILE_NAME    "guidebotNavConf.ini"

#define BEARING_SENSOR_NOISE_STD 	DEG2RAD(15.0f)
#define RANGE_SENSOR_NOISE_STD 		0.3f
#define DELTA_TIME                  	0.1f

#define VEHICLE_INITIAL_X			0.0f
#define VEHICLE_INITIAL_Y			0.0f
#define VEHICLE_INITIAL_V           1.0f
#define VEHICLE_INITIAL_W           DEG2RAD(20.0f)

#define TRANSITION_MODEL_STD_XY 	0.03f
#define TRANSITION_MODEL_STD_VXY 	0.20f

#define INCH_TO_METER_RATIO         38.4f


//************************************************************************************************
//                                            NAVIGATION PARAMS
//************************************************************************************************
//DEFAULT PARAM, SEE "guidebotNavConf.ini"
static string TTY_PORT       =     "/dev/ttyACM0";
static string COM_PORT       =     "COM4";
static int BAUD_RATE         =     9600;

static float ROBOT_RADIUS    =     0.30f;
static float MIN_STEP        =     0.40f;

static double TURN_THRESHOLD     =  M_PI/90;  // stop condition while turning
static double FORWARD_THRESHOLD  =  0.09f;    // stop condition while forward
static double SHARP_TURN         =  M_PI/5;   // when robot should stop and turn
static int POLL_INTERVAL         =  100;	  // delay interval between sensor readings
static int POLL_INTERVAL_DIV     =  5;

static double ANGULAR_SPEED      =  0.3;      // angular velocity
static double LINEAR_SPEED       =  0.2;      // linear velocity
static double SMALL_NUMBER       =  0.001;    // a useful number for comparing purpose
static int ANGULAR_SPEED_DIV     =  5;
static int LINEAR_SPEED_DIV      =  2;

static string MAP_FILE			=	"MCECSbot_map.png";
static float MAP_RESOLUTION		= 	0.048768f;
static int X_CENTRAL_PIXEL		=	-1; 		// start location, ()-1,-1) means center of map
static int Y_CENTRAL_PIXEL		=	-1;

static CMonteCarloLocalization2D pdf;
static float kinectMinTruncateDistance = 0.5;

static char *LRF_PORT_NAME		=	"/dev/ttyACM0";
static char *ODO_PORT_NAME		=	"/dev/ttyACM0";
static char *MOTOR_PORT_NAME  	=	"/dev/ttyACM0";
static int   ODO_READ_MIN		=	7;
static int   LRF_READ_MIN		=	6;


#define PORT "80" // the port client will be connecting to
#define MAXDATASIZE 100 // max number of bytes we can get at once
#define LOCAL_HOST "127.0.0.1"

// our threads 's sharing resources
struct TThreadRobotParam
{
	TThreadRobotParam() : quit(false), pushed_key(0), Hz(0) { }

	mrpt::synch::CThreadSafeVariable<bool>   						quit;
	mrpt::synch::CThreadSafeVariable<bool>   						stop;
	mrpt::synch::CThreadSafeVariable<int>							pushed_key;
	volatile double Hz;
	mrpt::synch::CThreadSafeVariable<COccupancyGridMap2D>		 * 	gridmap;	//
	mrpt::synch::CThreadSafeVariable<CPose2D>					 	currentOdo;
	mrpt::synch::CThreadSafeVariable<CPose2D>					 	currentKF;
	mrpt::synch::CThreadSafeVariable<CPose2D>					 	currentSonar;
	mrpt::synch::CThreadSafeVariable<CPose2D>					 	targetOdo;

	mrpt::synch::CThreadSafeVariable<CObservation2DRangeScanPtr>    	new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>            new_obs_imu; // Accelerometers
	//mrpt::synch::CThreadSafeVariable<CActivMediaRobotBase>     * robot;   // robot info
	//mrpt::synch::CThreadSafeVariable<CPose2D>                    new_pose;   // robot info

	//mrpt::synch::CThreadSafeVariable<CActionCollectionPtr>       actions;
	//mrpt::synch::CThreadSafeVariable<CSensoryFramePtr>           observations;
	//mrpt::synch::CThreadSafeVariable<CMonteCarloLocalization2D>  pdf;
	mrpt::synch::CThreadSafeVariable<CObjectPtr>                    pdf;
	mrpt::synch::CThreadSafeVariable<CParticleFilter>               PF;
	mrpt::synch::CThreadSafeVariable<bool>							displayNewPdf;

	mrpt::synch::CThreadSafeVariable<std::deque<poses::TPoint2D> > 	thePath;
	mrpt::synch::CThreadSafeVariable<bool>							displayNewPath;

	mrpt::synch::CThreadSafeVariable<deque<TSegment3D> >			sonars;
	mrpt::synch::CThreadSafeVariable<bool>							displayNewSonars;

	mrpt::synch::CThreadSafeVariable<bool>							displayClearOldPdf;
	mrpt::synch::CThreadSafeVariable<bool>							displayClearOldSonar;
	mrpt::synch::CThreadSafeVariable<bool>							displayClearOldPath;

	mrpt::synch::CThreadSafeVariable<bool>							pdfResetDeterministic;

	mrpt::synch::CThreadSafeVariable<bool>							leftObstacle;
	mrpt::synch::CThreadSafeVariable<bool>							rightObstacle;
	mrpt::synch::CThreadSafeVariable<bool>							centerObstacle;
	mrpt::synch::CThreadSafeVariable<bool>							isMoving; //set when robot is moving, clear otherwise
	mrpt::synch::CThreadSafeVariable<bool>							gettingLRF; //set when robot is gathering LRF, clear otherwise
	mrpt::synch::CThreadSafeVariable<bool>							usingLRF; //set when robot is using LRF, clear otherwise
	mrpt::synch::CThreadSafeVariable<bool>							isTurning; //set when robot is using LRF, clear otherwise
	mrpt::synch::CThreadSafeVariable<bool>							goForward;
	mrpt::synch::CThreadSafeVariable<bool>							goRight;


	mrpt::synch::CThreadSafeVariable<CPose2D>						odometryOffset;

	mrpt::synch::CThreadSafeVariable<int>							odo_fd; //file descripter for odometry port
	mrpt::synch::CThreadSafeVariable<int>							lrf_fd; //file descripter for lrf (may be same as odometry)
	mrpt::synch::CThreadSafeVariable<int>							vel_fd;
	mrpt::synch::CThreadSafeVariable<float>                         front_left;
	mrpt::synch::CThreadSafeVariable<float>                         front_right;
	mrpt::synch::CThreadSafeVariable<float>                         front_wall;
	mrpt::synch::CThreadSafeVariable<float>                         right_wall;
	mrpt::synch::CThreadSafeVariable<float>                         left_wall;
	mrpt::synch::CThreadSafeVariable<float>                         D1;
	//mrpt::synch::CThreadSafeVariable<CPose2D>						pdfMean;
	//mrpt::synch::CThreadSafeVariable<CPose2D>						pdfMostLikely;
};

/* jkw wipe*/
float initial_front_wall = 0.0f;
float initial_right_wall = 0.0f;
bool initialize_walls = 1;


// prototypes
int setupArduino(char * port, int readBytes);

void thread_update_pdf(TThreadRobotParam &p);
void thread_LRF(TThreadRobotParam &p);
void thread_display(TThreadRobotParam &p);
void thread_wall_detect(TThreadRobotParam &p);

static int PathPlanning(std::deque<poses::TPoint2D> &thePath, CPoint2D  origin, CPoint2D  target);
void displaySonars(TThreadRobotParam &p, CObservationRange &sonars);

void getOdometry(CPose2D &out_odom, int odo_fd,TThreadRobotParam &thrPar);
void fixOdometry(CPose2D & pose, CPose2D offset);
int getNextObservation(CObservation2DRangeScan & out_obs, bool there_is, bool hard_error, int fd,TThreadRobotParam &thrPar);
void adjustCObservationRangeSonarPose( CObservationRange &obs );

int clientCommunication();
int parseServerReply(char * server_reply);

int parsesequence(TThreadRobotParam &thrPar);
void sequence(char sequencechar, TThreadRobotParam &thrPar);

void move(char direction, TThreadRobotParam &thrPar);
void setVelocities(int linear, int angular, TThreadRobotParam &thrPar);
static void turn( double phi, TThreadRobotParam &p);

static void smoothDrive(CActivMediaRobotBase & aRobot, deque<poses::TPoint2D> aPath, TThreadRobotParam & thrPar);
double turnAngle(CActivMediaRobotBase & aRobot, double phi, TThreadRobotParam thrPar);
double turnAngle(double current_phi, double phi);

void WIFILocalize(CActivMediaRobotBase &robot, TThreadRobotParam &thrPar);

// ------------------------------------------------------
//		Implementation of the system models as a
// ------------------------------------------------------
class CRangeBearing :
	//public mrpt::bayes::CKalmanFilterCapable<4 /* x y vx vy*/, 2 /* range yaw */, 0               , 1 /* Atime */>
								   // <size_t VEH_SIZE,        size_t OBS_SIZE,   size_t FEAT_SIZE, size_t ACT_SIZE, size typename kftype = double>
	public mrpt::bayes::CKalmanFilterCapable<2 /* x y vx vy*/, 2 /* range yaw */, 0               , 1 /* Atime */>
								   // <size_t VEH_SIZE,        size_t OBS_SIZE,   size_t FEAT_SIZE, size_t ACT_SIZE, size typename kftype = double>
{
public:
	CRangeBearing( );
	virtual ~CRangeBearing();

	//void  doProcess( double DeltaTime, double observationRange, double observationBearing );
	void  doProcess( double sonar12,double sonar10, double dy, double dx );

	void getState( KFVector &xkk, KFMatrix &pkk)
	{
		xkk = m_xkk;
		pkk = m_pkk;
	}

 protected:

	float m_obsBearing,m_obsRange;
	float m_deltaTime;
	float m_sonar12;    // latest sonar reading
	float m_sonar10;
	float m_dy; // current positon - previous position
	float m_dx;

	/** @name Virtual methods for Kalman Filter implementation
		@{
	 */

	/** Must return the action vector u.
	  * \param out_u The action vector which will be passed to OnTransitionModel
	  */
	void OnGetAction( KFArray_ACT &out_u ) const;

	/** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
	  * \param in_u The vector returned by OnGetAction.
	  * \param inout_x At input has \f$ \hat{x}_{k-1|k-1} \f$, at output must have \f$ \hat{x}_{k|k-1} \f$.
	  * \param out_skip Set this to true if for some reason you want to skip the prediction step (to do not modify either the vector or the covariance). Default:false
	  */
	void OnTransitionModel(
		const KFArray_ACT &in_u,
		KFArray_VEH       &inout_x,
		bool &out_skipPrediction
		 ) const;

	/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
	  * \param out_F Must return the Jacobian.
	  *  The returned matrix must be \f$N \times N\f$ with N being either the size of the whole state vector or get_vehicle_size().
	  */
	void OnTransitionJacobian(KFMatrix_VxV  &out_F ) const;

	/** Implements the transition noise covariance \f$ Q_k \f$
	  * \param out_Q Must return the covariance matrix.
	  *  The returned matrix must be of the same size than the jacobian from OnTransitionJacobian
	  */
	void OnTransitionNoise(KFMatrix_VxV &out_Q ) const;

	/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
	  * \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
	  * \note Upon call, it can be assumed that the previous contents of out_R are all zeros.
	  */
	void OnGetObservationNoise(KFMatrix_OxO &out_R) const;

	/** This is called between the KF prediction step and the update step, and the application must return the observations and, when applicable, the data association between these observations and the current map.
	  *
	  * \param out_z N vectors, each for one "observation" of length OBS_SIZE, N being the number of "observations": how many observed landmarks for a map, or just one if not applicable.
	  * \param out_data_association An empty vector or, where applicable, a vector where the i'th element corresponds to the position of the observation in the i'th row of out_z within the system state vector (in the range [0,getNumberOfLandmarksInTheMap()-1]), or -1 if it is a new map element and we want to insert it at the end of this KF iteration.
	  * \param in_all_predictions A vector with the prediction of ALL the landmarks in the map. Note that, in contrast, in_S only comprises a subset of all the landmarks.
	  * \param in_S The full covariance matrix of the observation predictions (i.e. the "innovation covariance matrix"). This is a M·O x M·O matrix with M=length of "in_lm_indices_in_S".
	  * \param in_lm_indices_in_S The indices of the map landmarks (range [0,getNumberOfLandmarksInTheMap()-1]) that can be found in the matrix in_S.
	  *
	  *  This method will be called just once for each complete KF iteration.
	  * \note It is assumed that the observations are independent, i.e. there are NO cross-covariances between them.
	  */
	void OnGetObservationsAndDataAssociation(
		vector_KFArray_OBS			&out_z,
		mrpt::vector_int            &out_data_association,
		const vector_KFArray_OBS	&in_all_predictions,
		const KFMatrix              &in_S,
		const vector_size_t         &in_lm_indices_in_S,
		const KFMatrix_OxO          &in_R
		);

		/** Implements the observation prediction \f$ h_i(x) \f$.
		  * \param idx_landmark_to_predict The indices of the landmarks in the map whose predictions are expected as output. For non SLAM-like problems, this input value is undefined and the application should just generate one observation for the given problem.
		  * \param out_predictions The predicted observations.
		  */
		void OnObservationModel(
			const vector_size_t &idx_landmarks_to_predict,
			vector_KFArray_OBS  &out_predictions
			) const;

		/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
		  * \param idx_landmark_to_predict The index of the landmark in the map whose prediction is expected as output. For non SLAM-like problems, this will be zero and the expected output is for the whole state vector.
		  * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
		  * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
		  */
		void OnObservationJacobians(
			const size_t &idx_landmark_to_predict,
			KFMatrix_OxV &Hx,
			KFMatrix_OxF &Hy
			) const;

		/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
		  */
		void OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const;

	/** @}
	 */
};
