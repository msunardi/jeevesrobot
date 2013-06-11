/* 
 * guidebot Navigation, spring 2012
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
#include <mrpt/hwdrivers.h>
#include <mrpt/maps.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/hwdrivers/CActivMediaRobotBase.h>
#include <iostream>

//Includes for arduino serial communication
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <fstream>
#include <string>
#include <unistd.h>

//includes for network communication
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace std;

#define CONFIG_FILE_NAME    "guidebotNavConf.ini"

/**************************************************************************************************/
/*                                            NAVIGATION PARAMS                                   */		
/**************************************************************************************************/
/* DEFAULT PARAM, SEE "guidebotNavConf.ini" */
static string TTY_PORT       =     "/dev/ttyACM3";
static string COM_PORT       =     "COM4";
static int BAUD_RATE         =     9600;

static float ROBOT_RADIUS    =     0.30f;
static float MIN_STEP        =     0.40f;

static double TURN_THRESHOLD     =  M_PI/90;  /* stop condition while turning */
static double FORWARD_THRESHOLD  =  0.09f;    /* stop condition while forward */ 
static double SHARP_TURN         =  M_PI/5;   /* when robot should stop and turn */ 
static int POLL_INTERVAL         =  100;	  /* delay interval between sensor readings */
static int POLL_INTERVAL_DIV     =  5;

static double ANGULAR_SPEED      =  0.3;      /* angular velocity */
static double LINEAR_SPEED       =  0.2;      /* linear velocity */
static double SMALL_NUMBER       =  0.001;    /* a useful number for comparing purpose */
static int ANGULAR_SPEED_DIV     =  5;
static int LINEAR_SPEED_DIV      =  2;

static string MAP_FILE			=	"FAB-LL-Central-200px.png";
static float MAP_RESOLUTION		= 	0.048768f;
static int X_CENTRAL_PIXEL		=	-1; /* start location, ()-1,-1) means center of map */ 
static int Y_CENTRAL_PIXEL		=	-1;

static CMonteCarloLocalization2D pdf;
static float kinectMinTruncateDistance = 0.5;

static char* LRF_PORT_NAME		=	"/dev/ttyACM3";
static char* ODO_PORT_NAME		=	"/dev/ttyACM3";
static char* MOTOR_PORT_NAME		=	"/dev/ttyACM3";
static int   ODO_READ_MIN		=	7;
static int   LRF_READ_MIN		=	7;
#define PORT "80" // the port client will be connecting to 

#define MAXDATASIZE 100 // max number of bytes we can get at once 

#define LOCAL_HOST "127.0.0.1"

/* our threads 's sharing resources */
struct TThreadRobotParam
{
	TThreadRobotParam() : quit(false), pushed_key(0), Hz(0) { }

	mrpt::synch::CThreadSafeVariable<bool>   						quit;
	mrpt::synch::CThreadSafeVariable<bool>   						stop;
	mrpt::synch::CThreadSafeVariable<int>							pushed_key;
	volatile double Hz;
	mrpt::synch::CThreadSafeVariable<COccupancyGridMap2D>		 * 	gridmap;	//
	mrpt::synch::CThreadSafeVariable<CPose2D>					 	currentOdo;
	mrpt::synch::CThreadSafeVariable<CPose2D>					 	targetOdo;
	
	mrpt::synch::CThreadSafeVariable<CObservation2DRangeScanPtr>    	new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>            new_obs_imu; // Accelerometers
	
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
	mrpt::synch::CThreadSafeVariable<bool>							gettingLRF; //set when robot is using LRF, clear otherwise
	mrpt::synch::CThreadSafeVariable<bool>							isTurning; //set when robot is using LRF, clear otherwise


	mrpt::synch::CThreadSafeVariable<CPose2D>						odometryOffset;
	
	mrpt::synch::CThreadSafeVariable<int>							odo_fd; //file descripter for odometry port
	mrpt::synch::CThreadSafeVariable<int>							lrf_fd; //file descripter for lrf (may be same as odometry)
	mrpt::synch::CThreadSafeVariable<int>							vel_fd;
	//mrpt::synch::CThreadSafeVariable<CPose2D>						pdfMean;
	//mrpt::synch::CThreadSafeVariable<CPose2D>						pdfMostLikely;
};

int scanTest = 0;  //Testing global variable. Delete After test
/* prototypes */
double turnAngle(CActivMediaRobotBase & aRobot, double phi, TThreadRobotParam thrPar);
double turnAngle(double current_phi, double phi);
static void turn( double phi, TThreadRobotParam &p);
static int PathPlanning(std::deque<poses::TPoint2D> &thePath, CPoint2D  origin, CPoint2D  target);
static void smoothDrive(deque<poses::TPoint2D> aPath, TThreadRobotParam & thrPar);
static CObservation2DRangeScan* getKinect2DScan(const TThreadRobotParam & TP, CObservation3DRangeScanPtr & lastObs);
void thread_LRF(TThreadRobotParam &p);
void thread_update_pdf(TThreadRobotParam &p);
void thread_display(TThreadRobotParam &p);
void createCObservationRange( CObservationRange	&obs );
void displaySonars(TThreadRobotParam &p, CObservationRange &sonars);
static void displayFollowPath(TThreadRobotParam &p, deque<poses::TPoint2D> thePath);	
double CObservationRangeLikelihood(COccupancyGridMap2D & map, CPose2D & pose, CObservationRange & obs);
void computePdfLikelihoodValues(COccupancyGridMap2D & map, CMonteCarloLocalization2D & pdf, CObservationRange & obs);
void adjustCObservationRangeSonarPose( CObservationRange &obs );
void thread_wall_detect(TThreadRobotParam &p);
void fixOdometry(CPose2D & pose, CPose2D offset);
int getNextObservation(CObservation2DRangeScan & out_obs, bool there_is, bool hard_error, int fd,TThreadRobotParam &thrPar);
void getOdometry(CPose2D &out_odom, int odo_fd, TThreadRobotParam &thrPar);
int setupArduino(char * port, int readBytes);
int clientCommunication();
int parseServerReply(char * server_reply);
void setVelocities(int linear, int angular, TThreadRobotParam &thrPar);

/**************************************************************************************************/
/*                                         FUNCTION IMPLEMENTATIONS                               */		
/**************************************************************************************************/

/*
 * @Description
 * This thread does montecarlo localization and put the pdf into ThreadRobotParam.
 * Calculation requires a current observation-action pair, from kinect and odometry.
 * A map is also needed. Tuning parameters defined in guidebotNavConf.ini for:
 * - Motion and sensor model 
 * - PDF options
 * - Particle filter options
 * 
 * @param	TThreadRobotParam p (for current odometry)
 * @return	none
 */
void thread_update_pdf(TThreadRobotParam &p)
{
	CPose2D currentOdo, previousOdo;
	CConfigFile guidebotConfFile(CONFIG_FILE_NAME);
	COccupancyGridMap2D		gridmap;
	CParticleFilter::TParticleFilterStats	PF_stats;
	
	/* get the map first */
	bool thereis;
	CImage img;
	float	resolution = MAP_RESOLUTION; 		// size of the grid in meters
	float	xCentralPixel = X_CENTRAL_PIXEL; 			// x central pixel
	float	yCentralPixel = Y_CENTRAL_PIXEL;			// y central pixel 	// Load the gridmap:
		
	if (!mrpt::system::fileExists(MAP_FILE))
		THROW_EXCEPTION_CUSTOM_MSG1("Map file '%s' not found",MAP_FILE.c_str());

	printf("Loading gridmap...");
	gridmap.loadFromBitmapFile(MAP_FILE,resolution ,xCentralPixel,yCentralPixel);
	printf("Done! %f x %f m\n", gridmap.getXMax()-gridmap.getXMin(), gridmap.getYMax()-gridmap.getYMin());
	
	/* insert and likelihood observation options for gridmap */
	COccupancyGridMap2D::TInsertionOptions gridmapOption;
	COccupancyGridMap2D::TLikelihoodOptions likelihoodOption;

	int ind = 0;
 
	gridmapOption.loadFromConfigFile( guidebotConfFile, "MetricMap_occupancyGrid_00_insertOpts" );
	gridmapOption.loadFromConfigFile( guidebotConfFile, "MetricMap_occupancyGrid_00_likelihoodOpts" );
	
	gridmap.insertionOptions = gridmapOption;
	gridmap.likelihoodOptions = likelihoodOption;
	
	/****** MonteCarloLocalization  ******/
	
	
	uint64_t M = guidebotConfFile.read_uint64_t("LocalizationParams","PARTICLE_COUNT",1000, false);
	pdf = CMonteCarloLocalization2D(M);

	/* PDF Options: */
	TMonteCarloLocalizationParams	pdfPredictionOptions;
	pdfPredictionOptions.KLD_params.loadFromConfigFile( guidebotConfFile, "KLD_options");
	pdf.options = pdfPredictionOptions;
	
	pdf.options.metricMap = &gridmap;
	
	/* PF-algorithm Options: */
	CParticleFilter::TParticleFilterOptions		pfOptions;
	pfOptions.loadFromConfigFile( guidebotConfFile, "PF_options" );

	CParticleFilter	PF;
	PF.m_options = pfOptions;
	
	/* motion model */
	CActionRobotMovement2D::TMotionModelOptions dummy_odom_params;
	dummy_odom_params.modelSelection = CActionRobotMovement2D::mmGaussian;
	dummy_odom_params.gausianModel.minStdXY  = guidebotConfFile.read_double("DummyOdometryParams","minStdXY",0.04);
	dummy_odom_params.gausianModel.minStdPHI = DEG2RAD(guidebotConfFile.read_double("DummyOdometryParams","minStdPHI", 2.0));

	CPose2D	pdfEstimation, odometryEstimation;	
	//CPose2D currentOdo, previousOdo;

	/* get a COPY */
	previousOdo.x(p.currentOdo.get().x());
	previousOdo.y(p.currentOdo.get().y());
	previousOdo.phi(p.currentOdo.get().phi());
	
	/* reset all particle to a known location */	
	pdf.resetDeterministic(previousOdo,1000);
	

	/* repeat pdf calculation until terminated */
	while(p.quit.get() == false)
	{
		if ( p.pdfResetDeterministic.get() ) 
		{
			pdf.resetDeterministic(previousOdo,1000);
			p.pdfResetDeterministic.set(false);
		}
		
		/* pair of action-observation */
		CActionCollectionPtr actions;
		CSensoryFramePtr senFrame;		
				
		CObservationPtr obsPtr;		
		
		/* get a copy of the most current pose */
		currentOdo.x(p.currentOdo.get().x());
		currentOdo.y(p.currentOdo.get().y());
		currentOdo.phi(p.currentOdo.get().phi());
				
		/* it IS possible that computeFromOdometry return nothing if distance
		 * between pose is tiny, adjust minStdXY if need */
		
		if (currentOdo.distanceTo(previousOdo) > 2*dummy_odom_params.gausianModel.minStdXY)
		{ 
			/* make sure that we have new kinect reading */
			CObservation2DRangeScanPtr obs_Ptr = p.new_obs.get();			
			CObservation2DRangeScan* obs_2d = obs_Ptr.pointer();			
								
			/* make sure that we have new kinect reading */
			if (obs_2d == NULL)
				continue;
			obs_2d->truncateByDistanceAndAngle(kinectMinTruncateDistance,3);
			senFrame = CSensoryFrame::Create(); /* get a sensoryFrame Ptr */
			actions = CActionCollection::Create(); /* get an AC Ptr */
			/* insert action */
			CActionRobotMovement2DPtr dummy_odom = CActionRobotMovement2D::Create();	
			dummy_odom->computeFromOdometry(currentOdo - previousOdo, dummy_odom_params);				
			actions->insert(*dummy_odom);
		
			/* stamp this pose */
			previousOdo.x(currentOdo.x());
			previousOdo.y(currentOdo.y());
			previousOdo.phi(currentOdo.phi());
				
			/* insert observation */
			obsPtr.setFromPointerDoNotFreeAtDtor(obs_2d);
			senFrame->insert(obsPtr);
				
			/* add observation to our map for visualization */
			CPose3D currentOdo3D(currentOdo);
			obs_2d->insertObservationInto(&gridmap, &currentOdo3D);
			
			//	cout << "-------------------------      PDF     ----------------------" << endl;
			PF.executeOn(
				pdf,
				actions.pointer(),			// Action
				senFrame.pointer(),	// Obs.
				&PF_stats		// Output statistics
				);
	
			CActionRobotMovement2DPtr best_mov_estim = actions->getBestMovementEstimation();

			if (best_mov_estim)
			{
				odometryEstimation = odometryEstimation + best_mov_estim->poseChange->getMeanVal();
			//	cout << "odometryEstimation : " << odometryEstimation << endl;
			}
			pdf.getMean( pdfEstimation );
			//cout << "pdfEstimation : " << pdfEstimation << endl;
			//cout << "mostlikelyParticle: " << pdf.getMostLikelyParticle() << endl;
	
			p.pdf.set( pdf.duplicateGetSmartPtr() );
			p.displayNewPdf.set(true);
			
			//cout << "adaptive sample size : " << PF.adaptiveSampleSizepdf.size() << endl;
			//cout << "-------------------------------------------------------------" << endl;	
			
		}/* end if */			
			mrpt::system::sleep(POLL_INTERVAL); 
	}/* end while */
	
	/* done and save outputs */	
	pdf.saveToTextFile("pdf.txt"); /* Save PDF's m_particles to a text file.*/
	//gridmap.getAsImage(img,false, true);  /* Force a RGB image */
	//const std::string dest = "path_planning1.png";
	//cout << "Saving output to: " << dest << endl;
	//img.saveToFile(dest);
}

/*
 * @Description
 * Thread for grabbing: grabbing lrf observations. This is a replacement for the thread_kinect in the guidebot code
 */
void thread_LRF(TThreadRobotParam &p)
{
	//get lrf file descriptor
	int fd = p.lrf_fd.get();
	sleep(1000);

	try
	{
		const std::string cfgFile = CONFIG_FILE_NAME;
		if (mrpt::system::fileExists(cfgFile))
		{
			cout << "Loading calibration from: "<< cfgFile << endl;
			// Put any code needed to configure LRF from config file
		}
		else cerr << "Warning: Calibration file ["<< cfgFile <<"] not found -> Using default params.\n";

		// Open:
		cout << "Calling LRF::initialize()...";
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs=true, hard_error=false;
		int odometryDelayCounter = 0;

		/* monitor and update hardware status */ 
		while (!hard_error && !p.quit.get() )
		{
			/* Grab new observation from the camera:*/
			CObservation2DRangeScanPtr  obs     = CObservation2DRangeScan::Create(); 
			//CObservationIMUPtr          obs_imu = CObservationIMU::Create();
			if(!p.isMoving.get() & !p.gettingLRF.get())
			{
				getNextObservation(*obs,there_is_obs,hard_error,fd,p);
			}
			if (!hard_error && there_is_obs )
			{
				p.new_obs.set(obs);
			//	p.new_obs_imu.set(obs_imu);
			}	
			sleep(2500);
		}/* end while*/
	}
	catch(std::exception &e)
	{
		cout << "Exception in grabbing thread: " << e.what() << endl;
		p.quit.set(true);
	}
}


/*
 * @Description
 *This function is made to mock the getNextObservation function from the kinect
 *
 * @param	obs: pointer to a CObservation2DRangeScan object created from the LRF
 *		there_is_obs: boolean set if there is a new observation observation
 *		hard_error: true when an error occurs with the LRF
 * @return	none
 */
int getNextObservation(CObservation2DRangeScan & out_obs, bool there_is, bool hard_error, int fd, TThreadRobotParam &thrPar)
{

	unsigned char buf[256];
	char getCommand[1];
	int n;
	cout<<"lrf_odo "<<fd<<endl;
	cout<<thrPar.gettingLRF.get()<<"LRF VALUE"<<endl;	 
	
	thrPar.gettingLRF.set(true);	
	getCommand[0]='l';
	 /* Flush anything already in the serial buffer */
	cout<<thrPar.gettingLRF.get()<<"LRF VALUE"<<endl;	 
	 tcflush(fd, TCIFLUSH);
	 /* read up to 128 bytes from the fd */
	write(fd,getCommand,1);
	buf[0] = 0;

	//read values from serial port
	while(buf[0] != '!')
	{
	 	n = read(fd, buf, 7);
		
		sleep(100);
	 	printf("%i bytes got read...\n", n);
		printf("Buffer has \n%s\n",buf);
 	}
	
	sleep(150);	
	
	 printf("%i bytes got read...\n", n);
	 printf("Buffer 1 contains...\n%d\n", buf[0]);
	 printf("Buffer 2 contains...\n%d\n", buf[1]);
	 printf("Buffer 3 contains...\n%d\n", buf[2]);
	 printf("Buffer 4 contains...\n%d\n", buf[3]);
	 printf("Buffer 5 contains...\n%d\n", buf[4]);

	CPose2D curOdo = thrPar.currentOdo.get();
	CPose2D newOffset(curOdo.x(),curOdo.y(),curOdo.phi());
	out_obs.scan.clear();	
	try {
	fixOdometry( newOffset, thrPar.odometryOffset.get() );
	}
	catch (...) {
		printf("error fixodometry\n");
	}
	out_obs.validRange.clear();
	try {
	out_obs.setSensorPose(thrPar.odometryOffset.get());
	}
	catch (...) {
		printf("error out_obs.setSensorPose\n");
	}
	out_obs.aperture = M_PI*40/180;	
	sleep(1000);
	
	for(int i = 0; i < LRF_READ_MIN; i++)
	{	
//	 	printf("Buffer %d contains...\n%d\n",i,buf[i]);
		if (i > 0)
		{
			out_obs.scan.push_back(float(buf[i])/100);
			out_obs.validRange.push_back(1);
		}
	}
	thrPar.gettingLRF.set(false);
        cout<<"Getting LRf value is set to false"<<endl;
	cout<<thrPar.gettingLRF.get()<<"LRF VALUEx"<<endl;	 
		
	return 0;
}

/*
 * @Description
 * Replacement for the getOdometry function that was part of the CActivMediaRobotBase class
 * @param	out_odom: a pointer to the object in which the current odometry of the robot will get filled in
 *
 */
void getOdometry(CPose2D &out_odom, int odo_fd, TThreadRobotParam &thrPar)
{

	unsigned char buf[256];
	char getCommand[1];
	int n;
	CPose2D tempPose;
	short x,y,phi;
	getCommand[0]='e';
	cout<<"In getting ODO, thread works!"<<endl; 
	try {
	/* Flush anything already in the serial buffer */
	tcflush(odo_fd, TCIFLUSH);
	/* read up to 128 bytes from the fd */
	write(odo_fd,getCommand,1);
	cout<<"written"<<endl;
	cout<<"odo_fd "<<odo_fd<<endl;
	buf[0] = 0;
	while(buf[0] != '*')
	{
	 	n = read(odo_fd, buf, 7);
		
		sleep(100);
 	}
	
	sleep(150);	
	 
	x = ((buf[2] & 255)<< 8) | (buf[1] & 255);	
	y = ((buf[4] & 255) << 8) | (buf[3] & 255);
	phi = ((buf[6] & 255) << 8) | (buf[5] & 255);

	printf("x = %d, y = %d, phi = %d\n",x,y,phi);
	tempPose = thrPar.currentOdo.get();

	out_odom.x(float(x)/100.0);
	out_odom.y(float(y)/100.0);
	out_odom.phi(float(phi)*M_PI/180.0);
	
	sleep(1000);
	}
	catch (...) {
		cout << "Default exception occured!";
	}

}

/*
 * @Description
 * Setup serial communications to arduinos for getting sensor values
 * @param	port: the name of the port the device is on
 *		readBytes: The minimum number of bytes to be read before returning
 */
int setupArduino(char * port, int readBytes)
{

	/* initialize for hardware readings */		
	char *portname = port;
	int fd;
 
	/* Open the file descriptor in non-blocking mode */
	 fd = open(portname, O_RDWR | O_NOCTTY);
	 
	/* Set up the control structure */
	 struct termios toptions;
	 
	 /* Get currently set options for the tty */
	 tcgetattr(fd, &toptions);

	/* Set custom options */
	/* 9600 baud */
	 cfsetispeed(&toptions, B9600);
	 cfsetospeed(&toptions, B9600);

	 /* 8 bits, no parity, no stop bits */
	 toptions.c_cflag &= ~PARENB;
	 toptions.c_cflag &= ~CSTOPB;
	 toptions.c_cflag &= ~CSIZE;
	 toptions.c_cflag |= CS8;
	 /* no hardware flow control */
	 toptions.c_cflag &= ~CRTSCTS;

	 /* enable receiver, ignore status lines */
	 toptions.c_cflag |= CREAD | CLOCAL;

	 /* disable input/output flow control, disable restart chars */
	 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);

	 /* disable canonical input, disable echo,
	 disable visually erase chars,
	 disable terminal-generated signals */
	 toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	 /* disable output processing */
	 toptions.c_oflag &= ~OPOST;	 

	/* wait for 1 characters to come in before read returns */
	 toptions.c_cc[VMIN] = readBytes;

	 /* no minimum time to wait before read returns */
 	toptions.c_cc[VTIME] = 0;

	/* commit the options */
	 tcsetattr(fd, TCSANOW, &toptions); 

	/* Wait for the Arduino to reset */
	sleep(1000);
	return fd;
}

/*
 * @Description
 * Try to drive our robot smoothly: avoid stop (between steps) if don't need to. Also
 * adjust bearing toward the next step. 
 * 
 * @param	aPath:	calculated path
 *			thrPar:	access and update thread parameter list
 * @return	none
 */
static void smoothDrive( deque<poses::TPoint2D> aPath, TThreadRobotParam & thrPar)
{
	CPose2D initOdo;
	int odo_fd = thrPar.odo_fd.get();
	while(thrPar.gettingLRF.get());
	thrPar.isMoving.set(true);
	getOdometry(initOdo, odo_fd, thrPar);
	fixOdometry( initOdo, thrPar.odometryOffset.get() );
	
	CPose2D currentOdo, previousOdo;
	getOdometry(currentOdo, odo_fd, thrPar);	
	fixOdometry( currentOdo, thrPar.odometryOffset.get() );
	thrPar.currentOdo.set(currentOdo);

	getOdometry(previousOdo, odo_fd, thrPar);
	fixOdometry( previousOdo, thrPar.odometryOffset.get() );	
	
	/* for each individual step on path */
	for (deque<poses::TPoint2D>::const_iterator it=aPath.begin(); it!=aPath.end() && (thrPar.quit.get() == false); ++it) 
	{
		double phi;
		double tempAngle;				
		CPose2D tempOdo;
		
		cout << "current pose: " << currentOdo << endl;
		cout <<  "[next nearby target?] x: " << it->x << " y: " << it->y << " " << endl;
		
		//If getting LRF don't move to the next location
		while(thrPar.gettingLRF.get())
		{
			sleep(100);
		}

		
		thrPar.isMoving.set(true);
		/* check for undefined case atan(0,0), for sure */
		if( ( abs(it->y - currentOdo.y()) < SMALL_NUMBER ) && ( abs(it->x - currentOdo.x()) < SMALL_NUMBER ) )
		{			thrPar.currentOdo.set(currentOdo);
				cout << "continue.................. "<< endl;
			continue;
		}
		getOdometry( currentOdo,odo_fd, thrPar);//, v, w, left_ticks, right_ticks );	
		//cout << " Odometry: " << currentOdo; 
		fixOdometry( currentOdo, thrPar.odometryOffset.get() );
		//cout << " Fixed Odometry: " << currentOdo;
		thrPar.currentOdo.set(currentOdo);
		//cout << " Fixed Odometry 2: " << currentOdo << endl;
							
		while(    /* lower bound, make sure robot does not pass target */
		          ( currentOdo.distanceTo(CPoint2D(it->x,it->y)) > FORWARD_THRESHOLD ) 
		          /* upper bound, make sure robot does not go far off target */
		       && ( currentOdo.distanceTo(CPoint2D(it->x,it->y)) < 6 * MIN_STEP ) 
				  /* stop condition from user interface (esc/spacebar key) */
			   && (thrPar.quit.get() == false)
			   && (thrPar.stop.get() == false)
			 ) 
		{				
			/* calculate required phi to next nearby target */		
			phi = atan2 ( (it->y - currentOdo.y() ) , (it->x - currentOdo.x() ) );
			/* the next dozen lines of code basically determine whether robot
			 * should stop and turn or not 
			 */
			tempOdo = CPose2D(currentOdo);				
			tempOdo.normalizePhi();	 		
			tempAngle = abs(phi - tempOdo.phi());
			if ( tempAngle >= M_PI )
			{
				tempAngle = 2*M_PI - tempAngle;
			}
			/*  if current phi > SHARP_TURN, stop and turn */
			if ( tempAngle >= SHARP_TURN )
			{		
				cout << "phi: " << RAD2DEG(phi);
				cout << "\t tempOdo.phi: " << RAD2DEG(tempOdo.phi()) << endl;
				cout << "turning ... " << endl;									
				turn(phi, thrPar);
				cout << "done with turn"<< endl;
				getOdometry(currentOdo,odo_fd, thrPar);
				fixOdometry( currentOdo, thrPar.odometryOffset.get() );
				cout << "Phi after turn: " << currentOdo.phi() << endl;
			}	
			
			/* simple collision avoidance "smooth" drive */
			
			/* if no obstacle */
			if(			(thrPar.leftObstacle.get() == false)	
	    	   		&&	(thrPar.rightObstacle.get() == false)	
	    	   		&& 	(thrPar.centerObstacle.get() == false)		)
			{
				/* driving to next target */	
				/* FIXME : adjust ANGULAR_SPEED_DIV for better drive */
				double turn_angle = turnAngle(currentOdo.phi(), phi) / ANGULAR_SPEED_DIV;
				if (turn_angle < .02) 
					turn_angle = 0;				
				/* FIXME : segmentation fault happens sometime, probably it has something
				 * to do with the fixOdometry()
				 */
				//cout << "after turnangle " << turn_angle << endl;
				  setVelocities( 1,0,thrPar);
				//*setVelocities( LINEAR_SPEED, turn_angle, thrPar); //turnAngle(aRobot, phi, thrPar) / 5 );
			} 
			else if (thrPar.rightObstacle.get() == true && thrPar.leftObstacle.get() == false)
			{
				/*turn to the left to avoid the wall on the right */
				//setVelocities( LINEAR_SPEED / 2, 0.2, thrPar );
				//FIX THIS LATER OBSTICAL AVOIDANCE
				  setVelocities( 0,0,thrPar);
				//sleep(200);			
			}
			else if (thrPar.leftObstacle.get() == true && thrPar.rightObstacle.get() == false)
			{
				/* turn to the right to avoid the wall on the left. */
				//*setVelocities( LINEAR_SPEED / 2, -0.2, thrPar );
				setVelocities(0,0, thrPar);
				//sleep(200);
			}
			else	/* obstacle in front. FIXME: not having any appropriate behaviour for this */
			{
				cout << "error! I am confused and cannot navigate appropriately at this time." << endl;
				//thrPar.stop.set(true);
				//setVelocities(0, 0);
			}

			/* update sonar reading to display */
			CObservationRange obs;
			bool thereis;
			//aRobot.getSonarsReadings(thereis,obs);
			if (!thereis)
			{
				cout << "Sonars: NO" << endl;
			}
			else
			{
				adjustCObservationRangeSonarPose( obs );
				displaySonars(thrPar, obs);
				//cout << "Sonars: ";
				//for (CObservationRange::const_iterator i=obs.sensedData.begin();i!=obs.sensedData.end();++i)
				//	cout << i->sensedDistance << " ";
				//cout << endl;
			}

			mrpt::system::sleep(1000);
		//	mrpt::system::sleep(POLL_INTERVAL);
			//sleep(1000);	
			/* update robot status */
			setVelocities(0,0,thrPar);

			mrpt::system::sleep(1000);
			getOdometry( currentOdo, odo_fd, thrPar); /* for stop condition */
			fixOdometry( currentOdo, thrPar.odometryOffset.get() );
			thrPar.currentOdo.set(currentOdo);
			//cout << "end while" << endl;
		}/* end while */
		
		/* FIXME(FOR GUIDEBOT): update robot odometry to mostlikely particle, below is an example, not tested.
		 * The issue is in the changeOdometry() function, which does not work properly. Our current
		 * fix for this is to do a mapping using odometryOffset
		 */
//		CMonteCarloLocalization2D * tempPdf = (CMonteCarloLocalization2D*)thrPar.pdf.get().pointer();
//		CPose2D tempPose;
//		tempPdf->getMean( tempPose );
//		CPose2D startOdo;
//		getOdometry (startOdo);
//		startOdo.x(0);
//		startOdo.y(0);
//		//CPose2D startOdo(newX,newY,DEG2RAD(newPhi));
//		CPose2D newOffset = tempPose;
//		newOffset.phi( DEG2RAD(tempPose.phi()) - startOdo.phi() );
//		thrPar.odometryOffset.set(newOffset);					
//		aRobot.changeOdometry(startOdo);
//		currentOdo = tempPose;	
//
	//Stop moving a delay to allow getting LRF values
	//*setVelocities( 0, 0, thrPar );
	thrPar.isMoving.set(false);
	sleep(100);	
	} /* end for */

	/* done driving and save outputs */	
	
	//*setVelocities( 0, 0, thrPar );
	thrPar.stop.set(false);
//	
}

/*
 * @Description
 * Get current kinect reading
 * @param	lastObs:	last observation from kinect
 *			TP:			Access to current param list
 * @return	current kinect reading (CObservation2DRangeScan), null if no new reading
 */	
static CObservation2DRangeScan* getKinect2DScan(const TThreadRobotParam & TP, CObservation3DRangeScanPtr & lastObs)
{
	//CObservation3DRangeScanPtr newObs = TP.new_obs.get();
	//CObservation2DRangeScanPtr obs2D;
/*	
	if (newObs && newObs->timestamp!=INVALID_TIMESTAMP &&
		(!lastObs  || newObs->timestamp!=lastObs->timestamp ) )
	{
		// It IS a new observation: 
		lastObs = newObs;
		//last_obs_imu = thrPar.new_obs_imu.get();
		// Update visualization ---------------------------------------

		// Convert ranges to an equivalent 2D "fake laser" scan: 
		if (lastObs->hasRangeImage )
		{			
			// FIXME memory leak ?
			CObservation2DRangeScan* obs2D = new CObservation2DRangeScan();
			lastObs->convertTo2DScan(*obs2D, "KINECT_2D_SCAN");
			return obs2D;
		}
	}*/
	/* else return NULL */
	return NULL;
}

/*
 * @Description
 * This used to be part of the turn function, bring it out here for other uses
 * @param	phi:	calcultated phi to nearby target
 *			aRobot:	access to robot odometry
 *			thrPar:	access to thread parameter list
 * @return	offset to desired bearing
 */
double turnAngle(CActivMediaRobotBase & aRobot, double phi, TThreadRobotParam thrPar)
{
	CPose2D odoTemp;
	double ret;
	int odo_fd = thrPar.odo_fd.get();
	getOdometry( odoTemp,odo_fd, thrPar);
	
	fixOdometry( odoTemp, thrPar.odometryOffset.get() );
	odoTemp.normalizePhi();	

	/* find the optimized turn angle */	
	ret = abs(phi - odoTemp.phi());
	if ( ret >= M_PI )
	{
		ret = 2*M_PI - ret;
	}

	/* find the turn direction: add turnAngle, normalized then compare */	
	
	odoTemp.phi_incr(ret);
	odoTemp.normalizePhi();
	//cout << "after incr" << endl;		
	if ( abs(odoTemp.phi() - phi) > SMALL_NUMBER ) /* small diff */ 
	{
		ret = -ret;
	}
		//cout << "return " << ret << endl;		
	return ret;	
}

/*
 * @Description 
 * overloading method of turnAngle()
 * This used to be part of the turn function, bring it out here for other uses
 *
 * @param	phi:			calcultated phi to nearby target
 *			current phi:	current phi of robot
 * @return	offset to desired bearing
 */
double turnAngle(double current_phi, double phi)
{
	CPose2D odoTemp;
	odoTemp.x(0);
	odoTemp.y(0);
	odoTemp.phi(current_phi);	
	double ret;
	
	odoTemp.normalizePhi();	
	
	/* find the optimized turn angle */	
	ret = abs(phi - odoTemp.phi());
	if ( ret >= M_PI )
	{
		ret = 2*M_PI - ret;
	}

	/* find the turn direction: add turnAngle, normalized then compare */	
	
	odoTemp.phi_incr(ret);
	odoTemp.normalizePhi();

	if ( abs(odoTemp.phi() - phi) > SMALL_NUMBER ) /* small diff */ 
	{
		ret = -ret;
	}

	return ret;			
}

/*
 * @Description
 * @Description
 * Turn robot direction to target next target
 * 
 * @param	phi:	is the desired turn angle, which referenced to the map coordiate,
 *					not the current "phi" of robot phi must be normalized into range 
 *					[-pi,pi] before using. 
 *			p:	 	access and update current odometry to thread parameter list
 * @return	none
 * FIXME: Needs adjustments for MCECS BOT, will sometimes get stuck and spin in a circle
 */
static void turn( double phi, TThreadRobotParam &p)
{
	CPose2D	odo;
	CPose2D	target;
	double	v,w;
	int64_t	left_ticks, right_ticks;
	double	speed;
	double	turnAngle;
	int turnFix; //if the delta between phi is negative switch direction
	int odo_fd = p.odo_fd.get();	
	while(1)
	{
		getOdometry( odo, odo_fd, p);//, v, w, left_ticks, right_ticks );
		fixOdometry( odo, p.odometryOffset.get() );
		p.currentOdo.set(odo); 		// Update current odometry for threads.
		
		/* force odo.phi() to be in the range [-pi,pi]  */	
		odo.normalizePhi();	
		cout<<"TA "<<phi<<","<<odo.phi()<<endl;
		if ( abs(phi - odo.phi()) < .09 ) //was SMALL_NUMBER
			return;
		
		/* turn: Only turn toward target, avoid circling if over-turn */	
		/* find the optimized turn angle */	
		turnAngle = phi; //abs(phi-odo.phi());	
		cout<<"M_PI"<<TURN_THRESHOLD<<endl;	
		cout<<"TA "<<turnAngle<<endl;
		if ( turnAngle >= M_PI )
		{
			turnAngle = 2*M_PI - turnAngle;
		}
		cout<<"TA "<<turnAngle<<endl;
		/* find the turn direction: add turnAngle, normalized then compare */	
		CPose2D odoTemp(odo);
		odoTemp.phi_incr(turnAngle);
		odoTemp.normalizePhi();
		cout<<"TA "<<turnAngle<<endl;


		/* check if we reach turn angle */		
		if( abs(turnAngle) < TURN_THRESHOLD )	
			break;	
		
	//	if((phi -odo.phi())<0) turnFix = -1;
		else turnFix =1;		
		if( abs(phi - odo.phi()) <= M_PI) //if ( abs(odoTemp.phi() - phi) < SMALL_NUMBER ) /* small diff */ 
		{
		cout<<"1"<<endl;
			if( turnAngle < TURN_THRESHOLD * 2 ) /* slow down near desired angle */
				//speed = ANGULAR_SPEED / 2;
				speed = -1;
			else 
				//speed = ANGULAR_SPEED;
				speed = -1;
		}
		else
		{
		cout<<"2"<<endl;
			if( turnAngle < TURN_THRESHOLD * 2 ) 
				//speed = -ANGULAR_SPEED / 2; /* slow down near desired angle */
				speed = 1;
			else 
				//speed = -ANGULAR_SPEED;
				speed = 1;		
		}
		
		mrpt::system::sleep(1100);
		/* do turn */
		cout<<"Speed"<<speed<<endl;
		setVelocities( 0,speed , p );
		
		/* delay between reading */
		//mrpt::system::sleep(POLL_INTERVAL);
		mrpt::system::sleep(350);
		//sleep(250);
		
	setVelocities(0, 0, p); /* stop */
		mrpt::system::sleep(1100);
	}
	setVelocities(0, 0, p); /* stop */
}

bool returnGettingLRF(TThreadRobotParam &thrPar)
{
	return thrPar.gettingLRF.get();
}

/*
 * @Description
 * Send commands to set Velocities 
 * 
 * @param:	linear:		Set robot to move forwad (1) or backward (-1)	
		angular:	set robot to move right (1) or left (-1)
 *		thrPar:	 	access and update current odometry to thread parameter list
 * @return	none
 */
void setVelocities(int linear, int angular, TThreadRobotParam &thrPar)
{
	int velocity_fd = thrPar.vel_fd.get(); //file descriptor for writing to robot velocity		
	char velocityCommand[1];
	int n;
	
	//wait for lrf reading to finish

	while(thrPar.gettingLRF.get());	 
	/* Flush anything already in the serial buffer */
	tcflush(velocity_fd, TCIFLUSH);
	velocityCommand[0] = 0;
	//STOP
	if(linear == 0 && angular == 0)
	{
		velocityCommand[0] = 'x';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
		sleep(100);
	
		write(velocity_fd,velocityCommand,1);
	}
	//LEFT	
	else if(linear == 0 && angular == -1)
	{
		velocityCommand[0] = 'a';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	//RIGHT
	else if(linear == 0 && angular == 1)
	{
		velocityCommand[0] = 'd';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	//FORWARD
	else if(linear == 1 && angular == 0)
	{
		velocityCommand[0] = 'w';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	//BACK
	else if(linear == -1 && angular == 0)
	{
		velocityCommand[0] = 's';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	else
	{
		velocityCommand[0] = 'x';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}

}

/*
 * @Description
 * Thread to display robot and map in a 3D window.
 * 
 * @param	p:	 	thread parameter list
 * @return	none
 */
void thread_display(TThreadRobotParam &p)
{
	mrpt::opengl::CSetOfObjectsPtr		gl_grid,gl_pdf;

	p.displayClearOldPdf.set(true);
	p.displayClearOldSonar.set(true);
	p.displayClearOldPath.set(true);

	CDisplayWindow3D	win("Example of 3D Scene Visualization - MRPT",640,480);
	COccupancyGridMap2D		the_grid;

	CObservation2DRangeScan  last_obs;

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();
	the_grid.loadFromBitmapFile(MAP_FILE,MAP_RESOLUTION /*,xCentralPixel,yCentralPixel*/);

	{ 	/* display map */
		if (!gl_grid) gl_grid = CSetOfObjects::Create();
		gl_grid->clear();
		the_grid.getAs3DObject( gl_grid );
		theScene->insert( gl_grid );
	}

	{	/* display grid */
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-55,55,-41,41,0,1);
		obj->setColor(0.8,0.8,0.8);
		theScene->insert( obj );
	}

	{	/* display current pose */
		opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
		obj->setPose(p.currentOdo.get());
		obj->setName( "robot" );
		theScene->insert( obj );
	}

	{	/* display target point */
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,1,0);
		obj->setRadius(0.1);
		obj->setLocation(0,0,0);
		obj->setName( "target");
		theScene->insert( obj );
	}

	{	/* display sonar readings */
		opengl::CSetOfLinesPtr obj = opengl::CSetOfLines::Create();
		obj->setPose(p.currentOdo.get() );
		obj->setName( "sonars" );
		obj->setColor(1,0,0);
	}

	{	/* display pdf mean *FIXME* Arrow not positioned properly. The point is at the base of the arrow. */ 
		opengl::CArrowPtr obj = opengl::CArrow::Create(0,0,2, 0,0,0, 0.05, 0.01,0.02, 0,0,0 );
		obj->setPose(p.currentOdo.get());
		obj->setName( "mostlikelyParticle" );
		theScene->insert( obj );
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	win.captureImagesStart();

	// Texts:
	win.addTextMessage(0.01,0.85, "This is a 2D message", TColorf(1,1,1),"sans",11, mrpt::opengl::NICE, 0);

	win.setCameraElevationDeg( 25.0f );
	//win.setCameraProjective(false);
	bool end = false;
	CTicTac  timer;
	timer.Tic();

	while (!end && win.isOpen() )
	{
		const double t = timer.Tac();

		// Move the scene:
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();

		// Display the current gridmap x,y position of the cursor on the screen.
		int mouse_x,mouse_y;
		if (win.getLastMousePosition(mouse_x,mouse_y))  // See also: getLastMousePositionRay()
		{
			// Get the ray in 3D for the latest mouse (X,Y):
			mrpt::math::TLine3D ray;
			theScene->getViewport("main")->get3DRayForPixelCoord(mouse_x,mouse_y,ray);

			// Create a 3D plane, e.g. Z=0
			const mrpt::math::TPlane ground_plane(TPoint3D(0,0,0),TPoint3D(1,0,0),TPoint3D(0,1,0));

			// Intersection of the line with the plane:
			mrpt::math::TObject3D inters;
			mrpt::math::intersect(ray,ground_plane, inters);

			// Interpret the intersection as a point, if there is an intersection:
			mrpt::math::TPoint3D inters_pt;
			if (inters.getPoint(inters_pt))
			{
				// Display x,y coordinates and left, center, right collision detect flags.
				string location = format("X,Y Location: %05f, %05f", (float)inters_pt.x,(float)inters_pt.y );
				if( p.leftObstacle.get() ) location += string ( " LEFT" );
				if( p.centerObstacle.get() ) location += string ( " CENTER" );
				if( p.rightObstacle.get() ) location += string ( " RIGHT" );					
				win.addTextMessage(0.01,0.85, location, TColorf(0,0,1),"sans",12, mrpt::opengl::NICE, 0);
			}
		}		

		win.addTextMessage(5,-15,  // |X|,|Y|>1 means absolute coordinates, negative means from the top instead of the bottom.
			format("Time: %s", mrpt::system::dateTimeLocalToString( mrpt::system::now() ).c_str() ),
			TColorf(1,1,1),
			"mono",9,  // font name & size
			mrpt::opengl::NICE,
			20 // An arbitrary ID to always overwrite the same, previous 2D text message
			);

		// Point camera at the current robot position.
		win.setCameraPointingToPoint(p.currentOdo.get().x(),p.currentOdo.get().y(),0);

		/* Change pose of robot in display */
		opengl::CRenderizablePtr obj1 = theScene->getByName("robot");
		obj1->setPose( p.currentOdo.get() ); //.x() , p.currentOdo.get().y() , 0 );

		/* Change location of target marker */
		opengl::CRenderizablePtr obj2 = theScene->getByName("target");
		obj2->setLocation(p.targetOdo.get().x() , p.targetOdo.get().y() , 0 );

		/* Put new path in the display */
		if(p.displayNewPath.get() )  
		{
			std::deque<poses::TPoint2D> thePath(p.thePath.get());
			for (std::deque<poses::TPoint2D>::const_iterator it=thePath.begin();it!=thePath.end();++it) 
			{
				string objName;
				std::stringstream sstm;
				sstm << "path" << it->x << it->y;
				objName = sstm.str();
				opengl::CSpherePtr obj = opengl::CSphere::Create();
				obj->setColor(1,0,0);
				obj->setRadius(0.04);
				obj->setLocation(it->x,it->y,0);
				obj->setName( objName );
				theScene->insert( obj );
			}			
			p.displayNewPath.set(false);
		}

		/* Put new pdf in the display */
		if(p.displayNewPdf.get() )
		{
			if (!gl_pdf) {
				gl_pdf = CSetOfObjects::Create();
				
			}
			if( p.displayClearOldPdf.get() ) gl_pdf->clear();			
			CMonteCarloLocalization2D * tempPdf = (CMonteCarloLocalization2D*)p.pdf.get().pointer();
 			tempPdf->getAs3DObject( gl_pdf );
			theScene->insert( gl_pdf );
			
			opengl::CRenderizablePtr obj4 = theScene->getByName("mostlikelyParticle");
			CPose2D tempPose;
			tempPdf->getMean( tempPose );	
			obj4->setPose( tempPose );	//tempPdf->getMostLikelyParticle() ); 
			
			p.displayNewPdf.set(false);
		}

		/* Put new sonars in the display */
		if(p.displayNewSonars.get() )
		{
			if ( p.displayClearOldSonar.get() ) 
			{
				opengl::CRenderizablePtr obj3 = theScene->getByName("sonars"); 
				if (obj3 != NULL) ( theScene->removeObject(obj3) );
			}
			opengl::CSetOfLinesPtr sonar_object = opengl::CSetOfLines::Create();
			sonar_object->setPose(p.currentOdo.get() );
			sonar_object->setName( "sonars" );
			sonar_object->setColor(1,0,0);
			p.displayNewSonars.set( false );			
			deque<TSegment3D> sonarLines( p.sonars.get() );	
			for (std::deque<TSegment3D>::const_iterator it=sonarLines.begin();it!=sonarLines.end();++it) 
			{
				sonar_object->appendLine( it->point1.x, it->point1.y, .05, it->point2.x, it->point2.y, .05 );
				//cout << it->point1.x << " " << it->point1.y <<" " << it->point2.x << " " << it->point2.y << endl;
			}	
			//cout << endl;
			theScene->insert( sonar_object );
		}

		/* Put new kinect ranges in the display */
		{ 
			/* make sure that we have new kinect reading */	
			CObservation2DRangeScanPtr obs_Ptr = p.new_obs.get();			
			CObservation2DRangeScan* obs_2d = obs_Ptr.pointer();			
								
			/* make sure that we have new kinect reading */
			if (obs_2d != NULL)
			{
				obs_2d->truncateByDistanceAndAngle(kinectMinTruncateDistance,3);
				opengl::CRenderizablePtr obj4 = theScene->getByName( "kinect" ); 
				if (obj4 != NULL) ( theScene->removeObject(obj4) );
				opengl::CPlanarLaserScanPtr kinect_scan = opengl::CPlanarLaserScan::Create();
				kinect_scan->setScan( *obs_2d );
				kinect_scan->setPose(p.currentOdo.get() );
				kinect_scan->setName( "kinect" );
				kinect_scan->setColor(1,0,0);
				theScene->insert( kinect_scan );
			}				
		}

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		mrpt::system::sleep(100);

		/* Code to save images of the screen for creating movies.  */
		/* *FIXME* Caused the kinect to work poorly due to the time spent saving files.  
		//	It may help to reduce the frequency of saving the images 
		// Grab frame:
		//mrpt::utils::CImagePtr img = win.getLastWindowImagePtr();
		//if (img)
		//{
		//	static int i=0;
		//	const string s = format("GRAB_%06i.png", ++i );
		//	img->saveToFile(s);
		//	//printf("Saved frame image to: %s \r",s.c_str() );  // "\ r" is to overwrite the same line over and over again..
		//}
		*/

		/* Get key press events while 3D window is active. */
		if (win.keyHit())
		{

			mrptKeyModifier kmods;
			int key = win.getPushedKey(&kmods);
			printf("Key pushed: %c (%i) - modifiers: 0x%04X\n",char(key),key,kmods);
			
			p.pushed_key.set(key);
	
			switch (key)
			{

				case MRPTK_ESCAPE:
					end = true;
					p.quit.set(true);
					break;

				case MRPTK_RIGHT:
					win.setCameraAzimuthDeg( win.getCameraAzimuthDeg() + 5 );
					break;

				case MRPTK_LEFT: 
					win.setCameraAzimuthDeg( win.getCameraAzimuthDeg() - 5 );
					break;				

				case MRPTK_UP:
					win.setCameraElevationDeg( win.getCameraElevationDeg() + 5 );
					break;

				case MRPTK_DOWN: 
					win.setCameraElevationDeg( win.getCameraElevationDeg() - 5 );
					break;

				/* Note: changing camera pointing to point is temporary because the  
				   pointing to point is changed to current robot odometry each loop. */
				case 'w':
				case 'W':
					{
					float x,y,z;
					win.getCameraPointingToPoint( x,y,z);
					win.setCameraPointingToPoint( x , y + 1, z  );
					break;
					}

				case 'a':
				case 'A':
					{
					float x,y,z;
					win.getCameraPointingToPoint( x,y,z);
					win.setCameraPointingToPoint( x - 1, y , z  );
					break;
					}

				case 'x':
				case 'X':
					{
					float x,y,z;
					win.getCameraPointingToPoint( x,y,z);
					win.setCameraPointingToPoint( x , y - 1, z  );
					break;
					}

				case 'd':
				case 'D':
					{
					float x,y,z;
					win.getCameraPointingToPoint( x,y,z);
					win.setCameraPointingToPoint( x + 1, y , z  );
					break;
					}
				
				/* clear  / do not clear previous pdf before adding new pdf to display */
				case 'p':
					p.displayClearOldPdf.set(true);
					break; 

				case 'P':
					p.displayClearOldPdf.set(false);
					break;

				/* clear  / do not clear previous sonar readings before adding new pdf to display */
				case 's':
					p.displayClearOldSonar.set(true);
					break;

				case 'S':
					p.displayClearOldSonar.set(false);
					break;
	
				/* stops current path finding navigation */
				case ' ':
					p.stop.set(true);
					break;
			}		// end case

		}

	}

}



// ------------------------------------------------------
//				TestPathPlanning
// ------------------------------------------------------
static int PathPlanning(std::deque<poses::TPoint2D> &thePath, CPoint2D  origin, CPoint2D  target)
{

	//string   myGridMap( string("FAB-LL-Central-200px.png") );
	float	resolution = MAP_RESOLUTION; 		// size of the grid in meters
	float	xCentralPixel = X_CENTRAL_PIXEL; 			// x central pixel
	float	yCentralPixel = Y_CENTRAL_PIXEL;			// y central pixel 	// Load the gridmap:

	COccupancyGridMap2D		gridmap;
	
	if (!mrpt::system::fileExists(MAP_FILE))
		THROW_EXCEPTION_CUSTOM_MSG1("Map file '%s' not found",MAP_FILE.c_str());

	printf("Loading gridmap...");
	//CFileGZInputStream(myGridMap) >> gridmap;
	gridmap.loadFromBitmapFile(MAP_FILE,resolution ,xCentralPixel,yCentralPixel);
	printf("Done! %f x %f m\n", gridmap.getXMax()-gridmap.getXMin(), gridmap.getYMax()-gridmap.getYMin());


	// Find path:
	CPathPlanningCircularRobot	pathPlanning;
	pathPlanning.robotRadius = ROBOT_RADIUS;
	pathPlanning.minStepInReturnedPath = MIN_STEP;

	//std::deque<poses::TPoint2D>		thePath;
	bool	notFound;
	CTicTac	tictac;

	cout << "Origin: " << origin << endl;
	cout << "Target: " << target << endl;

	cout << "Searching path..."; cout.flush();
	tictac.Tic();
	pathPlanning.computePath( gridmap, origin, target, thePath, notFound, 1000.0f /* Max. distance */ );

	double t = tictac.Tac();
	cout << "Done in " << t*1000 << " ms" << endl;

	printf("Path found: %s\n", notFound ? "NO":"YES");
	printf("Path has %u steps\n", (unsigned)thePath.size());

	// Save result:
	CImage		img;
	gridmap.getAsImage(img,false, true);  // Force a RGB image

	const std::string dest = "path_planning1.png";
	cout << "Saving output to: " << dest << endl;
	img.saveToFile(dest);
	printf("Done\n");

	// Draw the path:
	// ---------------------
	int R = round(pathPlanning.robotRadius / gridmap.getResolution() );

	for (std::deque<poses::TPoint2D>::const_iterator it=thePath.begin();it!=thePath.end();++it) 
	{
		img.drawCircle( gridmap.x2idx(it->x),gridmap.getSizeY()-1-gridmap.y2idx(it->y),R, TColor(0,0,255) );
//		cout <<  "x: " << it->x << " y: " << it->y << " " << endl;
	}
	img.cross(gridmap.x2idx(origin.x()),gridmap.getSizeY()-1-gridmap.y2idx(origin.y()),TColor(0x20,0x20,0x20),'+',10);
	img.cross(gridmap.x2idx(target.x()),gridmap.getSizeY()-1-gridmap.y2idx(target.y()),TColor(0x50,0x50,0x50),'x',10);

	const std::string dest2 = "path_planning2.png";
	cout << "Saving output to: " << dest2 << endl;
	img.saveToFile(dest2);
	printf("Done\n");

	return notFound ? -1:1;
#if MRPT_HAS_WXWIDGETS
//	mrpt::gui::CDisplayWindow	win("Computed path");
//	win.showImage(img);

//	win.waitForKey();
#endif

}


/*
 * @Description
 * Set up the locations and headings of the sonars for simulating sonar in the map.
 * 
 * @param	obs:	the sonar readings to be set.
 * @return	none
 */
void createCObservationRange( CObservationRange	&obs )
{
	obs.minSensorDistance = 0;
	obs.maxSensorDistance = 7;

	int		i,N = 16;

	obs.sensorLabel = "BASE_SONARS";
	obs.sensorConeApperture = DEG2RAD( 30 );
	obs.timestamp = system::now();

	obs.sensedData.clear();
	float yawVals [] = { 90.00,	50.00,	30.00,	10.00,	-10.00,	-30.00,	-50.00,	-90.00,	-90.00,	-130.00, -150.00, -170.00, 170.00, 150.00, 130.00, 90.00 };
	float xVals [] = { 0.069,	0.114,	0.148,	0.166,	0.166,	0.148,	0.114,	0.069,	-0.02,	-0.024,	-0.058,	-0.077,	-0.077,	-0.058,	-0.024,	-0.02 };
	float yVals [] = { 0.136,	0.119,	0.078,	0.027,	-0.027,	-0.078,	-0.119,	-0.136,	-0.136,	-0.119,	-0.078,	-0.027,	0.027,	0.078,	0.119,	0.136 }; 
	for (i=0;i<N;i++)
	{

		obs.sensedData.push_back( CObservationRange::TMeasurement() );
		CObservationRange::TMeasurement & newObs = obs.sensedData.back();

		newObs.sensorID = i;
		newObs.sensorPose.x = xVals[i];
		newObs.sensorPose.y = yVals[i];
		newObs.sensorPose.z = 0; 
		newObs.sensorPose.yaw = DEG2RAD( yawVals[i] );
		newObs.sensorPose.pitch = 0;
		newObs.sensorPose.roll = 0;

		newObs.sensedDistance = 2;
	}


}

/*
 * @Description
 * Create a deque of line segments from a sonar reading.  To be used by the display thread.
 * 
 * @param	p:		the thread parameter struct.
			sonars: the sonar reading to be converted.
 * @return	none
 */
void displaySonars(TThreadRobotParam &p, CObservationRange &sonars)
{
	deque<TSegment3D> sonarLines;	
	for (CObservationRange::const_iterator i=sonars.sensedData.begin();i!=sonars.sensedData.end();++i)
	{
		//TSegment3D	line;
		TPose3D		sPos;
		sPos = i->sensorPose;					// set sPos as sensor pose.
		//sPos.x = sPos.x + p.currentOdo.get().x();		// add robot pose to sensor pose.
		//sPos.y = sPos.y + p.currentOdo.get().y();		// add robot pose to sensor pose.
		TPoint3D	p1,p2;						// starting and ending points of line segment
		p1.x = sPos.x;
		p1.y = sPos.y;	
		p1.z = 0;								// set first point to sensor pose 
		p2.x = sPos.x + cos( sPos.yaw ) * i->sensedDistance; 
		p2.y = sPos.y + sin( sPos.yaw ) * i->sensedDistance;
		p2.z = 0;
		TSegment3D line( p1,p2 );
		sonarLines.push_back( line );
		//cout << "p1: " << p1 << " p2: " << p2 << endl;
	}
		//cout << "displaySonars: ";		
		//for (std::deque<TSegment3D>::const_iterator it=sonarLines.begin();it!=sonarLines.end();++it) 
		//	{
		//		cout << it->point1 << " ";
		//	}
		//cout << endl;
	p.sonars.set( sonarLines );
	p.displayNewSonars.set( true );
			
}

/*
 * @Description
 * This function will adjust the poses for the sonars of the  
 * PeopleBot to the correct locations and angles.
 * The poses returned by the People bot are wrong.
 * I believe the error is that the Peoplebot thinks the back 
 * sonars are the upper ring, so they are facing forwards.
 * 
 * @param	obs:	the sonar readings to be fixed.
 * @return	none
 */
void adjustCObservationRangeSonarPose( CObservationRange &obs )
{
	int sensor = 0;
	TPose3D sensor_pose;
	
	float yawVals [] = { 90.00,	50.00,	30.00,	10.00,	-10.00,	-30.00,	-50.00,	-90.00,	-90.00,	-130.00, -150.00, -170.00, 170.00, 150.00, 130.00, 90.00 };
	float xVals [] = { 0.069,	0.114,	0.148,	0.166,	0.166,	0.148,	0.114,	0.069,	-0.02,	-0.024,	-0.058,	-0.077,	-0.077,	-0.058,	-0.024,	-0.02 };
	float yVals [] = { 0.136,	0.119,	0.078,	0.027,	-0.027,	-0.078,	-0.119,	-0.136,	-0.136,	-0.119,	-0.078,	-0.027,	0.027,	0.078,	0.119,	0.136 }; 
	float sensed_distance [] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	for (CObservationRange::const_iterator i=obs.sensedData.begin();i!=obs.sensedData.end();++i)
	{
		sensed_distance[sensor] = i->sensedDistance;		
		sensor ++;
	}
	
	sensor = 0;
	int i,N = 16;
	obs.sensedData.clear();
	for (i=0;i<N;i++)
	{

		obs.sensedData.push_back( CObservationRange::TMeasurement() );
		CObservationRange::TMeasurement & newObs = obs.sensedData.back();

		newObs.sensorID = i;
		newObs.sensorPose.x = xVals[i];
		newObs.sensorPose.y = yVals[i];
		newObs.sensorPose.z = 0; 
		newObs.sensorPose.yaw = DEG2RAD( yawVals[i] );
		newObs.sensorPose.pitch = 0;
		newObs.sensorPose.roll = 0;

		newObs.sensedDistance = sensed_distance[sensor];
		sensor++;
	}	

}

/*
 * @Description
 * This function computes the likelihood of the observation given the pose and map.
 * The mrpt library likelihood function only works with laser scans, sonar scans 
 * are not supported. 
 * 
 * @param	map: 	the map for checking the sonar readings.
 *			pose: 	pose that current sonar readings will be checked at.			
 *			obs:	the sonar readings to be checked.
 * @return	none
 */
double CObservationRangeLikelihood(COccupancyGridMap2D & map, CPose2D & pose, CObservationRange & obs)
{
	CObservationRange simObs;	
	float sensed_distance [] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	float simulated_distance [] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	int sensor = 0;
	double return_value = 1.0;
	int valid = 0;

	for (CObservationRange::const_iterator i=obs.sensedData.begin();i!=obs.sensedData.end();++i)
	{
		sensed_distance[sensor] = i->sensedDistance;		
		sensor ++;
	}
	
	map.sonarSimulator(	
		simObs,		//	CObservationRange & 	inout_observation,
		pose,	//	current pose
		0.5f,		//	float	threshold		= 0.5f
		0, 			//	float 	rangeNoiseStd 	= 0, 
		DEG2RAD(0)	// 	float 	angleNoiseStd 	= DEG2RAD(0) 
		);

	sensor = 0;
	for (CObservationRange::const_iterator i=simObs.sensedData.begin();i!=simObs.sensedData.end();++i)
	{
		simulated_distance[sensor] = i->sensedDistance;		
		sensor ++;
	}

	for (int i=0;i< sensor;i++)
	{
		if ( sensed_distance[i] > .05 && simulated_distance[i] > .05 )
		{
			if ( sensed_distance[i] > simulated_distance[i] ) 
			{
				return_value = return_value * ( sensed_distance[i] / simulated_distance[i] );
			}
			else 
			{		
				return_value = return_value * ( simulated_distance[i] / sensed_distance[i] );
			}
			valid++;
		}
	}

	if (valid > (sensor * .75) ) return return_value * valid / sensor;
	else return 0.0;
}

/*
 * @Description
 * *FIXME* Nonfunctional - the particle weights are not correctly set by this function.
 * This function uses the CObservationRangeLikelihood function to calculate likelihood
 * of each pose in the pdf with the current sonar readings.  Setting weights does not 
 * currently work.
 * 
 * @param	map: 	the map for checking the sonar readings.
 *			pdf: 	the list of possible robot poses to be compared to the current sonar readings.			
 *			obs:	the sonar readings to be checked.
 * @return	none
 */
void computePdfLikelihoodValues(COccupancyGridMap2D & map, CMonteCarloLocalization2D & pdf, CObservationRange & obs)
{
	int numberOfParticles = pdf.particlesCount();
	
	for(size_t i=0 ; i < numberOfParticles ; i++)
	{
		CPose2D pose = pdf.getParticlePose(i);
		pdf.setW( i , i/1000 );//CObservationRangeLikelihood( map, pose , obs ) );
	}
	
	pdf.saveToTextFile("test.txt");
	pdf.normalizeWeights();
}

/*
 * @Description
 * Sets flags when the kinect detects range values below 1 meter.  Used for collision
 * avoidance.
 * sets the following threadsafe variables: 
 * - bool leftObstacle
 * - bool centerObstacle
 * - bool rightObstacle
 * 
 * @param	p: 		the thread parameters struct.
 * @return	none
 */
void thread_wall_detect(TThreadRobotParam &p)
{
	//CObservation2DRangeScan  last_obs;		
	/* make sure that we have new kinect reading */
	
	
	while(p.quit == false)
	{
		CObservation2DRangeScanPtr obs_Ptr = p.new_obs.get();			
		CObservation2DRangeScan* obs_2d = obs_Ptr.pointer();			

		if (obs_2d == NULL)
		{
			//cout << "No readings." << endl;
		}
		else 
		{
			obs_2d->truncateByDistanceAndAngle(kinectMinTruncateDistance,3);

			//cout << "Kinect readings: " << endl;
			int count = 0;
//			for (std::vector<float>::iterator it=obs_2d->scan.begin();it!=obs_2d->scan.end();++it)
//			{
//				if(*it >0)
//				{
//					cout << count << " " << *it << endl;
//				}
//				count++;				
//			}
	
			//left

			int	left_count = 0,
				center_count = 0, 
				right_count = 0;
			float left_sum = 0,
				center_sum = 0,
				right_sum = 0;
			int left_offset = 50,
				center_offset = 340,
				right_offset = 680; 
			int stop = 50;			

			for (int i = 0; i < stop; i+=10)  
			{
				if(obs_2d->scan[i+left_offset] > kinectMinTruncateDistance) 
				{
					left_count += 1;
					left_sum += obs_2d->scan[i+left_offset];
				}
				if(obs_2d->scan[i+center_offset] > kinectMinTruncateDistance) 
				{
					center_count += 1;
					center_sum += obs_2d->scan[i+center_offset];
				}
				if(obs_2d->scan[i+right_offset] > kinectMinTruncateDistance) 
				{
					right_count += 1;
					right_sum += obs_2d->scan[i+right_offset];
				}
			}
	
			if(left_count > 0) 
			{
				//cout << "Left: " << left_sum / left_count << "   ";	
				p.leftObstacle.set( ( (left_sum / left_count) > 1) ? false : true );  
			}
			if(center_count > 0) 
			{
				//cout << "Center: " << center_sum / center_count << "    ";			
				p.centerObstacle.set( ( (center_sum / center_count) > 1) ? false : true );  
			} else p.centerObstacle.set( false );
			if(right_count > 0)
			{
				//cout << "Right: " << right_sum / right_count << endl;
				p.rightObstacle.set( ( (right_sum / right_count) > 1) ? false : true ); 
			}
			//else cout << endl;		
		}

		sleep(300);
	} 

}

/*
 * @Description
 * Adjusts the given pose by rotating by phi given in offset and then adds the offset
 * x and y values.
 * 
 * @param	pose: 		the pose to be adjusted.
 * 			offset:		the pose containing the x,y, phi offset.
 * @return	none
 */
void fixOdometry(CPose2D & pose, CPose2D offset)
{

	double x = pose.x();
	double y = pose.y();
	double phi = offset.phi();
	//cout << "fixOdometry x 1: " << x << " y: "<< y << " z: " << phi << endl;

	double xprime = x * cos (phi) - y * sin (phi);
	double yprime = y * cos (phi) + x * sin (phi);

	pose.x( xprime + offset.x() );
	pose.y( yprime + offset.y() );
	double newPhi = pose.phi() + phi;
	pose.phi( newPhi );
	
	//cout << "fixOdometry x 2: " << pose.x() << " y: "<< pose.y() << " z: " << pose.phi() << endl;
	pose.normalizePhi ();

}




int clientCommunication()
{
	int sock;
	struct sockaddr_in server;
	char message[1000] , server_reply[2000];
	
	//Create socket
	sock = socket(AF_INET , SOCK_STREAM , 0);
	if (sock == -1)
	{
		printf("Could not create socket");
	}
	puts("Socket created");
	
	server.sin_addr.s_addr = inet_addr(LOCAL_HOST);
	server.sin_family = AF_INET;
	//server.sin_port = htons( 8888 );
	server.sin_port = htons( 80 );

	//Connect to remote server
	if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		perror("connect failed. Error");
		return 1;
	}
	
	puts("Connected\n");
	
	//keep communicating with server
	while(1)
	{
		printf("Enter message : ");
		scanf("%s" , message);
		
		//Send some data
		//if( send(sock , message , strlen(message) , 0) < 0)
		//{
		//	puts("Send failed");
		//	return 1;
		//}
		
		//Receive a reply from the server
		if( recv(sock , server_reply , 2000 , 0) < 0)
		{
			puts("recv failed");
			break;
		}
	
		// server_reply (char [2000]) is the response (a string) from the server
		/* 
			The message format would be like this:
				<target>:<command>:<input argument1>(:<input argument2>)
			
			The navigation should respond to messages like these from the server:
			- Navigate to a location: 'nav:goto:fab70', 'nav:goto:eb84'
			- Short/small movements: 'nav:move:forward:1', 'nav:move:left:2' (maybe 1 = 30 degrees, 2 = 60 degrees, etc.)
			
			You might need to check the stringmatching.c file from the Project page > Files to see how to use regular expression to parse the message.
		*/

		puts("Server reply :");
		puts(server_reply);
		
		parseServerReply(server_reply);
		// Clear the server_reply array
		memset(server_reply, 0, sizeof server_reply);

		printf("x");
	}
	
	close(sock);
	return 0;
}



int parseServerReply(char * server_reply)
{
	//Function to parse the server reply

}

/**************************************************************************************************/
/*                                            MAIN THREAD                                         */		
/**************************************************************************************************/
/*
 * @Description
 * After starting the threads for kinect, pdf, display and collision avoidance, 
 * loop while accepting manual commands.
 * command menu: 
 * 	- 	a/d   : +/- angular speed
 * 	- 	space : stop current motion
 * 	- 	o     : Query odometry (display odometry is only updated here)
 * 	- 	n     : Query sonars
 * 	- 	b     : Query battery level
 * 	- 	p     : Query bumpers
 * 	- 	P	  : Follow Path (manual control is not available during navigation)
 * 	- 	e	  : Enter new current pose
 * 	- 	t 	  : Enter new target for path Planning
 * 	- 	x     : Quit
 * 
 * @param	pose: 		the pose to be adjusted.
 * 			offset:		the pose containing the x,y, phi offset.
 * @return	none
 */
int main(int argc, char **argv)
{
	try
	{			
		/* read parametters from config file */
		ASSERT_(fileExists(CONFIG_FILE_NAME))
		CConfigFile guidebotConfFile(CONFIG_FILE_NAME);
		
		TTY_PORT       =     guidebotConfFile.read_string("NavigationParams","TTY_PORT","/dev/ttyACM3", true);
		COM_PORT       =     guidebotConfFile.read_string("NavigationParams","COM_PORT","COM4", true);
		BAUD_RATE      =     guidebotConfFile.read_int("NavigationParams","BAUD_RATE",9600, true);
		ROBOT_RADIUS   =     guidebotConfFile.read_float("NavigationParams","ROBOT_RADIUS",0.30f, true);
		MIN_STEP       =     guidebotConfFile.read_float("NavigationParams","MIN_STEP",0.40f, true);
		TURN_THRESHOLD     =  guidebotConfFile.read_double("NavigationParams","TURN_THRESHOLD",M_PI/90, true);  
		SHARP_TURN         =  guidebotConfFile.read_double("NavigationParams","SHARP_TURN",M_PI/5, true);
		FORWARD_THRESHOLD  =  guidebotConfFile.read_double("NavigationParams","FORWARD_THRESHOLD",0.09, true);
		POLL_INTERVAL      =  guidebotConfFile.read_int("NavigationParams","POLL_INTERVAL",100, true);
		ANGULAR_SPEED      =  guidebotConfFile.read_double("NavigationParams","ANGULAR_SPEED",0.3, true);
		LINEAR_SPEED       =  guidebotConfFile.read_double("NavigationParams","LINEAR_SPEED",0.2, true);
		SMALL_NUMBER       =  guidebotConfFile.read_double("NavigationParams","SMALL_NUMBER",0.001, true);
		ANGULAR_SPEED_DIV  =  guidebotConfFile.read_int("NavigationParams","ANGULAR_SPEED_DIV",5, true);
		LINEAR_SPEED_DIV  =  guidebotConfFile.read_int("NavigationParams","LINEAR_SPEED_DIV",2, true);
		
		MAP_FILE			=	guidebotConfFile.read_string("NavigationParams","MAP_FILE","FAB-LL-Central-200px.png",true);
		MAP_RESOLUTION		= 	guidebotConfFile.read_float("NavigationParams","MAP_RESOLUTION",0.048768f, true);
		X_CENTRAL_PIXEL		=	guidebotConfFile.read_int("NavigationParams","X_CENTRAL_PIXEL",-30, true);
		Y_CENTRAL_PIXEL		=	guidebotConfFile.read_int("NavigationParams","Y_CENTRAL_PIXEL",6, true);
		
		kinectMinTruncateDistance = guidebotConfFile.read_float("NavigationParams","kinectMinTruncateDistance", .5f , true);
		
		TURN_THRESHOLD = DEG2RAD(TURN_THRESHOLD);				
		SHARP_TURN     = DEG2RAD(SHARP_TURN);	
		
		/* our robot base object REMOVED FOR MCECS*/
		CActivMediaRobotBase	robot;
/*
#ifdef MRPT_OS_WINDOWS
		string	port= COM_PORT;
#else
		string	port= TTY_PORT;
#endif

		int 	port_baud = BAUD_RATE;

		cout << "Setting serial port to: " << port << " @ " << port_baud << endl;
		robot.setSerialPortConfig( port, port_baud );

		// -------------------------------------------
		//  Init comms:
		// -------------------------------------------
		robot.enableSonars();
		robot.initialize();
*/
		double cur_v = 0;
		double cur_w = 0;

		CActivMediaRobotBase::TRobotDescription  robInfo;
		robot.getRobotInformation(robInfo);

		CPoint2D  target( -29, 8);  // target for path planning.

				
		//CPoint2D  origin( -29, 10 );  // origin for path planning.

	//	cout << "Robot # front bumpers : " << robInfo.nFrontBumpers << endl;
	//	cout << "Robot # rear bumpers  : " << robInfo.nRearBumpers << endl;
	//	cout << "Robot # sonars        : " << robInfo.nSonars << endl;

		/* --------------------------------------------------------
		 * Launch threads
		 * --------------------------------------------------------
		 */	 
		TThreadRobotParam thrPar;
		mrpt::system::TThreadHandle pdfHandle;
		mrpt::system::TThreadHandle displayHandle;
		mrpt::system::TThreadHandle thHandle; 
		mrpt::system::TThreadHandle wallDetectHandle;
	


		//setup arduino for odo
		cout<<"Initializing odometry arduino"<<endl;
		int odo_fd = setupArduino(ODO_PORT_NAME,ODO_READ_MIN);
		thrPar.odo_fd.set(odo_fd);
		cout<<"odo_fd: "<<odo_fd<<endl;
		//if lrf and odometry are on the same arduino port
		if(strcmp(ODO_PORT_NAME,LRF_PORT_NAME) == 0)
		{
			cout<<"Initializing lrf arduino on the same port as odometry"<<endl;
			thrPar.lrf_fd.set(odo_fd);
			
		}
		else
		{
			cout<<"Initializing lrf arduino on unique port"<<endl;
			int lrf_fd = setupArduino(LRF_PORT_NAME,LRF_READ_MIN);
			thrPar.lrf_fd.set(lrf_fd);
		}

		//if motor is also on the same port
		if(strcmp(ODO_PORT_NAME,MOTOR_PORT_NAME) == 0)
		{
			cout<<"Initializing motor arduino on the same port as odometry"<<endl;
			thrPar.vel_fd.set(odo_fd);
		}
		else
		{
			cout<<"Initializing vel arduino on unique port"<<endl;
			int vel_fd = setupArduino(MOTOR_PORT_NAME,1);
			thrPar.vel_fd.set(vel_fd);
		}
		
		//Robot is not in motion, or getting LRF value
		thrPar.isMoving.set(false);
		thrPar.isTurning.set(false);
		thrPar.gettingLRF.set(false);

		pdfHandle = mrpt::system::createThreadRef(thread_update_pdf ,thrPar);
		displayHandle = mrpt::system::createThreadRef(thread_display ,thrPar);
		thHandle = mrpt::system::createThreadRef(thread_LRF ,thrPar);
		wallDetectHandle = mrpt::system::createThreadRef(thread_wall_detect, thrPar);
		/* Wait until data stream starts so we can say for sure the sensor has been initialized OK: */	
		cout << "Waiting for sensor initialization...\n";
		/* May need to do similar for LRF
		do 
		{
			CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
			if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP)
				break;
			else 	
				mrpt::system::sleep(10);			
		} while (!thrPar.quit);
		*/
		/* Check error condition: */
		if (thrPar.quit.get()) 
			return 0;

		bool show_menu = true;
		while (1)
		{		
			if (show_menu)
			{
				show_menu=false;
				cout << "Press the key for your option:" << endl << endl;
				cout << " w/s   : +/- forward or back" << endl;
				cout << " a/d   : +/- left or right" << endl;
				cout << " space : stop" << endl;
				cout << " o     : Query odometry" << endl;
				cout << " n     : Query sonars" << endl;
				cout << " b     : Query battery level" << endl;
				cout << " p     : Query bumpers" << endl;
				cout << " P		: Follow Path" << endl;
				cout << " e		: Enter new current pose: " << endl;
				cout << " t 	: Enter new target for path Planning: " << endl;
				cout << " n	: Network communication mode" << endl;
				cout << " x     : Quit" << endl;
			}

			if (!mrpt::system::os::kbhit())
			{
				robot.doProcess();
				CGenericSensor::TListObservations dummy;
				robot.getObservations(dummy);  // Empty the list
				mrpt::system::sleep(20);
				continue;
			}
			char c = mrpt::system::os::getch();

			show_menu=true;

			if (c=='x') break;	/* quit */

			if (c=='w' || c=='s') /* increase or decrease current linear velocity */
			{
				if (c=='w') cur_v = 1;
				if (c=='s') cur_v = -1;
				setVelocities( cur_v, 0, thrPar);
				sleep(1000);
				setVelocities( 0, 0, thrPar);
			}

			if (c=='a' || c=='d')  /* increase or decrease current angular velocity */
			{
				if (c=='a') cur_w = -1;
				if (c=='d') cur_w = 1;
				setVelocities( 0, cur_w, thrPar );
				sleep(1000);
				setVelocities( 0, 0, thrPar);
			}

			if (c==' ')  /* stop, set current linear and anhular velocities to 0 */
			{
				cur_v = 0;
				cur_w = 0;
				setVelocities( cur_v, cur_w, thrPar );
			}

			if (c=='o')  /* get current odometry reading */
			{
				CPose2D 	odo;
				double 		v,w;
				int64_t  	left_ticks, right_ticks;
				bool 		pollLRF = true;
				cout<<"getting odo"<<endl;
				thrPar.isMoving.set(true);
				//{
				while(thrPar.gettingLRF.get());
				//	pollLRF = returnGettingLRF(thrPar);
				//	cout<<pollLRF<<endl;
				//	sleep(1000);

				//}
					
				cout<<"OK"<<endl;
				getOdometry( odo, odo_fd, thrPar );
				thrPar.isMoving.set(false);
				printf("***x = %d, y = %d, phi = %d\n",odo.x(),odo.y(),odo.phi());
				fixOdometry( odo, thrPar.odometryOffset.get() );
				thrPar.currentOdo.set(odo);
				printf("***x = %d, y = %d, phi = %d\n",odo.x(),odo.y(),odo.phi());
				cout << "Odometry: " << odo << " v: " << v << " w: " << RAD2DEG(w) << " left: " << left_ticks << " right: " << right_ticks << endl;
			}

			if (c=='p')  /* get current bumper readings */
			{
				vector_bool bumps;
				robot.getBumpers(bumps);
				cout << "Bumpers: "<< bumps << endl;
			}

			if (c=='n' || c=='N') /* get current sonar readings */
			{
				CObservationRange obs;
				bool thereis;
				robot.getSonarsReadings(thereis,obs);

				if (!thereis)
				{
					cout << "Sonars: NO" << endl;
				}
				else
				{
					adjustCObservationRangeSonarPose( obs );
					displaySonars(thrPar, obs);
 					cout << "Sonars: ";
					for (CObservationRange::const_iterator i=obs.sensedData.begin();i!=obs.sensedData.end();++i)
						cout << i->sensedDistance << " ";
					cout << endl;
				}
			}

			if (c=='b') /* get current battery charge */
			{
				double bat;
				robot.getBatteryCharge(bat);
				cout << "Battery: " << bat << endl;
			}

			if (c=='e') /* enter current pose */
			{
				double newX, newY, newPhi;
				cout << "Input the current x location: ";
				cin >> newX;	cin.clear();
				cout << "Input the current y location: ";
				cin >> newY;	cin.clear();
				cout << "Input the current phi: ";
				cin >> newPhi;	cin.clear();
				
				CPose2D startOdo;
				getOdometry( startOdo, odo_fd, thrPar);
				startOdo.x(0);
				startOdo.y(0);
				//CPose2D startOdo(newX,newY,DEG2RAD(newPhi));
				CPose2D newOffset(newX,newY,DEG2RAD(newPhi) - startOdo.phi() );
				thrPar.odometryOffset.set(newOffset);					
				robot.changeOdometry(startOdo);	
				CPose2D 	odo;
				double 		v,w;
				int64_t  	left_ticks, right_ticks;
				double 		phi;
				//getOdometryFull( odo, v, w, left_ticks, right_ticks );

				fixOdometry( odo, thrPar.odometryOffset.get() );
				thrPar.currentOdo.set(odo);
				cout << " New odometry has been set! " << odo << endl;
				thrPar.pdfResetDeterministic.set(true);	
				
			}

			if (c=='T') // Turn to target pose
			{
				CPose2D 	odo;		
				/* calculate required phi to next point */			
				double phi = atan2 ( (thrPar.targetOdo.get().y() ) , (thrPar.targetOdo.get().x() ) );		
				turn(45.0*M_PI/180,thrPar);
				cout << "phi: " << RAD2DEG(phi);
				getOdometry( odo, odo_fd, thrPar );
				fixOdometry( odo, thrPar.odometryOffset.get() );
				thrPar.currentOdo.set(odo);
			}

			if (c=='t') /* enter target pose */
			{
				double newX, newY;
				cout << "Input the target x location: ";
				cin >> newX;	cin.clear();
				cout << "Input the target y location: ";
				cin >> newY;	cin.clear();
				
				target.x(newX);
				target.y(newY);
				thrPar.targetOdo.set(target);
				thrPar.targetOdo.set(target);	
					
				cout << " New target has been set! " << target << endl;				
			}

			if (c=='P') /* calculate a path and travel there using the smoothDrive function */
			{				
				std::deque<poses::TPoint2D>		thePath;
				CPose2D 	odo;
				double 		v,w;
				int64_t  	left_ticks, right_ticks;
				double 		phi;
				
				while(thrPar.gettingLRF.get());
				getOdometry( odo, odo_fd, thrPar );
				fixOdometry( odo, thrPar.odometryOffset.get() );
			
				CPoint2D origin( odo.x(), odo.y() );
				
				if (PathPlanning( thePath, origin, target ) == 1) 
				{
					thrPar.pdfResetDeterministic.set(true);					
					thrPar.thePath.set(thePath);
					thrPar.displayNewPath.set(true);
					cout << "found a Path..." << endl;
					smoothDrive( thePath, thrPar );  
					cout << "at target..." << endl;					
				} /* end path following */
				
			}

			if (c=='n')
			{
	
				clientCommunication();			

			}


		}

	/*join threads */
		cout << "Waiting for grabbing thread to exit...\n";
		thrPar.quit = true;
		mrpt::system::joinThread(pdfHandle);
		mrpt::system::joinThread(thHandle);
		mrpt::system::joinThread(wallDetectHandle);
		mrpt::system::joinThread(displayHandle);
		cout << "threads ended!\n";
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}
	return 0;
}
