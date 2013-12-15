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
#include "guidebotNavigation.h"



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

		TTY_PORT       =     guidebotConfFile.read_string("NavigationParams","TTY_PORT","/dev/ttyACM0", true);
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

		MAP_FILE			=	guidebotConfFile.read_string("NavigationParams","MAP_FILE","MCECSbot_map.png",true);
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
		mrpt::system::TThreadHandle thHandle;
		mrpt::system::TThreadHandle displayHandle;
		//mrpt::system::TThreadHandle wallDetectHandle;



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

		//pdfHandle = mrpt::system::createThreadRef(thread_update_pdf ,thrPar);
		displayHandle = mrpt::system::createThreadRef(thread_display ,thrPar);
		thHandle = mrpt::system::createThreadRef(thread_LRF ,thrPar);
		//wallDetectHandle = mrpt::system::createThreadRef(thread_wall_detect, thrPar);
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

		int counter = 0;

        bool run = true;

		while (run)
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

			/*if (!mrpt::system::os::kbhit())
			{
				robot.doProcess();
				CGenericSensor::TListObservations dummy;
				robot.getObservations(dummy);  // Empty the list
				mrpt::system::sleep(20);
				continue;
			}*/

			char c;

 			// ==== Update odometry
				CPose2D 	odo;
				double 		v,w;
				int64_t  	left_ticks, right_ticks;
				bool 		pollLRF = true;
				cout<<"getting odo"<<endl;
				thrPar.isMoving.set(true);
				while(thrPar.gettingLRF.get());
				//{
				//	pollLRF = returnGettingLRF(thrPar);
				//	cout<<pollLRF<<endl;
				//	sleep(1000);

				//}

				cout<<"OK"<<endl;
				getOdometry( odo, odo_fd, thrPar );
				thrPar.isMoving.set(false);
				//printf("***x = %d, y = %d, phi = %d\n",odo.x(),odo.y(),odo.phi());
				fixOdometry( odo, thrPar.odometryOffset.get() );
				thrPar.currentOdo.set(odo);
				//printf("***x = %d, y = %d, phi = %d\n",odo.x(),odo.y(),odo.phi());
				cout << "Odometry: " << odo << " v: " << v << " w: " << RAD2DEG(w) << " left: " << left_ticks << " right: " << right_ticks << endl;
				counter = 0;
            // ==== End update odometry

			c = mrpt::system::os::getch();

			cout << c;

			show_menu=true;

			if (c=='x') run=false;	/* quit */

			if (c=='w' || c=='s') /* increase or decrease current linear velocity */
			{
                		thrPar.goForward.set(true);
				thrPar.goRight.set(false);
                		initial_front_wall = thrPar.front_wall.get();
                		sleep(1000);

				if (c=='w') cur_v = 1;
				if (c=='s') cur_v = -1;


				setVelocities( cur_v, 0, thrPar);
				sleep(1000);
				//setVelocities( 0, 0, thrPar);

			}

			if (c=='a' || c=='d')  /* increase or decrease current angular velocity */
			{
				//if (c=='a') cur_w = 1;
				//if (c=='d') cur_w = 1;
				thrPar.goForward.set(false);
				thrPar.goRight.set(true);
				if (c=='a') cur_w = 1;
				if (c=='d') cur_w = 1;

				setVelocities( 0, cur_w, thrPar );
				//setVelocities( cur_v, 0, thrPar);
				sleep(1000);
				//setVelocities( 0, 0, thrPar);
			}

			if (c==' ')  /* stop, set current linear and anhular velocities to 0 */
			{
				cur_v = 0;
				cur_w = 0;
				thrPar.goForward.set(false);
				thrPar.goRight.set(false);
				setVelocities( cur_v, cur_w, thrPar );
			}


			/*

			if (counter >= 3) {
			    c='o';
			} else {
			    counter += 1;
			}

			if (c=='o')  // get current odometry reading
			{
				CPose2D 	odo;
				double 		v,w;
				int64_t  	left_ticks, right_ticks;
				bool 		pollLRF = true;
				cout<<"getting odo"<<endl;
				thrPar.isMoving.set(true);
				while(thrPar.gettingLRF.get());
				//{
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
				counter = 0;
			}
			*/

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
				getOdometry( odo, odo_fd, thrPar );
				fixOdometry( odo, thrPar.odometryOffset.get() );

				CPoint2D origin( odo.x(), odo.y() );

				if (PathPlanning( thePath, origin, target ) == 1)
				{
					thrPar.pdfResetDeterministic.set(true);
					thrPar.thePath.set(thePath);
					thrPar.displayNewPath.set(true);
					cout << "found a Path..." << endl;
					smoothDrive(robot, thePath, thrPar );
					cout << "at target..." << endl;
				} /* end path following */

			}

			if (c=='n')
			{

				clientCommunication();

			}

			if (c=='m')
			{
			    cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SEQUENCE FOLLOWING >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";

			    parsesequence(thrPar);
			}



		}

	/*join threads */
		cout << "Waiting for grabbing thread to exit...\n";
		thrPar.quit = true;
		mrpt::system::joinThread(pdfHandle);
		mrpt::system::joinThread(thHandle);
		//mrpt::system::joinThread(wallDetectHandle);
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

	/* this part below uniformly distributes particles to the whole map */

//		if ( !guidebotConfFile.read_bool("LocalizationParams","init_PDF_mode",false, /*Fail if not found*/true) )
//		pdf.resetUniformFreeSpace(
//			&gridmap,
//			0.7f,
//			M ,
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_min_x",0,true),
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_max_x",0,true),
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_min_y",0,true),
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_max_y",0,true),
//			DEG2RAD(guidebotConfFile.read_float("LocalizationParams","init_PDF_min_phi_deg",-180)),
//			DEG2RAD(guidebotConfFile.read_float("LocalizationParams","init_PDF_max_phi_deg",180))
//			);
//	else
//		pdf.resetUniform(
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_min_x",0,true),
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_max_x",0,true),
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_min_y",0,true),
//			guidebotConfFile.read_float("LocalizationParams","init_PDF_max_y",0,true),
//			DEG2RAD(guidebotConfFile.read_float("LocalizationParams","init_PDF_min_phi_deg",-180)),
//			DEG2RAD(guidebotConfFile.read_float("LocalizationParams","init_PDF_max_phi_deg",180)),
//			M
//			);

	//CObservation3DRangeScanPtr  last_obs;

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
	//const std::string dest = "path_planning1.";
	//cout << "Saving output to: " << dest << endl;
	//img.saveToFile(dest);
}

/*
 * @Description
 * Thread for grabbing: monitor kinect reading, this is required for getKinect2DScan()
 * calibration file can be used for more accurate reading.
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
		//Put code to initialize LRF :::
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
			//cout << "While !hard_error && !p.quit.get\n";
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
	//cout<<"lrf_odo "<<fd<<endl;
	//cout<<thrPar.gettingLRF.get()<<"LRF VALUE"<<endl;

	thrPar.gettingLRF.set(true);
	getCommand[0]='l';
	 /* Flush anything already in the serial buffer */
	//cout<<thrPar.gettingLRF.get()<<"LRF VALUE"<<endl;
	 tcflush(fd, TCIFLUSH);
	 /* read up to 128 bytes from the fd */
	write(fd,getCommand,1);
	buf[0] = 0;
	while(buf[0] != '!')
	{
	 	n = read(fd, buf, 7);

		sleep(100);
	 	//printf("%i sonar bytes got read...\n", n);
		//printf("Buffer has \n%s\n",buf);
 	}

	sleep(150);
/*jkw
	 printf("%i bytes got read...\n", n);
	 printf("Buffer 1 contains...\n%d (%.03f)\n", buf[1], buf[1]/38.4);
	 printf("Buffer 2 contains...\n%d (%.03f)\n", buf[2], buf[2]/38.4);
	 printf("Buffer 3 contains...\n%d (%.03f)\n", buf[3], buf[3]/38.4);
	 printf("Buffer 4 contains...\n%d (%.03f)\n", buf[4], buf[4]/38.4);
	 printf("Buffer 5 contains...\n%d (%.03f)\n", buf[5], buf[5]/38.4);
*/

    /*
                CPose2D 	odo;
                double 		v,w;
                int64_t  	left_ticks, right_ticks;
                bool 		pollLRF = true;
                cout<<"counter getting odo"<<endl;
                p.isMoving.set(true);
                while(p.gettingLRF.get());
                //{
                //	pollLRF = returnGettingLRF(thrPar);
                //	cout<<pollLRF<<endl;
                //	sleep(1000);

                //}

                cout<<"OK"<<endl;
                getOdometry( odo, fd, p );
                p.isMoving.set(false);
                printf("***x = %d, y = %d, phi = %d\n",odo.x(),odo.y(),odo.phi());
                fixOdometry( odo, p.odometryOffset.get() );
                p.currentOdo.set(odo);
                printf("***x = %d, y = %d, phi = %d\n",odo.x(),odo.y(),odo.phi());
                cout << "Odometry: " << odo << " v: " << v << " w: " << RAD2DEG(w) << " left: " << left_ticks << " right: " << right_ticks << endl;
    */

	CPose2D curOdo = thrPar.currentOdo.get();
	CPose2D newOffset(curOdo.x(),curOdo.y(),curOdo.phi());
	out_obs.scan.clear();
	fixOdometry( newOffset, thrPar.odometryOffset.get() );
	out_obs.validRange.clear();
	out_obs.setSensorPose(thrPar.odometryOffset.get());
	out_obs.aperture = M_PI;//*120/180;
	sleep(1000);

	//for(int i = 1; i < LRF_READ_MIN; i++)
	for(int i=5; i > 0; i--)
	{
//	 	printf("Buffer %d contains...\n%d\n",i,buf[i]);
		if (i > 0)
		{
			//out_obs.scan.push_back(float(buf[i])/100);
			out_obs.scan.push_back(float(buf[i])/INCH_TO_METER_RATIO); // inches converted into metric / map unit
			out_obs.validRange.push_back(1);
			//cout << "yak!";
		}
	}

	if (initialize_walls) {
	    initial_front_wall = float(buf[2])/INCH_TO_METER_RATIO;    // Set initial front wall only the first time the program run
	    initial_right_wall = float(buf[5])/INCH_TO_METER_RATIO;
	    initialize_walls = 0;
	}

	thrPar.front_wall.set(float(buf[2])/INCH_TO_METER_RATIO);  // Save the front sonar reading
	thrPar.right_wall.set(float(buf[5])/INCH_TO_METER_RATIO);  // Save the right sonar reading
	thrPar.left_wall.set(float(buf[1])/INCH_TO_METER_RATIO);   // Save the left sonar reading
	thrPar.front_left.set(float(buf[2])/INCH_TO_METER_RATIO);
	thrPar.front_right.set(float(buf[4])/INCH_TO_METER_RATIO);
	thrPar.gettingLRF.set(false);
	//cout<<thrPar.gettingLRF.get()<<"LRF VALUE"<<endl;

	return 0;
}

/*
 * @Description
 * Replacement for the getOdometry function that was part of the CActivMediaRobotBase class
 * @param	out_odom: a pointer to the object in which the current odometry of the robot will get filled in
 *
 */
void getOdometry(CPose2D &out_odom, int odo_fd,TThreadRobotParam &thrPar)
{

	unsigned char buf[256];
	char getCommand[1];
	int n;
	CPose2D tempPose;
	short x,y,phi;
	getCommand[0]='e';
	cout<<"In getting ODO"<<endl;
	/* Flush anything already in the serial buffer */
	tcflush(odo_fd, TCIFLUSH);
	/* read up to 128 bytes from the fd */
	write(odo_fd,getCommand,1);
	cout<<"written"<<endl;
	cout<<"odo_fd "<<odo_fd<<endl;
	buf[0] = 0;
	while(buf[0] != '*')
	{
		cout<<"reading"<<endl;
	 	n = read(odo_fd, buf, 7);

		sleep(100);
	 	//printf("%i odometry bytes got read...\n", n);
		//printf("Buffer has \n%s\n",buf);
 	}

	sleep(150);

	//read data in
//	n = read(fd, buf,6);

	// printf("Buffer has \n%s\n",buf);
//	 printf("%i bytes got read...\n", n);
//	 printf("Buffer 1 contains...\n%d\n", buf[0]);
//	 printf("Buffer 2 contains...\n%d\n", buf[1]);
//	 printf("Buffer 3 contains...\n%d\n", buf[2]);
//	 printf("Buffer 4 contains...\n%d\n", buf[3]);
//	 printf("Buffer 5 contains...\n%d\n", buf[4]);
//	 printf("Buffer 5 contains...\n%d\n", buf[5]);
//	 printf("Buffer 5 contains...\n%d\n", buf[6]);
	cout<<"read"<<endl;
	x = ((buf[2] & 255)<< 8) | (buf[1] & 255);
	y = ((buf[4] & 255) << 8) | (buf[3] & 255);
	phi = ((buf[6] & 255) << 8) | (buf[5] & 255);

	//flip phi over x axis
//	if(phi != 0)
//	{
//		phi = 360 - phi;
//	}
	//printf("x = %d, y = %d, phi = %d\n",x,y,phi);
	tempPose = thrPar.currentOdo.get();

	thrPar.D1.set(float(x)/100.0);
	//x = x + tempPose.x();
	//y = y + tempPose.y();
	//phi = phi + tempPose.phi();

	out_odom.x(float(x)*0.707/100.0);
	out_odom.y(float(y)*0.707/100.0);
	//out_odom.phi((float(phi)+90)*M_PI/180.0);

	/*if (thrPar.goRight.get() && !thrPar.goForward.get()) {
            cout << "UPDATING X*****\n";
	    out_odom.x(thrPar.currentOdo.get().x() + float(x)/100.0);
	    out_odom.y(thrPar.currentOdo.get().y());
	    out_odom.phi(90*M_PI/180.0);
	} else if (thrPar.goForward.get() && !thrPar.goRight.get()) {
            cout << "UPDATING Y*****\n";
	    out_odom.y(thrPar.currentOdo.get().y() + float(y)/100.0);
	    out_odom.x(thrPar.currentOdo.get().x());
	    out_odom.phi(90*M_PI/180.0);
	} else {
            cout << "NOT UPDATING ODOMETRY!!!!!\n";
            out_odom.x(thrPar.currentOdo.get().x());
            out_odom.y(thrPar.currentOdo.get().y());
            out_odom.phi(thrPar.currentOdo.get().phi());
        }*/
	//out_odom.phi((float(phi)+90)*M_PI/180.0);
	out_odom.phi(90*M_PI/180.0);
	cout << format("Odometry: x=%.03f, y=%.03f, phi=%.03f",(float)out_odom.x(), (float)out_odom.y(), (float)out_odom.phi()) << endl;


	sleep(1000);


}

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
 *			aRobot:	access to robot odometry
 *			thrPar:	access and update thread parameter list
 * @return	none
 */
static void smoothDrive(CActivMediaRobotBase & aRobot, deque<poses::TPoint2D> aPath, TThreadRobotParam & thrPar)
{
	CPose2D initOdo;
	int odo_fd = thrPar.odo_fd.get();
	getOdometry(initOdo, odo_fd, thrPar);
	 ( initOdo, thrPar.odometryOffset.get() );

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
			aRobot.getSonarsReadings(thereis,obs);
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

		/* FIXME: update robot odometry to mostlikely particle, below is an example, not tested.
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
 *			robot:	access to robot odometry
 *			p:	 	access and update current odometry to thread parameter list
 * @return	none
 */
static void turn( double phi, TThreadRobotParam &p)
{
	CPose2D	odo;
	CPose2D	target;
	double	v,w;
	int64_t	left_ticks, right_ticks;
	double	speed;
	double	turnAngle;
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

		/* check if we reach turn angle */
		if( turnAngle < TURN_THRESHOLD )
			break;

		if ( abs(odoTemp.phi() - phi) < SMALL_NUMBER ) /* small diff */
		{
		cout<<"1"<<endl;
			if( turnAngle < TURN_THRESHOLD * 2 ) /* slow down near desired angle */
				//speed = ANGULAR_SPEED / 2;
				speed = 1;
			else
				//speed = ANGULAR_SPEED;
				speed = 1;
		}
		else
		{
		cout<<"2"<<endl;
			if( turnAngle < TURN_THRESHOLD * 2 )
				//speed = -ANGULAR_SPEED / 2; /* slow down near desired angle */
				speed = -1;
			else
				//speed = -ANGULAR_SPEED;
				speed = -1;
		}

		//mrpt::system::sleep(500);
		/* do turn */
		cout<<"Speed"<<speed<<endl;
		setVelocities( 0,-1* speed , p );

		/* delay between reading */
		//mrpt::system::sleep(POLL_INTERVAL);
		mrpt::system::sleep(500);
		//sleep(250);

	setVelocities(0, 0, p); /* stop */
		mrpt::system::sleep(1200);
	}
	setVelocities(0, 0, p); /* stop */
}

bool returnGettingLRF(TThreadRobotParam &thrPar)
{
	return thrPar.gettingLRF.get();
}

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
		sleep(100);
		write(velocity_fd,velocityCommand,1);
		sleep(100);
		write(velocity_fd,velocityCommand,1);
		sleep(100);
		write(velocity_fd,velocityCommand,1);
		sleep(100);
	}
	//LEFT
	else if(linear == 0 && angular == -1)
	{
	    cout << "****************************** LEFT ******************************\n";
		velocityCommand[0] = 'a';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	//RIGHT
	else if(linear == 0 && angular == 1)
	{
	    cout << "****************************** RIGHT ******************************\n";
		velocityCommand[0] = 'd';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	//FORWARD
	else if(linear == 1 && angular == 0)
	{
	    cout << "****************************** FORWARD ******************************\n";
		velocityCommand[0] = 'w';
		/* read up to 128 bytes from the fd */
		write(velocity_fd,velocityCommand,1);
	}
	//BACK
	else if(linear == -1 && angular == 0)
	{
	    cout << "****************************** BACKWARD ******************************\n";
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

    {	/* display pose by encoder */
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(1,1,0);
		obj->setRadius(0.1);
		obj->setLocation(0,0,0);
		obj->setName( "encoder");
		theScene->insert( obj );
	}

	{	/* display pose by sonar */
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,0,1);
		obj->setRadius(0.1);
		obj->setLocation(0,0,0);
		obj->setName( "sonarpos");
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

	{	// display sonar readings
		opengl::CSetOfLinesPtr obj = opengl::CSetOfLines::Create();
		obj->setPose(p.currentOdo.get() );
		obj->setName( "sonars" );
		obj->setColor(1,0,0);
	}

	{	/* display pdf mean *FIXME* Arrow not positioned properly. The point is at the base of the arrow. */
		opengl::CArrowPtr obj = opengl::CArrow::Create(0,0,2, 0,0,0, 0.05, 0.01,0.02, 0,0,0 );
		obj->setPose(p.currentOdo.get());
		obj->setName( "mostlikelyParticle" );
		// uncomment to display particles
		//theScene->insert( obj );
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	win.captureImagesStart();

	// Texts:
	win.addTextMessage(0.01,0.85, "This is a 2D message", TColorf(1,1,1),"sans",11, mrpt::opengl::NICE, 0);

	win.setCameraElevationDeg( 25.0f );
	//win.setCameraProjective(false);

    //CDisplayWindowPlots		winEKF("Tracking - Extended Kalman Filter",450,400);


	//winEKF.setPos(10,10);


	//winEKF.axis(-2,20,-10,10); winEKF.axis_equal();

		// Create EKF
	// ----------------------
	CRangeBearing 	EKF;
	EKF.KF_options.method = kfEKFNaive;

	EKF.KF_options.verbose = true;
	EKF.KF_options.enable_profiler = true;


	bool end = false;
	CTicTac  timer;
	timer.Tic();

	float x=VEHICLE_INITIAL_X,y=VEHICLE_INITIAL_Y,phi=DEG2RAD(-180),v=VEHICLE_INITIAL_V,w=VEHICLE_INITIAL_W;
	//float initial_front_wall;
	float dy=0, old_y=0;
	float dx=0, old_x=0;
	float  t=0;
	float sonarposition = 0, sonarpositionright = 0;

	int counter = 0;
	ofstream EKFEstimate;
    EKFEstimate.open ("ekf.csv");
    EKFEstimate << "y, x, front_sonar, right wall, dy, dx, initial_front_wall_distance, initial_right_wall_distance, EKF_X, EKF_Y" << endl;

	while (!end && win.isOpen() && !p.quit.get())
	{
        const double t = timer.Tac();

		// Move the scene:
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();

		//x+=v*DELTA_TIME*(cos(phi)-sin(phi));
		//y+=v*DELTA_TIME*(sin(phi)+cos(phi));

        //x=0;
		//y=p.currentOdo.get().y();
		 x = p.currentOdo.get().x();
		 y = p.currentOdo.get().y();

        /*if (p.goRight.get()==true) {
            x = p.currentOdo.get().x();
            //y = 0;
            dx = x - old_x;
            cout << "************************Going Right************************\n";
        }
        else if (p.goForward.get()==true) {
            cout << "************************Going Forward************************\n";
            y = p.currentOdo.get().y();
            //x = 0;
            dy = y - old_y;
        }*/

        dx = x - old_x;
        dy = y - old_y;

		 //


		phi+=w*DELTA_TIME;

		v+=1.0f*DELTA_TIME*cos(t);
		w-=0.1f*DELTA_TIME*sin(t);



        /*
		// Simulate noisy observation:
		float realBearing = atan2( y,x );
		float obsBearing = realBearing  + BEARING_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized();
		//printf("Real/Simulated bearing: %.03f / %.03f deg\n", RAD2DEG(realBearing), RAD2DEG(obsBearing) );
		win.addTextMessage(0.01, 0.93, format("Real/Simulated bearing: %.03f / %.03f deg\n", RAD2DEG(realBearing), RAD2DEG(obsBearing) ), TColorf(0,0,1), "mono", 9, mrpt::opengl::NICE,10);

		float realRange = sqrt(square(x)+square(y));
		float obsRange = max(0.0, realRange  + RANGE_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized() );
		//printf("Real/Simulated range: %.03f / %.03f \n", realRange, obsRange );
		win.addTextMessage(0.01, 0.9, format("Real/Simulated range: %.03f / %.03f \n", realRange, obsRange ), TColorf(0,0,1), "mono", 9, mrpt::opengl::NICE,12);
        */

		//EKF.doProcess(DELTA_TIME,obsRange, obsBearing);

		float sonar12;
		float sonar10;
		sonar12 = p.front_wall.get();
		sonar10 = p.right_wall.get();

//jkw		EKF.doProcess(sonar12, sonar10, dy, dx);

		/*EKF.getProfiler().enter("PF:complete_step");
		PF.executeOn(particles, NULL,&SF);  // Process in the PF
		EKF.getProfiler().leave("PF:complete_step");*/

		// Show EKF state:
		CRangeBearing::KFVector EKF_xkk;
		CRangeBearing::KFMatrix EKF_pkk;

		EKF.getState( EKF_xkk, EKF_pkk ); //Should be editted?

		//printf("Real: x:%.03f  y=%.03f heading=%.03f v=%.03f w=%.03f\n",x,y,phi,v,w);
		//cout << format("KF parameters: x=%.03f, y=%.03f, phi=%.03f, v=%.03f, w=%.03f, t=%.03f", x,y,phi,v,w, t) << endl;
		//cout << format("y=%.03f, front_sonar=%.03f, dy=%.03f, initial front wall distance=%.03f", y, p.front_wall.get(), dy, initial_front_wall) << endl;
		//cout << format("y=%.03f, x=%.03f, front_sonar=%.03f, right_sonar=%.03f, dy=%.03f, dx=%.03f, initial front wall distance=%.03f, initial right wall distance=%.03f", y, x, p.front_wall.get(), p.right_wall.get(), dy, dx, initial_front_wall, initial_right_wall) << endl;
		//cout << "EKF: " << EKF_xkk << endl; // and this I think this should be for x and y
		//cout << "EKF_Y: " << EKF_xkk[0]<<",   EKF_X: "<<EKF_xkk[1] << endl; //and this I think this should be for x and y. I editted them though!
		EKFEstimate << format("%.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f, %.03f", y, x, p.front_wall.get(), p.right_wall.get(), dy, dx, initial_front_wall, initial_right_wall) << EKF_xkk[0]<< EKF_xkk[1] << endl;



		old_y = y; // replace old y with current y
		old_x = x;

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

        CPose2D kalman;
        kalman.y(EKF_xkk[0]);
        kalman.x(EKF_xkk[1]);
        kalman.phi(p.currentOdo.get().phi());
		p.currentKF.set(kalman);

		sonarposition = (initial_front_wall - p.front_wall.get());
		sonarpositionright = (initial_right_wall - p.right_wall.get());

		CPose2D sonarpos_;
        sonarpos_.y(sonarposition);
        sonarpos_.x(sonarpositionright);
        sonarpos_.phi(p.currentOdo.get().phi());
		p.currentSonar.set(sonarpos_);

		// Change pose of robot in display
		opengl::CRenderizablePtr obj1 = theScene->getByName("robot");
		obj1->setPose( p.currentKF.get() ); //.x() , p.currentOdo.get().y() , 0 );

		// Change pose of robot based on encoder in display
		opengl::CRenderizablePtr obj2 = theScene->getByName("encoder");
		obj2->setPose( p.currentOdo.get() ); //.x() , p.currentOdo.get().y() , 0 );
		//cout << format("x=%.03f, y=%.03f, phi=%.03f", p.currentOdo.get().x(), p.currentOdo.get().y(), p.currentOdo.get().phi()) << endl;

		// Change pose of robot based on sonar in display
		opengl::CRenderizablePtr obj3 = theScene->getByName("sonarpos");

		obj3->setPose( p.currentSonar.get() ); //.x() , p.currentOdo.get().y() , 0 );

		// Change location of target marker
		opengl::CRenderizablePtr obj4 = theScene->getByName("target");
		obj4->setLocation(p.targetOdo.get().x() , p.targetOdo.get().y() , 0 );

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

		// Put new sonars in the display
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

		// Put new kinect ranges in the display
		{
			// make sure that we have new kinect reading
			CObservation2DRangeScanPtr obs_Ptr = p.new_obs.get();
			CObservation2DRangeScan* obs_2d = obs_Ptr.pointer();

			// make sure that we have new kinect reading
			if (obs_2d != NULL)
			{
			    //cout << "obs_2d IS NOT NULL!\n";
				obs_2d->truncateByDistanceAndAngle(kinectMinTruncateDistance,5);
				opengl::CRenderizablePtr obj4 = theScene->getByName( "kinect" );
				if (obj4 != NULL) ( theScene->removeObject(obj4) );
				opengl::CPlanarLaserScanPtr kinect_scan = opengl::CPlanarLaserScan::Create();
				kinect_scan->setScan( *obs_2d );
				//kinect_scan->setPose(p.currentOdo.get() );
				kinect_scan->setPose(p.currentKF.get() );
				kinect_scan->setName( "kinect" );
				kinect_scan->setColor(1,0,0);
				theScene->insert( kinect_scan );
			} else {
			    //cout << "obs_2d IS NULL!\n";
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

		// Get key press events while 3D window is active.
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

				// Note: changing camera pointing to point is temporary because the
                // pointing to point is changed to current robot odometry each loop.
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

				// clear  / do not clear previous pdf before adding new pdf to display
				case 'p':
					p.displayClearOldPdf.set(true);
					break;

				case 'P':
					p.displayClearOldPdf.set(false);
					break;

				// clear  / do not clear previous sonar readings before adding new pdf to display
				case 's':
					p.displayClearOldSonar.set(true);
					break;

				case 'S':
					p.displayClearOldSonar.set(false);
					break;

				// stops current path finding navigation
				case ' ':
					p.stop.set(true);
					break;
			}		// end case

		}

	}

	EKFEstimate.close();

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

CRangeBearing::CRangeBearing()
{
	//KF_options.method = kfEKFNaive;
	KF_options.method = kfEKFAlaDavison;

	// INIT KF STATE
	/*m_xkk.resize(4,0);	// State: (x,y,heading,v,w)
	m_xkk[0]= VEHICLE_INITIAL_X;
	m_xkk[1]= VEHICLE_INITIAL_Y;
	m_xkk[2]=-VEHICLE_INITIAL_V;
	m_xkk[3]=0;
	*/
	m_xkk.resize(2,0);
	m_xkk[0]= VEHICLE_INITIAL_Y; // for y
	m_xkk[1]= VEHICLE_INITIAL_X; // for x

	// Initial cov:  Large uncertainty
	/*m_pkk.setSize(4,4);
	m_pkk.unit();
	m_pkk(0,0)=
	m_pkk(1,1)= square( 5.0f );
	m_pkk(2,2)=
	m_pkk(3,3)= square( 1.0f );*/
	m_pkk.setSize(2,2);
	m_pkk.unit();
	m_pkk(0,0)=square( 5.0f );
	m_pkk(1,1)=square( 5.0f );
}

CRangeBearing::~CRangeBearing()
{

}


/*void  CRangeBearing::doProcess( double DeltaTime, double observationRange, double observationBearing )
{
	m_deltaTime = (float)DeltaTime;
	m_obsBearing = (float)observationBearing;
	m_obsRange = (float) observationRange;

	runOneKalmanIteration();
}*/

void CRangeBearing::doProcess( double sonar12, double sonar10, double dy , double dx) {

    m_sonar12 = (float)sonar12;
    m_sonar10 = (float)sonar10;
    m_dy = (float)dy;
    m_dx = (float)dx;

    runOneKalmanIteration();
}


/** Must return the action vector u.
  * \param out_u The action vector which will be passed to OnTransitionModel
  */
void CRangeBearing::OnGetAction( KFArray_ACT &u ) const
{
	u[0] = 1;
}

/** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
  * \param in_u The vector returned by OnGetAction.
  * \param inout_x At input has \f$ \hat{x}_{k-1|k-1} \f$, at output must have \f$ \hat{x}_{k|k-1} \f$.
  * \param out_skip Set this to true if for some reason you want to skip the prediction step (to do not modify either the vector or the covariance). Default:false
  */
void CRangeBearing::OnTransitionModel(
	const KFArray_ACT &in_u,
	KFArray_VEH       &inout_x,
	bool &out_skipPrediction
	) const
{
	// in_u[0] : Delta time
	// in_out_x: [0]:x  [1]:y  [2]:vx  [3]: vy
	//inout_x[0] += in_u[0] * inout_x[2];
	//inout_x[1] += in_u[0] * inout_x[3];

	inout_x[0] += in_u[0]*(m_dy); // changed by omar, + is removed look at the paper! FOR Y
	inout_x[1] += in_u[0]*(m_dx); // For X
	//This should  be:       inout_x[0] = y that comes from the encoder. as the state equals y from encoder.

}

/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
  * \param out_F Must return the Jacobian.
  *  The returned matrix must be \f$N \times N\f$ with N being either the size of the whole state vector or get_vehicle_size().
  */
void CRangeBearing::OnTransitionJacobian(KFMatrix_VxV  &F) const
{
	F.unit();

	//F(0,2) = m_deltaTime;
	//F(1,3) = m_deltaTime;
	F(0,0) = 1; //changed by omar // THIS IS the A matrix
	F(1,1) = 1;
}

/** Implements the transition noise covariance \f$ Q_k \f$
  * \param out_Q Must return the covariance matrix.
  *  The returned matrix must be of the same size than the jacobian from OnTransitionJacobian
  */
void CRangeBearing::OnTransitionNoise(KFMatrix_VxV &Q) const
{
	/*Q(0,0) =
	Q(1,1) = square( TRANSITION_MODEL_STD_XY );
	Q(2,2) =
	Q(3,3) = square( TRANSITION_MODEL_STD_VXY );
	*/
	Q(0,0) = 14; //changed by omar
	Q(1,1) = 14;
}

/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
* \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
* \note Upon call, it can be assumed that the previous contents of out_R are all zeros.
*/
void CRangeBearing::OnGetObservationNoise(KFMatrix_OxO &R) const
{
	/*R(0,0) = square( BEARING_SENSOR_NOISE_STD );
	R(1,1) = square( RANGE_SENSOR_NOISE_STD );
	*/
	R(0,0) = 12; // changed by omar
	R(1,1) = 12;
}

void CRangeBearing::OnGetObservationsAndDataAssociation(
	vector_KFArray_OBS			&out_z,
	mrpt::vector_int            &out_data_association,
	const vector_KFArray_OBS	&in_all_predictions,
	const KFMatrix              &in_S,
	const vector_size_t         &in_lm_indices_in_S,
	const KFMatrix_OxO          &in_R
	)
{
	//out_z.resize(1);
	//out_z[0][0] = m_obsBearing;
	//out_z[0][1] = m_obsRange;
	out_z.resize(1);
	out_z[0][0] = m_sonar12;
	out_z[0][1] = m_sonar10;

	out_data_association.clear(); // Not used
}


/** Implements the observation prediction \f$ h_i(x) \f$.
  * \param idx_landmark_to_predict The indices of the landmarks in the map whose predictions are expected as output. For non SLAM-like problems, this input value is undefined and the application should just generate one observation for the given problem.
  * \param out_predictions The predicted observations.
  */
void CRangeBearing::OnObservationModel(
	const vector_size_t       &idx_landmarks_to_predict,
	vector_KFArray_OBS	&out_predictions
	) const
{
	// predicted bearing:
	/*kftype x = m_xkk[0];
	kftype y = m_xkk[1];

	kftype h_bear = atan2(y,x);
	kftype h_range = sqrt(square(x)+square(y));

	// idx_landmarks_to_predict is ignored in NON-SLAM problems
	out_predictions.resize(1);
	out_predictions[0][0] = h_bear;
	out_predictions[0][1] = h_range;
	*/
	kftype y = m_xkk[0]; //OMAR, this is the state, and in our case the state is the encoder measurement.
	kftype x = m_xkk[1];

	kftype h_distance = initial_front_wall - y;
	kftype h_distancex = initial_right_wall - x;

	out_predictions.resize(1);
	out_predictions[0][0] = h_distance;
	out_predictions[0][1] = h_distancex;
}

/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
  * \param idx_landmark_to_predict The index of the landmark in the map whose prediction is expected as output. For non SLAM-like problems, this will be zero and the expected output is for the whole state vector.
  * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
  * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
  */
void CRangeBearing::OnObservationJacobians(
	const size_t &idx_landmark_to_predict,
	KFMatrix_OxV &Hx,
	KFMatrix_OxF &Hy
	) const
{
	// predicted bearing:
	/*kftype x = m_xkk[0];
	kftype y = m_xkk[1];

	Hx.zeros();
	Hx(0,0) = -y/(square(x)+square(y));
	Hx(0,1) = 1/(x*(1+square(y/x)));

	Hx(1,0) = x/sqrt(square(x)+square(y));
	Hx(1,1) = y/sqrt(square(x)+square(y));*/

	// Hy: Not used

	Hx.zeros();
	Hx(0,0) = -1; //changed by omar
	Hx(1,1) = -1;
}

/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
  */
void CRangeBearing::OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
{
	A -= B;
	math::wrapToPiInPlace(A[0]); // The angular component
}


//Based on sensor readings determines the appropriate file to parse for appropriate sequences
//Parses first line of base movements of the file
//Commands robot to execute a certain base movement based on chars found in sequence line
int parsesequence(TThreadRobotParam &thrPar)
{
 	float sensed_distance [] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	//char const* const fileName = "seq_1obj(5-7_0-2)_strt(0_0)_end(10_0)_20x20grid.txt";
	const char * fileName = NULL;

	//Get latest ultrasonic sensor values and save them in sensed_distance array
	/*for (CObservationRange::const_iterator i=obs.sensedData.begin();i!=obs.sensedData.end();++i)
	{
		sensed_distance[sensor] = i->sensedDistance;
		sensor ++;
	}*/

	//need to set i, j, x, y accordingly for each condition
	//i and j particular sonars
	// x, y are the values read by the sonars i and j respectively

	float front, left, right, front_left, front_right;
	front = thrPar.front_wall.get();
	left = thrPar.left_wall.get();
	right = thrPar.right_wall.get();
	front_left = thrPar.front_left.get();
	front_right = thrPar.front_right.get();

	float front_threshold = 1.5;
	float left_threshold = 1.5;
	float right_threshold = 1.5;
	float limit = 1.0;

    cout << "****************************** IN PARSESEQUENCE ************************\n";

	//if(front <= 26)
        //fileName = "seq_1obj(5-7_0-2)_strt(0_0)_end(10_0)_20x20grid.txt";
	if(front_left <= front_threshold && front_right < right_threshold) {
	    cout << "CASE 1: ";
	    // Straightahead
        fileName = "straightahead.txt";
        cout << "[front: " << front << ", left: " << left << ", right: " << right << "]\n";
	}

	else if(front_left > front_right){
        cout << "CASE 2: ";
        // Two objects, right closer
        fileName = "2obj_rightcloser.txt";
        cout << "[front: " << front << ", left: " << left << ", right: " << right << "]\n";
	}
    else if(front_right > front_left){
        cout << "CASE 3: ";
        // Two objects, left closer
        fileName = "2obj_leftcloser.txt";
        cout << "[front: " << front << ", left: " << left << ", right: " << right << "]\n";
	}
    else if(front_right > front_left && right>= right_threshold){
        cout << "CASE 4: ";
        // One object left side
        fileName = "1obj_leftside.txt";
        cout << "[front: " << front << ", left: " << left << ", right: " << right << "]\n";
	}
    else if(front_left > front_right && left >= left_threshold){
        cout << "CASE 5: ";
        // One object right side
        fileName = "1obj_rightside.txt";
        cout << "[front: " << front << ", left: " << left << ", right: " << right << "]\n";
	}


    else {
        cout << "STOPPPPPP\n";
        return 1;   // ERROR!
    }

    cout << "[front: " << front << ", left: " << left << ", right: " << right << "]\n";

	//open the appropriate file for obstacle situation
    FILE* file = fopen(fileName, "r");
    char line[256];

    if (file != NULL) {
        cout << "****************************** SUCCESS OPENING FILE ******************************\n";
        //read the very first line of sequences
        fgets(line, sizeof(line), file);
        for ( int i = 1; i <= sizeof(line); ++i)
        {
            //if return character found break out of loop. you have parsed until the end of the line
            cout << "****************************** LOOP ******************************\n";
            /*if(line[i]= '\n') {
                    cout << "****************************** LINE BREAK ******************************\n";
                    break;
            }*/
            if (front < limit ) {
                cout << "****************************** OBSTACLE!!! STOPPNIG ... ******************************\n";
                sequence('s', thrPar);
                break;
            }
            //for each sequence execute a certain base movement command
            //sequence(line[i], thrPar, sonars);
            sequence(line[i], thrPar);
            cout << "RUNNING SEQUENCE: " << line[i] << endl;
        }
    }
    else {
        cout << "****************************** ERROR OPENING FILE ******************************\n";
    }

    fclose(file);

    return 0;
}

//gets sequence character and executes appropriate action
void sequence(char sequencechar, TThreadRobotParam &thrPar)
{
    cout << "****************************** SEQUENCE: " << sequencechar << " ******************************\n";
	//execute the movement for the updated sequence char
	if( sequencechar == 'l')
	{
	    cout << "****************************** LEFT ******************************\n";
	//move left
	setVelocities( 0, -1, thrPar );
	//delay to turn 90 degrees
	sleep(11000);
	setVelocities( 1, 0, thrPar );
	sleep(8000);
	setVelocities( 0, 0, thrPar );
	}
	else if( sequencechar == 'r')
	{
	    cout << "****************************** RIGHT ******************************\n";
	//move right
	setVelocities( 0, 1, thrPar );
	sleep(11000);
	setVelocities( 1, 0, thrPar );
	sleep(8000);
	setVelocities( 0, 0, thrPar );
	}
	else if( sequencechar == 'f')
	{
	    cout << "****************************** FORWARD ******************************\n";
	//go forward
	setVelocities( 1, 0, thrPar );
	sleep(8000);
	setVelocities( 0, 0, thrPar );
	}
	else if( sequencechar == 'b')
	{
	    cout << "****************************** BACKWARD ******************************\n";
	//go backwards
	setVelocities( -1, 0, thrPar );
	sleep(8000);
	setVelocities( 0, 0, thrPar );
	}
	//stop the robot
	else if( sequencechar == 's' || sequencechar == '\n')
	{
    setVelocities( 0, 0, thrPar );
	sleep(5000);

	}

}

