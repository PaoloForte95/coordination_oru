package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Random;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner.PLANNING_ALGORITHM;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment;
import se.oru.coordination.coordination_oru.taskassignment.Robot;
import se.oru.coordination.coordination_oru.taskassignment.Task;
import com.google.ortools.linearsolver.*;


@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TaskAssignmentOrebroWarehouseWithDistance {
	//load library used for optimization
	 static {
		    System.loadLibrary("jniortools");
		  }
	public static void main(String[] args) throws InterruptedException {

		//Maximum acceleration/deceleration and speed for all robots
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		
		//Create a coordinator with interfaces to robots
		//in the built-in 2D simulator
		//final TimedTrajectoryEnvelopeCoordinatorSimulation tec = new TimedTrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
		
		//Define a network with uncertainties (see Mannucci et al., 2019)
		NetworkConfiguration.setDelays(0, 0);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.0;
		
		//Tell the coordinator
		// (1) what is known about the communication channel, and
		// (2) the accepted probability of constraint violation
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);
		
		//Avoid deadlocks via global re-ordering
		tec.setBreakDeadlocks(true, false, false);
		
		//Need to instantiate the fleetmaster interface
		//tec.instantiateFleetMaster(0.1, true);
		
		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//Start the thread that revises precedences at every period
		tec.startInference();
		
		//set the map yalm file
		String yamlFile = new String("maps/basement.yaml");
		
		//Start a visualization (will open a new browser tab)
		BrowserVisualization viz = new BrowserVisualization();
		viz.setMap(yamlFile);
		viz.setInitialTransform(13, 6.1, 6.8);
		tec.setVisualization(viz);
		tec.setBreakDeadlocks(true, false, false);
		//Robot IDs can be non-sequential (but must be unique)
		double xl = 1.0;
		double yl = .5;
		Coordinate footprint1 = new Coordinate(-xl,yl);
		Coordinate footprint2 = new Coordinate(xl,yl);
		Coordinate footprint3 = new Coordinate(xl,-yl);
		Coordinate footprint4 = new Coordinate(-xl,-yl);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		Coordinate[] fp2 = new Coordinate[] {
				new Coordinate(-0.7,0.7),
				new Coordinate(0.7,0.7),
				new Coordinate(0.7,-0.7),
				new Coordinate(-0.7,-0.7),
		};
		
		Robot robot1 = new Robot(1,1,tec.getDefaultFootprint());
		Robot robot2 = new Robot(2,1,tec.getDefaultFootprint());
		Robot robot3 = new Robot(3,1,tec.getDefaultFootprint());
		Robot robot4 = new Robot(4,1,tec.getDefaultFootprint());
		Robot robot5 = new Robot(5,1,tec.getDefaultFootprint());
		Robot robot6 = new Robot(6,1,tec.getDefaultFootprint());
		Robot robot7 = new Robot(7,1,tec.getDefaultFootprint());
	   
		//tec.setFootprint(2, fp2);
		//tec.setFootprint(6, fp2);
		//Define start and goal poses for each robot
		Pose startPoseRobot1 = new Pose(33.0,5.0,Math.PI);
		Pose startPoseRobot2 = new Pose(3.0,28.0,0.0);
		Pose startPoseRobot3 = new Pose(3.0,20.0,0.0);
		Pose startPoseRobot4 = new Pose(3.0,25.0,0.0);
		Pose startPoseRobot5 = new Pose(8.0,2.8,Math.PI/2);	
		Pose startPoseRobot6 = new Pose(11.0,2.8,Math.PI/2);	
		Pose startPoseRobot7 = new Pose(20.0,2.8,Math.PI/2);	

		
		
		
		Pose startPoseGoal1 = new Pose(7.0,7.0,Math.PI/2);	
		Pose startPoseGoal2 = new Pose(13.0,20.0,0.0);
		Pose startPoseGoal3 = new Pose(13.0,24.25,0.0);
		Pose startPoseGoal4 = new Pose(25.0,5.0,0.0);	
		Pose startPoseGoal5 = new Pose(18.0,15.0,0.0);
		Pose startPoseGoal6 = new Pose(11.0,5.0,Math.PI/2);	
		Pose startPoseGoal7 = new Pose(20.0,11.0,0.0);
		
		Pose goalPoseRobot1 = new Pose(6.0,15.5,Math.PI/2);
		Pose goalPoseRobot2 = new Pose(3.0,23.0,0.0);
		Pose goalPoseRobot3 = new Pose(20.0,24.25,0.0);
		Pose goalPoseRobot4 = new Pose(30.5,7.0,Math.PI/2);
		Pose goalPoseRobot5 = new Pose(25.0,15.0,0.0);
		Pose goalPoseRobot6 = new Pose(3.0,26.0,0.0);
		Pose goalPoseRobot7 = new Pose(24.0,11.0,0.0);
		

		///////////////////////////////////////////////
		
		//Set the default motion planner
		ReedsSheppCarPlanner rsp1 = new ReedsSheppCarPlanner(PLANNING_ALGORITHM.RRTstar);
		rsp1.setRadius(0.1);
		rsp1.setTurningRadius(4.0);
		rsp1.setFootprint(tec.getDefaultFootprint());
		rsp1.setDistanceBetweenPathPoints(0.5);
		rsp1.setMap(yamlFile);
		rsp1.setPlanningTimeInSecs(10);
		
		tec.addRobot(robot1, startPoseRobot1);
		tec.addRobot(robot2, startPoseRobot2);
		tec.addRobot(robot3, startPoseRobot3);
		tec.addRobot(robot4, startPoseRobot4);
		tec.addRobot(robot5, startPoseRobot5);
		tec.addRobot(robot6, startPoseRobot6);
		tec.addRobot(robot7, startPoseRobot7);
		
		//Primo set di tasks
		
		Task task1 = new Task(1,startPoseGoal1,goalPoseRobot1,1);
		Task task2 = new Task(2,startPoseGoal2,goalPoseRobot2,1);
		Task task3 = new Task(3,startPoseGoal3,goalPoseRobot3,1);
		Task task4 = new Task(4,startPoseGoal4,goalPoseRobot4,1);
		Task task5 = new Task(5,startPoseGoal5,goalPoseRobot5,1);
		Task task6 = new Task(6,startPoseGoal6,goalPoseRobot6,1);
		Task task7 = new Task(7,startPoseGoal7,goalPoseRobot7,1);
				
		
		
		
		
		int a = 1;
		//FOLDER NUMBER 
		int b = 2;
		tec.setTestNumber(a);
		tec.setFolderNumber(b);
		
		TaskAssignment assignmentProblem = new TaskAssignment();
		
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);
		assignmentProblem.addTask(task4);
		assignmentProblem.addTask(task5);

		assignmentProblem.setFleetVisualization(viz);
		int numPaths = 1;
		
		for (int robotID : tec.getIdleRobots()) {
			
			//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
			ReedsSheppCarPlanner rsp = (ReedsSheppCarPlanner) rsp1.getCopy(false);
			tec.setMotionPlanner(robotID, rsp);
		}
		

	    ///////////////////////// //////////////////////////////
		//Solve the problem to find some feasible solution
		assignmentProblem.setTimeOutinMin(6.5);
		double alpha = 0.5;
		double [][][]optimalAllocation = {{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
		};
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario1");
		assignmentProblem.LoadScenarioAllocation(optimalAllocation);
		assignmentProblem.setmaxNumPaths(numPaths);
		assignmentProblem.setFleetVisualization(viz);
		assignmentProblem.setLinearWeight(alpha);
		//tec.setFakeCoordinator(true);
		//assignmentProblem.setCostFunctionsWeight(0.8, 0.1, 0.1);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		
		assignmentProblem.startTaskAssignment(tec);
		//assignmentProblem.startTaskAssignmentGreedyAlgorithm(tec);
		//assignmentProblem.startTaskAssignmentLocalSearchAlgorithm(tec, -1);
		
		
		//New tasks real
		//SET 2
		//Second set of tasks
		//-> 4 more tasks ( 3 of type 1 and 1 of type 2)
		Pose startPoseGoal1Set2 = new Pose(9.0,24.0,0.0);
		Pose goalPoseRobot1Set2 = new Pose(15.0,28.0,Math.PI/2);
		Task task1Set2 = new Task(8,startPoseGoal1Set2,goalPoseRobot1Set2,1);
		///////////////////
		Pose startPoseGoal2Set2 = new Pose(22.0,5.0,0.0);
		Pose goalPoseRobot2Set2 = new Pose(33.5,4.0,0.0);
		Task task2Set2 = new Task(9,startPoseGoal2Set2,goalPoseRobot2Set2,1);
		///////////////////
		Pose startPoseGoal3Set2 = new Pose(22.0,3.0,0.0);
		Pose goalPoseRobot3Set2 = new Pose(33.7,2.0,0.0);
		Task task3Set2 = new Task(10,startPoseGoal3Set2,goalPoseRobot3Set2,1);
		///////////////////
		Pose startPoseGoal4Set2 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot4Set2 = new Pose(2.8,6.5,Math.PI);
		Task task4Set2 = new Task(11,startPoseGoal4Set2,goalPoseRobot4Set2,1);
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		//Third set of tasks
		//-> 5 more tasks (4 of type 1 and 1 of type 2)
		Pose startPoseGoal1Set3 = new Pose(9.5,33.5,0.0);
		Pose goalPoseRobot1Set3 = new Pose(43.5,30.6,0.0);
		Task task1Set3 = new Task(12,startPoseGoal1Set3,goalPoseRobot1Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal2Set3 = new Pose(11.0,12.0,-Math.PI/2);
		Pose goalPoseRobot2Set3 = new Pose(2.4,3.0,Math.PI);
		Task task2Set3 = new Task(13,startPoseGoal2Set3,goalPoseRobot2Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal3Set3 = new Pose(18.0,8.0,-Math.PI/2);
		Pose goalPoseRobot3Set3 = new Pose(17.0,2.8,-Math.PI/2);
		Task task3Set3 = new Task(14,startPoseGoal3Set3,goalPoseRobot3Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal4Set3 = new Pose(6.0,28.0,0.0);
		Pose goalPoseRobot4Set3 = new Pose(3.0,28.0,0.0);
		Task task4Set3 = new Task(15,startPoseGoal4Set3,goalPoseRobot4Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal5Set3 = new Pose(23.5,33.5,0.0);
		Pose goalPoseRobot5Set3 = new Pose(43.5,33.6,0.0);
		Task task5Set3 = new Task(16,startPoseGoal5Set3,goalPoseRobot5Set3,1);
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		
		
		
		
		
		
		//Forth set of tasks
		//-> 4 more tasks (4 of type 1 and 0 of type 2)
		Pose startPoseGoal1Set4 = new Pose(18.0,15.0,0.0);
		Pose goalPoseRobot1Set4 = new Pose(25.0,15.0,0.0);
		Task task1Set4 = new Task(17,startPoseGoal1Set4,goalPoseRobot1Set4,1);
		//////////////////////////////////////
		Pose startPoseGoal2Set4 = new Pose(15.0,24.0,0.0);
		Pose goalPoseRobot2Set4 = new Pose(20.0,24.5,0.0);
		Task task2Set4 = new Task(18,startPoseGoal2Set4,goalPoseRobot2Set4,1);
		//////////////////////////////////////
		Pose startPoseGoal3Set4 = new Pose(20.0,11.0,0.0);
		Pose goalPoseRobot3Set4 = new Pose(24.0,11.0,0.0);
		Task task3Set4 = new Task(19,startPoseGoal3Set4,goalPoseRobot3Set4,1);
		//////////////////////////////////////
		Pose startPoseGoal4Set4 = new Pose(9.0,22.0,Math.PI/2);
		Pose goalPoseRobot4Set4 = new Pose(6.0,33.5,Math.PI/2);
		Task task4Set4 = new Task(20,startPoseGoal4Set4,goalPoseRobot4Set4,1);
		//////////////////////////////////////
		//Fifth set of tasks
		//->  7 more tasks (5 of type 1 and 2 of type 2)
		Pose startPoseGoal1Set5 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot1Set5 = new Pose(6.0,10.0,Math.PI);
		Task task1Set5 = new Task(21,startPoseGoal1Set5,goalPoseRobot1Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal2Set5 = new Pose(2.0,16.0,-Math.PI/2);
		Pose goalPoseRobot2Set5 = new Pose(2.0,11.0,-Math.PI/2);
		Task task2Set5 = new Task(22,startPoseGoal2Set5,goalPoseRobot2Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal3Set5 = new Pose(9.0,23.0,Math.PI);
		Pose goalPoseRobot3Set5 = new Pose(5.0,20.0,-Math.PI/2);
		Task task3Set5 = new Task(23,startPoseGoal3Set5,goalPoseRobot3Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal4Set5 = new Pose(18.0,19.0,0.0);
		Pose goalPoseRobot4Set5 = new Pose(23.0,19.0,0.0);
		Task task4Set5 = new Task(24,startPoseGoal4Set5,goalPoseRobot4Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal5Set5 = new Pose(20.0,5.0,0.0);
		Pose goalPoseRobot5Set5 = new Pose(33.5,7.5,0.0);
		Task task5Set5 = new Task(25,startPoseGoal5Set5,goalPoseRobot5Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal6Set5 = new Pose(20.0,4.0,0.0);
		Pose goalPoseRobot6Set5 = new Pose(25.5,3.0,Math.PI/2);
		Task task6Set5 = new Task(26,startPoseGoal6Set5,goalPoseRobot6Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal7Set5 = new Pose(6.0,28.0,0.0);
		Pose goalPoseRobot7Set5 = new Pose(3.0,33.5,Math.PI/2);
		Task task7Set5 = new Task(27,startPoseGoal7Set5,goalPoseRobot7Set5,1);
		////////////////////////////////////////////////////
		//////////////////////////////////////
		//Sixth set of tasks
		//->  7 more tasks (5 of type 1 and 2 of type 2)
		Pose startPoseGoal1Set6 = new Pose(18.0,33.0,0.0);
		Pose goalPoseRobot1Set6 = new Pose(43.5,27.6,0.0);
		Task task1Set6 = new Task(28,startPoseGoal1Set6,goalPoseRobot1Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set6 = new Pose(19.0,33.0,0.0);
		Pose goalPoseRobot2Set6 = new Pose(43.5,24.6,0.0);
		Task task2Set6 = new Task(29,startPoseGoal2Set6,goalPoseRobot2Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal3Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot3Set6 = new Pose(43.5,21.6,0.0);
		Task task3Set6 = new Task(30,startPoseGoal3Set6,goalPoseRobot3Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal4Set6 = new Pose(22.0,33.0,0.0);
		Pose goalPoseRobot4Set6 = new Pose(43.5,18.6,0.0);
		Task task4Set6 = new Task(31,startPoseGoal4Set6,goalPoseRobot4Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal5Set6 = new Pose(21.0,33.0,0.0);
		Pose goalPoseRobot5Set6 = new Pose(43.5,15.6,0.0);
		Task task5Set6 = new Task(32,startPoseGoal5Set6,goalPoseRobot5Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal6Set6 = new Pose(23.0,33.0,0.0);
		Pose goalPoseRobot6Set6 = new Pose(43.5,12.6,0.0);
		Task task6Set6 = new Task(33,startPoseGoal6Set6,goalPoseRobot6Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal7Set6 = new Pose(11.0,5.0,Math.PI/2);
		Pose goalPoseRobot7Set6 = new Pose(3.0,26.0,0.0);
		Task task7Set6 = new Task(34,startPoseGoal7Set6,goalPoseRobot7Set6,1);
		/////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////
		//->  7 more tasks (5 of type 1 and 2 of type 2)
		Pose startPoseGoal1Set10 = new Pose(22.0,33.0,0.0);
		Pose goalPoseRobot1Set10 = new Pose(43.5,27.6,0.0);
		Task task1Set10 = new Task(35,startPoseGoal1Set10,goalPoseRobot1Set10,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set10 = new Pose(24.0,33.0,0.0);
		Pose goalPoseRobot2Set10 = new Pose(43.5,24.6,0.0);
		Task task2Set10 = new Task(36,startPoseGoal2Set10,goalPoseRobot2Set10,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal3Set10 = new Pose(23.0,33.0,0.0);
		Pose goalPoseRobot3Set10 = new Pose(43.5,21.6,0.0);
		Task task3Set10 = new Task(37,startPoseGoal3Set10,goalPoseRobot3Set10,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal4Set10 = new Pose(20.0,32.0,0.0);
		Pose goalPoseRobot4Set10 = new Pose(43.5,18.6,0.0);
		Task task4Set10 = new Task(38,startPoseGoal4Set10,goalPoseRobot4Set10,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal5Set10 = new Pose(18.0,32.0,0.0);
		Pose goalPoseRobot5Set10 = new Pose(43.5,15.6,0.0);
		Task task5Set10 = new Task(39,startPoseGoal5Set10,goalPoseRobot5Set10,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal6Set10 = new Pose(20.0,32.0,0.0);
		Pose goalPoseRobot6Set10 = new Pose(43.5,12.6,0.0);
		Task task6Set10 = new Task(40,startPoseGoal6Set10,goalPoseRobot6Set10,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal7Set10 = new Pose(16.0,32.0,0.0);
		Pose goalPoseRobot7Set10 = new Pose(43.5,9.6,0.0);
		Task task7Set10 = new Task(41,startPoseGoal7Set10,goalPoseRobot7Set10,1);
		
		
		
		//->  7 more tasks (5 of type 1 and 2 of type 2)
		Pose startPoseGoal1Set9 = new Pose(29.0,4.0,Math.PI);
		Pose goalPoseRobot1Set9 = new Pose(33.0,4.0,Math.PI);
		Task task1Set9 = new Task(42,startPoseGoal1Set9,goalPoseRobot1Set9,1);
		////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set9 = new Pose(6.0,28.0,0.0);
		Pose goalPoseRobot2Set9 = new Pose(3.0,28.0,0.0);
		Task task2Set9 = new Task(43,startPoseGoal2Set9,goalPoseRobot2Set9,1);
		///////////////////////////////////////////////////
		Pose startPoseGoal3Set9 = new Pose(6.0,20.0,0.0);
		Pose goalPoseRobot3Set9 = new Pose(3.0,20.0,0.0);
		Task task3Set9 = new Task(44,startPoseGoal3Set9,goalPoseRobot3Set9,1);
		///////////////////////////////////////////////////
		Pose startPoseGoal4Set9 = new Pose(7.0,25.0,0.0);
		Pose goalPoseRobot4Set9 = new Pose(3.0,25.0,0.0);
		Task task4Set9 = new Task(45,startPoseGoal4Set9,goalPoseRobot4Set9,1);
		//////////////////////////////////////////////////////////////////
		////////////////////////GIVE AGAIN THE SAME TASKS TO HAVE 75 TOTAL TASKS///////////////////////////////////////////
		//////
		//SET 11
		Pose startPoseGoal1Set11 = new Pose(11.0,5.0,Math.PI/2);
		Pose goalPoseRobot1Set11 = new Pose(3.0,26.0,0.0);
		Task task1Set11 = new Task(46,startPoseGoal1Set11,goalPoseRobot1Set11,1);
		///////////////////
		Pose startPoseGoal2Set11 = new Pose(20.0,11.0,0.0);
		Pose goalPoseRobot2Set11 = new Pose(24.0,11.0,0.0);
		Task task2Set11 = new Task(47,startPoseGoal2Set11,goalPoseRobot2Set11,1);
		////////////////////////////////////////////
		
		Pose startPoseGoal3Set11 = new Pose(9.0,24.0,0.0);
		Pose goalPoseRobot3Set11 = new Pose(14.5,28.0,Math.PI/2);
		Task task3Set11 = new Task(48,startPoseGoal3Set11,goalPoseRobot3Set11,1);
		///////////////////
		Pose startPoseGoal4Set11 = new Pose(22.0,5.0,0.0);
		Pose goalPoseRobot4Set11 = new Pose(33.5,4.0,0.0);
		Task task4Set11 = new Task(49,startPoseGoal4Set11,goalPoseRobot4Set11,1);
		///////////////////
		Pose startPoseGoal5Set11 = new Pose(22.0,3.0,0.0);
		Pose goalPoseRobot5Set11 = new Pose(33.7,2.0,0.0);
		Task task5Set11 = new Task(50,startPoseGoal5Set11,goalPoseRobot5Set11,1);
		////////////////////////////////////////
		///////////////////////////////////////
		//SET 12 
		///////////////////
		Pose startPoseGoal1Set12 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot1Set12 = new Pose(2.8,6.5,Math.PI);
		Task task1Set12 = new Task(51,startPoseGoal1Set12,goalPoseRobot1Set12,1);
		//////////////////////////////////////////////
		Pose startPoseGoal2Set12 = new Pose(9.5,33.5,0.0);
		Pose goalPoseRobot2Set12 = new Pose(43.5,30.6,0.0);
		Task task2Set12 = new Task(52,startPoseGoal2Set12,goalPoseRobot2Set12,1);
		////////////////////////////////////////////
		Pose startPoseGoal3Set12 = new Pose(11.0,12.0,-Math.PI/2);
		Pose goalPoseRobot3Set12 = new Pose(2.4,3.0,Math.PI);
		Task task3Set12 = new Task(53,startPoseGoal3Set12,goalPoseRobot3Set12,1);
		////////////////////////////////////////////
		Pose startPoseGoal4Set12 = new Pose(18.0,8.0,-Math.PI/2);
		Pose goalPoseRobot4Set12 = new Pose(17.0,2.8,-Math.PI/2);
		Task task4Set12 = new Task(54,startPoseGoal4Set12,goalPoseRobot4Set12,1);
		////////////////////////////////////////////
		Pose startPoseGoal5Set12 = new Pose(6.0,28.0,0.0);
		Pose goalPoseRobot5Set12 = new Pose(3.0,28.0,0.0);
		Task task5Set12 = new Task(55,startPoseGoal5Set12,goalPoseRobot5Set12,1);
		////////////////////////////////////////////
		///////////////////////////////////////////
		//SET 13
		Pose startPoseGoal1Set13 = new Pose(23.5,33.5,0.0);
		Pose goalPoseRobot1Set13 = new Pose(43.5,33.6,0.0);
		Task task1Set13 = new Task(56,startPoseGoal1Set13,goalPoseRobot1Set13,1);
		//////////////////////////////////////////////
		Pose startPoseGoal2Set13 = new Pose(18.0,15.0,0.0);
		Pose goalPoseRobot2Set13 = new Pose(25.0,15.0,0.0);
		Task task2Set13 = new Task(57,startPoseGoal2Set13,goalPoseRobot2Set13,1);
		//////////////////////////////////////
		Pose startPoseGoal3Set13 = new Pose(15.0,24.0,0.0);
		Pose goalPoseRobot3Set13 = new Pose(20.0,24.5,0.0);
		Task task3Set13 = new Task(58,startPoseGoal3Set13,goalPoseRobot3Set13,1);
		//////////////////////////////////////
		Pose startPoseGoal4Set13 = new Pose(20.0,11.0,0.0);
		Pose goalPoseRobot4Set13 = new Pose(24.0,11.0,0.0);
		Task task4Set13 = new Task(59,startPoseGoal4Set13,goalPoseRobot4Set13,1);
		//////////////////////////////////////
		Pose startPoseGoal5Set13 = new Pose(9.0,22.0,Math.PI/2);
		Pose goalPoseRobot5Set13 = new Pose(6.0,33.5,Math.PI/2);
		Task task5Set13 = new Task(60,startPoseGoal5Set13,goalPoseRobot5Set13,1);
		//////////////////////////////////////
		/////////////////////////////////////////////////////////
		//SET 14
		Pose startPoseGoal1Set14 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot1Set14 = new Pose(6.0,10.0,Math.PI);
		Task task1Set14 = new Task(61,startPoseGoal1Set14,goalPoseRobot1Set14,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal2Set14 = new Pose(2.0,16.0,-Math.PI/2);
		Pose goalPoseRobot2Set14 = new Pose(2.0,11.0,-Math.PI/2);
		Task task2Set14 = new Task(62,startPoseGoal2Set14,goalPoseRobot2Set14,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal3Set14 = new Pose(9.0,23.0,Math.PI);
		Pose goalPoseRobot3Set14 = new Pose(5.0,20.0,-Math.PI/2);
		Task task3Set14 = new Task(63,startPoseGoal3Set14,goalPoseRobot3Set14,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal4Set14 = new Pose(18.0,19.0,0.0);
		Pose goalPoseRobot4Set14 = new Pose(23.0,19.0,0.0);
		Task task4Set14 = new Task(64,startPoseGoal4Set14,goalPoseRobot4Set14,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal5Set14 = new Pose(20.0,5.0,0.0);
		Pose goalPoseRobot5Set14 = new Pose(33.5,7.5,0.0);
		Task task5Set14 = new Task(65,startPoseGoal5Set14,goalPoseRobot5Set14,1);
		////////////////////////////////////////////////////
		///////////////////////////////////////////////////
		//SET 15
		Pose startPoseGoal1Set15 = new Pose(20.0,4.0,0.0);
		Pose goalPoseRobot1Set15 = new Pose(25.5,3.0,Math.PI/2);
		Task task1Set15 = new Task(66,startPoseGoal1Set15,goalPoseRobot1Set15,1);
		///////////////////////////////////////
		Pose startPoseGoal2Set15 = new Pose(6.0,28.0,0.0);
		Pose goalPoseRobot2Set15 = new Pose(3.0,33.5,Math.PI/2);
		Task task2Set15 = new Task(67,startPoseGoal2Set15,goalPoseRobot2Set15,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal3Set15 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot3Set15 = new Pose(43.5,27.6,0.0);
		Task task3Set15 = new Task(68,startPoseGoal3Set15,goalPoseRobot3Set15,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal4Set15 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot4Set15 = new Pose(43.5,24.6,0.0);
		Task task4Set15 = new Task(69,startPoseGoal4Set15,goalPoseRobot4Set15,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal5Set15 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot5Set15 = new Pose(43.5,21.6,0.0);
		Task task5Set15 = new Task(70,startPoseGoal5Set15,goalPoseRobot5Set15,1);
		/////////////////////////////////////////////////////////////////////
		////////////////////////////
		//SET 16
		
		Pose startPoseGoal1Set16 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot1Set16 = new Pose(43.5,18.6,0.0);
		Task task1Set16 = new Task(71,startPoseGoal1Set16,goalPoseRobot1Set16,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set16 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot2Set16 = new Pose(43.5,15.6,0.0);
		Task task2Set16 = new Task(72,startPoseGoal2Set16,goalPoseRobot2Set16,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal3Set16 = new Pose(8.0,5.8,Math.PI/2);
		Pose goalPoseRobot3Set16 = new Pose(8.0,2.8,Math.PI/2);	
		Task task3Set16 = new Task(73,startPoseGoal3Set16,goalPoseRobot3Set16,1);
		////////////////////////////////////////////////////////////////////
		Pose startPoseGoal4Set16 = new Pose(11.0,5.8,Math.PI/2);
		Pose goalPoseRobot4Set16 = new Pose(11.0,2.8,Math.PI/2);
		Task task4Set16 = new Task(74,startPoseGoal4Set16,goalPoseRobot4Set16,1);
		///////////////////////////////////////////////////////////////////
		Pose startPoseGoal5Set16 = new Pose(18.0,7.8,Math.PI/2);
		Pose goalPoseRobot5Set16 = new Pose(20.0,2.5,Math.PI/2);
		Task task5Set16 = new Task(75,startPoseGoal5Set16,goalPoseRobot5Set16,1);
		
		
		
		
		
		
		
		
		
		
		
		//TEST ZONE
		Pose startPoseGoal1New = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot1New = new Pose(2.0,5.0,Math.PI);
	
		Robot robot1New = new Robot(8, 1,tec.getDefaultFootprint() );
		Robot robot2New = new Robot(9,2,tec.getDefaultFootprint() );
		Robot robot3New = new Robot(10,1,tec.getDefaultFootprint() );
		Robot robot4New = new Robot(11,1,tec.getDefaultFootprint());
		Robot robot5New = new Robot(12,1,tec.getDefaultFootprint());
		Robot robot6New = new Robot(13,2,fp2);	
		
		//tec.addRobot(robot1New,startPoseGoal1Set3);
		//tec.addRobot(robot2New,goalPoseRobot1Set3);
		//tec.addRobot(robot3New,startPoseGoal1Set7);
		//tec.addRobot(robot4New, startPoseGoal7Set5);
		//tec.addRobot(robot5New, startPoseGoal2Set3);
		//tec.addRobot(robot6New, goalPoseRobot2Set3);
		///////
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(55000); //sys alpha != 1 synchronous real
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(450000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		//Add the second set of tasks
		assignmentProblem.addTask(task6);
		assignmentProblem.addTask(task1Set10);
		assignmentProblem.addTask(task1Set2);
		assignmentProblem.addTask(task2Set2);
		assignmentProblem.addTask(task3Set2);
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario2");
		double [][][]optimalAllocation2 = {{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation2);
		
		
		///Sleep for a while
		
		//Thread.sleep(150000);  //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous real
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(100000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task1Set3);
		assignmentProblem.addTask(task2Set3);
		assignmentProblem.addTask(task3Set3);
		assignmentProblem.addTask(task4Set3);
		assignmentProblem.addTask(task5Set3);
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario3");
		double [][][]optimalAllocation3 = {{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation3);
		
		
		//Thread.sleep(150000); //sys synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous real
		Thread.sleep(150000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(250000);
		//Thread.sleep(500000);
			//Thread.sleep(75000);
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task4Set2);
		assignmentProblem.addTask(task1Set6);
		assignmentProblem.addTask(task2Set6);
		assignmentProblem.addTask(task3Set6);
		assignmentProblem.addTask(task4Set6);
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario4");
		double [][][]optimalAllocation4 = {{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation4);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(600000); //sys alpha _=! 1
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		assignmentProblem.addTask(task5Set6);
		assignmentProblem.addTask(task6Set6);
		assignmentProblem.addTask(task7Set6);
		assignmentProblem.addTask(task7);
		assignmentProblem.addTask(task1Set4);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario5");
		double [][][]optimalAllocation5 = {{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation5);
		
		//Thread.sleep(300000);
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(500000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task2Set4);
		assignmentProblem.addTask(task3Set4);
		assignmentProblem.addTask(task4Set4);
		assignmentProblem.addTask(task1Set5);
		assignmentProblem.addTask(task2Set5);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario6");
		double [][][]optimalAllocation6 = {{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation6);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(500000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		assignmentProblem.addTask(task3Set5);
		assignmentProblem.addTask(task4Set5);
		assignmentProblem.addTask(task5Set5);
		assignmentProblem.addTask(task6Set5);
		assignmentProblem.addTask(task7Set5);
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario7");
		double [][][]optimalAllocation7 = {{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation7);
		

		//Thread.sleep(150000);//sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task2Set10);
		assignmentProblem.addTask(task3Set10);
		assignmentProblem.addTask(task4Set10);
		assignmentProblem.addTask(task5Set10);
		assignmentProblem.addTask(task6Set10);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario8");
		double [][][]optimalAllocation8 = {{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation8);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		assignmentProblem.addTask(task7Set10);
		assignmentProblem.addTask(task1Set9);
		assignmentProblem.addTask(task2Set9);
		assignmentProblem.addTask(task3Set9);
		assignmentProblem.addTask(task4Set9);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario9");
		double [][][]optimalAllocation9 = {{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation9);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
	
		assignmentProblem.addTask(task1Set11);
		assignmentProblem.addTask(task2Set11);
		assignmentProblem.addTask(task3Set11);
		assignmentProblem.addTask(task4Set11);
		assignmentProblem.addTask(task5Set11);
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario10");
		double [][][]optimalAllocation10 = {{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation10);
		
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task1Set12);
		assignmentProblem.addTask(task2Set12);
		assignmentProblem.addTask(task3Set12);
		assignmentProblem.addTask(task4Set12);
		assignmentProblem.addTask(task5Set12);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario11");
		double [][][]optimalAllocation11 = {{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation11);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task1Set13);
		assignmentProblem.addTask(task2Set13);
		assignmentProblem.addTask(task3Set13);
		assignmentProblem.addTask(task4Set13);
		assignmentProblem.addTask(task5Set13);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario12");
		double [][][]optimalAllocation12 = {{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation12);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
	
		
		assignmentProblem.addTask(task1Set14);
		assignmentProblem.addTask(task2Set14);
		assignmentProblem.addTask(task3Set14);
		assignmentProblem.addTask(task4Set14);
		assignmentProblem.addTask(task5Set14);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario13");
		double [][][]optimalAllocation13 = {{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation13);
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		
		assignmentProblem.addTask(task1Set15);
		assignmentProblem.addTask(task2Set15);
		assignmentProblem.addTask(task3Set15);
		assignmentProblem.addTask(task4Set15);
		assignmentProblem.addTask(task5Set15);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario14");
		double [][][]optimalAllocation14 = {{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation14);
	
		
		//Thread.sleep(150000); //sys alpha = 1 synchronous
		//Thread.sleep(550000); //sys alpha != 1 synchronous
		Thread.sleep(100000); //sys alpha != 1 synchronous nominal
		//Thread.sleep(650000); //sys
		//Thread.sleep(150000); //greedy
		//Thread.sleep(200000); //Local
		
		
		assignmentProblem.addTask(task1Set16);
		assignmentProblem.addTask(task2Set16);
		assignmentProblem.addTask(task3Set16);
		assignmentProblem.addTask(task4Set16);
		assignmentProblem.addTask(task5Set16);
		
		assignmentProblem.LoadScenario("ScenarioOrebroWarehouse/ProvaScenario15");
		double [][][]optimalAllocation15 = {{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				
		};
		assignmentProblem.LoadScenarioAllocation(optimalAllocation15);
	
	}
}
