package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Comparator;
import java.util.Random;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment;
import se.oru.coordination.coordination_oru.taskassignment.Robot;
import se.oru.coordination.coordination_oru.taskassignment.Task;
import com.google.ortools.linearsolver.*;


@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TaskAssignmentOrebroWarehouse2 {
	//load library used for optimization
	 static {
		    System.loadLibrary("jniortools");
		  }
	public static void main(String[] args) throws InterruptedException {
		//Max Vel and Acc for the robots
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a timed trajectory envelope coordinator.
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		//final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		final TimedTrajectoryEnvelopeCoordinatorSimulation tec = new TimedTrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
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
		
		//Need to instantiate the fleetmaster interface
		tec.instantiateFleetMaster(0.1, false);
		
		
		
		//Coordinate footprint1 = new Coordinate(-1.0,0.5);
		//Coordinate footprint2 = new Coordinate(1.0,0.5);
		//Coordinate footprint3 = new Coordinate(1.0,-0.5);
		//Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(-0.5,-0.5);
		Coordinate footprint3 = new Coordinate(0.5,-0.5);
		Coordinate footprint4 = new Coordinate(0.5,0.5);
		
		
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		Coordinate[] fp2 = new Coordinate[] {
				new Coordinate(-0.8,0.0),
				new Coordinate(0.0,0.8),
				new Coordinate(0.8,0.0),
				new Coordinate(0.0,-0.8),
		};
		
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 0, 0);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);

		

	   
		Pose startPoseRobot1 = new Pose(33.0,6.0,Math.PI);
		Pose startPoseRobot2 = new Pose(3.0,28.0,0.0);
		Pose startPoseRobot3 = new Pose(3.0,20.0,0.0);
		Pose startPoseRobot4 = new Pose(3.0,25.0,0.0);
		Pose startPoseRobot5 = new Pose(8.0,1.0,Math.PI/2);	
		Pose startPoseRobot6 = new Pose(11.0,2.0,Math.PI/2);	
		Pose startPoseRobot7 = new Pose(20.0,1.0,Math.PI/2);	
		
		
		
		Robot robot1 = new Robot(1, 1,tec.getDefaultFootprint() );
		Robot robot2 = new Robot(2,2,fp2);
		Robot robot3 = new Robot(3,1,tec.getDefaultFootprint() );
		Robot robot4 = new Robot(4,1,tec.getDefaultFootprint());
		Robot robot5 = new Robot(5,1,tec.getDefaultFootprint());
		Robot robot6 = new Robot(6,2,fp2);
		Robot robot7 = new Robot(7,1,tec.getDefaultFootprint());
		
		//tec.setFootprint(robot6.getRobotID(), footprint1, footprint2, footprint3, footprint4);
		tec.setFootprint(robot2.getRobotID(),fp2);
		tec.setFootprint(robot6.getRobotID(),fp2);
		
		
		
		Pose startPoseGoal1 = new Pose(7.0,7.0,Math.PI/2);	
		Pose startPoseGoal2 = new Pose(13.0,20.0,0.0);
		Pose startPoseGoal3 = new Pose(13.0,24.0,0.0);
		Pose startPoseGoal4 = new Pose(25.0,5.0,0.0);	
		Pose startPoseGoal5 = new Pose(18.0,15.0,0.0);
		Pose startPoseGoal6 = new Pose(11.0,5.0,Math.PI/2);	
		Pose startPoseGoal7 = new Pose(20.0,11.0,0.0);
		
		Pose goalPoseRobot1 = new Pose(6.0,16.5,Math.PI/2);
		Pose goalPoseRobot2 = new Pose(24.0,19.0,0.0);
		Pose goalPoseRobot3 = new Pose(21.0,24.0,0.0);
		Pose goalPoseRobot4 = new Pose(30.5,8.0,Math.PI/2);
		Pose goalPoseRobot5 = new Pose(25.0,15.0,0.0);
		Pose goalPoseRobot6 = new Pose(3.0,23.0,0.0);
		Pose goalPoseRobot7 = new Pose(24.0,11.0,0.0);

		
		
		
		tec.addRobot(robot1,startPoseRobot1);
		tec.addRobot(robot2,startPoseRobot2);
		tec.addRobot(robot3,startPoseRobot3);
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
		Task task6 = new Task(6,startPoseGoal6,goalPoseRobot6,2);
		Task task7 = new Task(7,startPoseGoal7,goalPoseRobot7,2);

		
		viz.setMap("maps/basement.yaml");
		
		
		ReedsSheppCarPlanner rsp2 = new ReedsSheppCarPlanner();
		rsp2.setRadius(0.2);
		rsp2.setTurningRadius(4.0);
		rsp2.setFootprint(tec.getDefaultFootprint());
		rsp2.setDistanceBetweenPathPoints(0.5);
		rsp2.setMapFilename("maps"+File.separator+Missions.getProperty("image","maps/basement.yaml"));
		double res2 = Double.parseDouble(Missions.getProperty("resolution","maps/basement.yaml"));
		//rsp2.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/map-empty.yaml"));
		//double res2 = 0.2;
		rsp2.setMapResolution(res2);
		rsp2.setPlanningTimeInSecs(5);
		
		
		TaskAssignment assignmentProblem = new TaskAssignment();
		
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);
		assignmentProblem.addTask(task4);
		assignmentProblem.addTask(task5);
		assignmentProblem.addTask(task6);
		assignmentProblem.addTask(task7);
		assignmentProblem.setFleetVisualization(viz);
		int numPaths = 1;
		
		for (int robotID : tec.getIdleRobots()) {
			Coordinate[] footprint = tec.getFootprint(robotID);
			//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setFootprint(footprint);
			rsp.setRadius(0.2);
			rsp.setTurningRadius(4.0);
			rsp.setDistanceBetweenPathPoints(0.5);
			rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/basement.yaml"));
			double res = Double.parseDouble(Missions.getProperty("resolution", "maps/basement.yaml"));
			//rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/map-empty.yaml"));
			//double res = 0.2;
			rsp.setMapResolution(res);
			rsp.setPlanningTimeInSecs(2);
			tec.setMotionPlanner(robotID, rsp);
			//rsp.setStart(startPoseGoal1	);
			//rsp.setGoals(goalPoseRobot1);
			//rsp.setFootprint(tec.getFootprint(robotID));
			//if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot3 + " and " + goalPoseRobot3);
			//PoseSteering[] pss3 = rsp.getPath();

			//Mission m3 = new Mission(1,pss3);
			//tec.addMissions(m3);
		}
		

	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		double alpha = 1.0;
		
		double [][][]optimalAllocation = {{{1.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{1.0},{0.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{1.0},{0.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{1.0},{0.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{1.0}},
				
		};
		
		//assignmentProblem.LoadScenario("ProvaScenario");
		//assignmentProblem.LoadScenarioAllocation(optimalAllocation);
		assignmentProblem.setmaxNumPaths(numPaths);
		assignmentProblem.setFleetVisualization(viz);
		assignmentProblem.setLinearWeight(alpha);
		//assignmentProblem.setCostFunctionsWeight(0.8, 0.1, 0.1);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.startTaskAssignment(tec);
		
		//INitial
		//Pose startPoseGoal1New = new Pose(7.0,4.0,Math.PI/2);	
		//Pose startPoseGoal2new = new Pose(13.0,20.0,0.0);
		//Pose startPoseGoal3New = new Pose(13.0,24.0,0.0);
		//Pose startPoseGoal4New = new Pose(20.0,7.0,Math.PI/2);	
		//Pose startPoseGoal5New = new Pose(18.0,15.0,0.0);
		//Pose startPoseGoal6New = new Pose(11.0,5.0,Math.PI/2);	
		//Pose startPoseGoal7New = new Pose(18.0,7.0,Math.PI/2);	
		
		//Pose goalPoseRobot1New = new Pose(7.0,15.5,Math.PI/2);
		//Pose goalPoseRobot2New = new Pose(24.0,19.0,0.0);
		//Pose goalPoseRobot3New = new Pose(21.0,24.0,0.0);
		//Pose goalPoseRobot4New = new Pose(30.5,7.5,Math.PI/2);
		//Pose goalPoseRobot5New = new Pose(25.0,15.0,0.0);
		//Pose goalPoseRobot6New = new Pose(3.0,23.0,0.0);
		//Pose goalPoseRobot7New = new Pose(24.0,11.0,0.0);
		
		
		
		//New tasks real
		//Second set of tasks
		//-> 4 more tasks ( 3 of type 1 and 1 of type 2)
		Pose startPoseGoal1Set2 = new Pose(11.0,24.0,0.0);
		Pose goalPoseRobot1Set2 = new Pose(14.0,29.0,Math.PI/2);
		Task task1Set2 = new Task(1,startPoseGoal1Set2,goalPoseRobot1Set2,1);
		///////////////////
		Pose startPoseGoal2Set2 = new Pose(22.0,5.0,Math.PI/2);
		Pose goalPoseRobot2Set2 = new Pose(34.5,5.5,0.0);
		Task task2Set2 = new Task(2,startPoseGoal2Set2,goalPoseRobot2Set2,2);
		///////////////////
		Pose startPoseGoal3Set2 = new Pose(22.0,3.0,Math.PI/2);
		Pose goalPoseRobot3Set2 = new Pose(34.5,3.5,0.0);
		Task task3Set2 = new Task(3,startPoseGoal3Set2,goalPoseRobot3Set2,1);
		///////////////////
		Pose startPoseGoal4Set2 = new Pose(11.0,18.0,0.0);
		Pose goalPoseRobot4Set2 = new Pose(5.0,12.5,Math.PI);
		Task task4Set2 = new Task(4,startPoseGoal4Set2,goalPoseRobot4Set2,1);
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		//Third set of tasks
		//-> 5 more tasks (4 of type 1 and 1 of type 2)
		Pose startPoseGoal1Set3 = new Pose(8.0,26.0,0.0);
		Pose goalPoseRobot1Set3 = new Pose(2.0,34.0,Math.PI/2);
		Task task1Set3 = new Task(5,startPoseGoal1Set3,goalPoseRobot1Set3,2);
		////////////////////////////////////////////
		Pose startPoseGoal2Set3 = new Pose(11.0,12.0,-Math.PI/2);
		Pose goalPoseRobot2Set3 = new Pose(2.0,3.0,Math.PI);
		Task task2Set3 = new Task(6,startPoseGoal2Set3,goalPoseRobot2Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal3Set3 = new Pose(18.0,8.0,-Math.PI/2);
		Pose goalPoseRobot3Set3 = new Pose(15.0,1.5,-Math.PI/2);
		Task task3Set3 = new Task(7,startPoseGoal3Set3,goalPoseRobot3Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal4Set3 = new Pose(11.0,10.0,-Math.PI/2);
		Pose goalPoseRobot4Set3 = new Pose(2.0,6.0,Math.PI);
		Task task4Set3 = new Task(8,startPoseGoal4Set3,goalPoseRobot4Set3,1);
		////////////////////////////////////////////
		Pose startPoseGoal5Set3 = new Pose(7.5,33.5,0.0);
		Pose goalPoseRobot5Set3 = new Pose(14.0,34,Math.PI/2);
		Task task5Set3 = new Task(9,startPoseGoal5Set3,goalPoseRobot5Set3,1);
		//////////////////////////////////////////////
		//////////////////////////////////////////////
		//Forth set of tasks
		//-> 4 more tasks (3 of type 1 and 1 of type 2)
		Pose startPoseGoal1Set4 = new Pose(18.0,15.0,0.0);
		Pose goalPoseRobot1Set4 = new Pose(25.0,15.0,0.0);
		Task task1Set4 = new Task(10,startPoseGoal1Set4,goalPoseRobot1Set4,1);
		//////////////////////////////////////
		Pose startPoseGoal2Set4 = new Pose(15.0,24.0,0.0);
		Pose goalPoseRobot2Set4 = new Pose(21.0,24.0,0.0);
		Task task2Set4 = new Task(11,startPoseGoal2Set4,goalPoseRobot2Set4,1);
		//////////////////////////////////////
		Pose startPoseGoal3Set4 = new Pose(20.0,11.0,0.0);
		Pose goalPoseRobot3Set4 = new Pose(24.0,11.0,0.0);
		Task task3Set4 = new Task(12,startPoseGoal3Set4,goalPoseRobot3Set4,2);
		//////////////////////////////////////
		Pose startPoseGoal4Set4 = new Pose(9.0,22.0,Math.PI/2);
		Pose goalPoseRobot4Set4 = new Pose(10.0,29.0,Math.PI/2);
		Task task4Set4 = new Task(13,startPoseGoal4Set4,goalPoseRobot4Set4,1);
		//////////////////////////////////////
		//Fifth set of tasks
		//->  7 more tasks (5 of type 1 and 2 of type 2)
		Pose startPoseGoal1Set5 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot1Set5 = new Pose(5.0,10.0,Math.PI);
		Task task1Set5 = new Task(14,startPoseGoal1Set5,goalPoseRobot1Set5,2);
		////////////////////////////////////////////////////
		Pose startPoseGoal2Set5 = new Pose(2.0,15.0,-Math.PI/2);
		Pose goalPoseRobot2Set5 = new Pose(2.0,10.0,-Math.PI/2);
		Task task2Set5 = new Task(15,startPoseGoal2Set5,goalPoseRobot2Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal3Set5 = new Pose(9.0,23.0,Math.PI);
		Pose goalPoseRobot3Set5 = new Pose(6.0,19.0,-Math.PI/2);
		Task task3Set5 = new Task(16,startPoseGoal3Set5,goalPoseRobot3Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal4Set5 = new Pose(18.0,19.0,0.0);
		Pose goalPoseRobot4Set5 = new Pose(24.0,19.0,0.0);
		Task task4Set5 = new Task(17,startPoseGoal4Set5,goalPoseRobot4Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal5Set5 = new Pose(20.0,5.0,0.0);
		Pose goalPoseRobot5Set5 = new Pose(34.5,7.5,0.0);
		Task task5Set5 = new Task(18,startPoseGoal5Set5,goalPoseRobot5Set5,1);
		////////////////////////////////////////////////////
		Pose startPoseGoal6Set5 = new Pose(20.0,3.0,0.0);
		Pose goalPoseRobot6Set5 = new Pose(25.5,7.5,Math.PI/2);
		Task task6Set5 = new Task(19,startPoseGoal6Set5,goalPoseRobot6Set5,2);
		////////////////////////////////////////////////////
		Pose startPoseGoal7Set5 = new Pose(8.0,12.0,-Math.PI/2);
		Pose goalPoseRobot7Set5 = new Pose(12.0,10.0,0.0);
		Task task7Set5 = new Task(20,startPoseGoal7Set5,goalPoseRobot7Set5,1);
		////////////////////////////////////////////////////
		//////////////////////////////////////
		//Sixth set of tasks
		//->  7 more tasks (5 of type 1 and 2 of type 2)
		Pose startPoseGoal1Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot1Set6 = new Pose(28.0,37.0,0.0);
		Task task1Set6 = new Task(21,startPoseGoal1Set6,goalPoseRobot1Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot2Set6 = new Pose(28.0,34.0,0.0);
		Task task2Set6 = new Task(22,startPoseGoal2Set6,goalPoseRobot2Set6,2);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal3Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot3Set6 = new Pose(28.0,31.0,0.0);
		Task task3Set6 = new Task(23,startPoseGoal3Set6,goalPoseRobot3Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal4Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot4Set6 = new Pose(28.0,28.0,0.0);
		Task task4Set6 = new Task(24,startPoseGoal4Set6,goalPoseRobot4Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal5Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot5Set6 = new Pose(28.0,19.0,0.0);
		Task task5Set6 = new Task(25,startPoseGoal5Set6,goalPoseRobot5Set6,1);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal6Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot6Set6 = new Pose(28.0,22.0,0.0);
		Task task6Set6 = new Task(26,startPoseGoal6Set6,goalPoseRobot6Set6,2);
		/////////////////////////////////////////////////////////////////////
		Pose startPoseGoal7Set6 = new Pose(20.0,33.0,0.0);
		Pose goalPoseRobot7Set6 = new Pose(28.0,25.0,0.0);
		Task task7Set6 = new Task(27,startPoseGoal7Set6,goalPoseRobot7Set6,1);
		/////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////
		//Seventh set of tasks
		//->  3 more tasks (3 of type 1 and 0 of type 2)
		Pose startPoseGoal1Set7 = new Pose(12.0,24.0,0.0);
		Pose goalPoseRobot1Set7 = new Pose(24.5,19.0,0.0);
		Task task1Set7 = new Task(28,startPoseGoal1Set7,goalPoseRobot1Set7,1);
		////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set7 = new Pose(11.0,22.0,0.0);
		Pose goalPoseRobot2Set7 = new Pose(25.5,15.0,0.0);
		Task task2Set7 = new Task(29,startPoseGoal2Set7,goalPoseRobot2Set7,1);
		///////////////////////////////////////////////////
		Pose startPoseGoal3Set7 = new Pose(10.0,22.0,0.0);
		Pose goalPoseRobot3Set7 = new Pose(25.5,7.0,Math.PI/2);
		Task task3Set7 = new Task(30,startPoseGoal3Set7,goalPoseRobot3Set7,1);
		///////////////////////////////////////////////////
		//Eighth set of tasks
		//->  4 more tasks (3 of type 1 and 1 of type 2)
		Pose startPoseGoal1Set8 = new Pose(12.0,20.0,0.0);
		Pose goalPoseRobot1Set8 = new Pose(35.0,5.5,0.0);
		Task task1Set8 = new Task(31,startPoseGoal1Set8,goalPoseRobot1Set8,2);
		///////////////////////////////////////////////////////////////////
		Pose startPoseGoal2Set8 = new Pose(12.0,20.0,0.0);
		Pose goalPoseRobot2Set8 = new Pose(35.0,2.5,0.0);
		Task task2Set8 = new Task(32,startPoseGoal2Set8,goalPoseRobot2Set8,1);
		///////////////////////////////////////////////////////////////////
		Pose startPoseGoal3Set8 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot3Set8 = new Pose(2.0,2.0,Math.PI);
		Task task3Set8 = new Task(33,startPoseGoal3Set8,goalPoseRobot3Set8,1);
		///////////////////////////////////////////////////////////////////
		Pose startPoseGoal4Set8 = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot4Set8 = new Pose(2.0,5.0,Math.PI);
		Task task4Set8 = new Task(34,startPoseGoal4Set8,goalPoseRobot4Set8,1);
		///////////////////////////////////////////////////
		///////////////////////////////////////////////////
		
		//TEST ZONE
		Pose startPoseGoal1New = new Pose(11.0,18.0,-Math.PI/2);
		Pose goalPoseRobot1New = new Pose(2.0,5.0,Math.PI);
	
		Robot robot1New = new Robot(8, 1,tec.getDefaultFootprint() );
		Robot robot2New = new Robot(9,2,fp2);
		Robot robot3New = new Robot(10,1,tec.getDefaultFootprint() );
		Robot robot4New = new Robot(11,1,tec.getDefaultFootprint());
		Robot robot5New = new Robot(12,1,tec.getDefaultFootprint());
		Robot robot6New = new Robot(13,2,fp2);
		
		//tec.addRobot(robot1New,startPoseGoal1New);
		//tec.addRobot(robot2New,goalPoseRobot1New);
		//tec.addRobot(robot3New,startPoseGoal2new);
		//tec.addRobot(robot4New, goalPoseRobot2New);
		//tec.addRobot(robot5New, startPoseGoal1Set5);
		//tec.addRobot(robot6New, goalPoseRobot1Set5);
		///////
		Thread.sleep(40000);
		
		//Add the second set of tasks
		assignmentProblem.addTask(task1Set2);
		assignmentProblem.addTask(task2Set2);
		assignmentProblem.addTask(task3Set2);
		assignmentProblem.addTask(task4Set2);
		///Sleep for a while
		
		Thread.sleep(40000);
		
		
		
		assignmentProblem.addTask(task1Set3);
		assignmentProblem.addTask(task2Set3);
		assignmentProblem.addTask(task3Set3);
		assignmentProblem.addTask(task4Set3);
		assignmentProblem.addTask(task5Set3);
		
		
		
		Thread.sleep(40000);
		
		assignmentProblem.addTask(task1Set4);
		assignmentProblem.addTask(task2Set4);
		assignmentProblem.addTask(task3Set4);
		assignmentProblem.addTask(task4Set4);
		
		
		//if(assignmentProblem.getTaskIDs() != null) {
		//	System.out.println("NOOOOOOOOOOO ANCORA TASKS");
		//while(!assignmentProblem.getTaskIDs().isEmpty()) {
				
			//}
		//}
		
		Thread.sleep(40000);
		
		assignmentProblem.addTask(task1Set5);
		assignmentProblem.addTask(task2Set5);
		assignmentProblem.addTask(task3Set5);
		assignmentProblem.addTask(task4Set5);
		assignmentProblem.addTask(task5Set5);
		assignmentProblem.addTask(task6Set5);
		assignmentProblem.addTask(task7Set5);
		
		Thread.sleep(60000);
		
		//assignmentProblem.addTask(task1Set6);
		//assignmentProblem.addTask(task2Set6);
		assignmentProblem.addTask(task3Set6);
		assignmentProblem.addTask(task4Set6);
		assignmentProblem.addTask(task5Set6);
		assignmentProblem.addTask(task6Set6);
		assignmentProblem.addTask(task7Set6);
		
		//Thread.sleep(40000);
		
		//assignmentProblem.addTask(task1Set7);
		//assignmentProblem.addTask(task2Set7);
		//assignmentProblem.addTask(task3Set7);
		
		
		//Thread.sleep(40000);
		
		//assignmentProblem.addTask(task1Set8);
		//assignmentProblem.addTask(task2Set8);
		//assignmentProblem.addTask(task3Set8);
		//assignmentProblem.addTask(task4Set8);
	}
}
