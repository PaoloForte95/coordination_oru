package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import aima.core.agent.Model;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner.PLANNING_ALGORITHM;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;




import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignmentSimple;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.taskassignment.Robot;
import se.oru.coordination.coordination_oru.taskassignment.Task;




import com.google.ortools.linearsolver.*;
import com.google.ortools.linearsolver.MPSolver.ResultStatus;
import com.google.ortools.linearsolver.PartialVariableAssignment;
import com.google.ortools.constraintsolver.Solver;
import com.google.ortools.constraintsolver.Solver;
import com.google.ortools.sat.*;

@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TaskAssignmentCorridors2 {
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
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		//Need to setup infrastructure that maintains the representation
		
		tec.setupSolver(0, 100000000);
		tec.startInference();
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(13, 6.8, 9.41);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);
		//tec.setFakeCoordinator(true);
		viz.setMap("maps/map-corridors-modified.yaml");
		String yamlFile = new String("maps/map-corridors-modified.yaml");
		
		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		//Set the default motion planner
		ReedsSheppCarPlanner rsp1 = new ReedsSheppCarPlanner(PLANNING_ALGORITHM.RRTstar);
		rsp1.setRadius(0.1);
		rsp1.setTurningRadius(4.0);
		rsp1.setFootprint(tec.getDefaultFootprint());
		rsp1.setDistanceBetweenPathPoints(0.5);
		rsp1.setMap(yamlFile);
		rsp1.setPlanningTimeInSecs(10);
		
		Pose startPoseRobot1 = new Pose(8.0,11.0,0.0);
		Pose startPoseRobot2 = new Pose(8.0,13.0,0.0);
		//Pose startPoseRobot3 = new Pose(8.0,18.0,0.0);
		Pose startPoseRobot3 = new Pose(68.0,18.0,Math.PI);
		Pose startPoseRobot4 = new Pose(8.0,22.5,0.0);
		
		
		//Pose startPoseRobot5 = new Pose(4.0,12.0,Math.PI/2);

		Robot robot1 = new Robot(1, 1 );
		Robot robot2 = new Robot(2, 2 );
		Robot robot3 = new Robot(3, 3 );
		Robot robot4 = new Robot(4, 4 );
		Robot robot5 = new Robot(5, 1 );
		
		tec.addRobot(robot1,startPoseRobot1);
		tec.addRobot(robot2,startPoseRobot2);
		tec.addRobot(robot3,startPoseRobot3);
		//tec.addRobot(robot4,startPoseRobot4);
		//tec.addRobot(robot5,startPoseRobot5);
		
		
		
	/*
		Pose startinter = new Pose(26.0,15.0,0.0);
		Pose fin = new Pose(68.0,12.0,0.0);
		if(robotID==1 && path==0) {
			startinter = new Pose(26.0,15.0,0.0);
			fin = new Pose(68.0,12.0,0.0);
		}else if(robotID==1 && path==1) {
			startinter = new Pose(30.0,24.0,0.0);
			fin = new Pose(68.1,12.0,0.0);
		}else if(robotID==2 && path==0) {
			startinter = new Pose(26.0,15.0,0.0);
			fin = new Pose(68.0,15.0,0.0);
		}else if(robotID==2 && path==1) {
			startinter = new Pose(30.0,24.0,0.0);
			fin = new Pose(68.1,15.0,0.0);
		}else if(robotID==3 && path==0) {
			startinter = new Pose(50.0,15.0,0.0);
			fin = new Pose(8.0,18.0,Math.PI);
		}else if(robotID==3 && path==1) {
			startinter = new Pose(50.0,24.0,0.0);
			fin = new Pose(8.1,18.0,Math.PI);
		}
		
		
		*/	
		
		
		
		
		
		
		
		
		
		//Pose startPoseGoal1 = new Pose(28.0,32.0,0.0);
		Pose startPoseGoal1 = new Pose(17.0,15.0,0.0);
		Pose startPoseGoal2 = new Pose(17.0,15.0,0.0);
		Pose startPoseGoal3 = new Pose(62.0,18.0,0.0);
		
	
		
		
		
		Pose goalPoseRobot1 = new Pose(68.0,12.0,0.0);
		Pose goalPoseRobot2 = new Pose(68.0,15.0,0.0);
		//Pose goalPoseRobot3 = new Pose(68.0,18.0,0.0);
		Pose goalPoseRobot3 = new Pose(8.0,18.0,Math.PI);
		

		
		//Pose goalPoseRobot8 = new Pose(68.0,20.0,0.0);
		
		
		Task task1 = new Task(1,startPoseGoal1,goalPoseRobot1,1);
		Task task2 = new Task(2,startPoseGoal2,goalPoseRobot2,2);
		Task task3 = new Task(3,startPoseGoal3,goalPoseRobot3,3);

	
		
		
		
		//Task task8 = new Task(8,startPoseGoal8,goalPoseRobot8,4);
		
		for (int robotID : tec.getIdleRobots()) {	
			//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
			ReedsSheppCarPlanner rsp = (ReedsSheppCarPlanner) rsp1.getCopy(false);
			tec.setMotionPlanner(robotID, rsp);
		}

	    ///////////////////////////////////////////////////////
		
		int a = 5;
		//FOLDER NUMBER 
		int b = 3;
		tec.setTestNumber(a);
		tec.setFolderNumber(b);
		
		//Solve the problem to find some feasible solution
		double alpha = 1.0;
		TaskAssignment assignmentProblem = new TaskAssignment();
		
		assignmentProblem.LoadScenario("ProvaScenario1");
		
		double [][][]optimalAllocation = new double[3][3][3];
		
		
		optimalAllocation[0][0][0]= 1;
		optimalAllocation[1][1][0]= 1;
		optimalAllocation[2][2][1]= 1;
		
		
		
		assignmentProblem.LoadScenarioAllocation(optimalAllocation);
		
		
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);
		//assignmentProblem.addTask(task8);
		
		assignmentProblem.setFleetVisualization(viz);
		
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.setLinearWeight(alpha);
		//assignmentProblem.setCostFunctionsWeight(0.8, 0.1, 0.1);
		assignmentProblem.setmaxNumPaths(2);
		
		assignmentProblem.startTaskAssignment(tec);
		//assignmentProblem.startTaskAssignmentGreedyAlgorithm(tec);
	}
}

