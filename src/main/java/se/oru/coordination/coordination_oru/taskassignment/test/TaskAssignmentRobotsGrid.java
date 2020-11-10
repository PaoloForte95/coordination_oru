package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import aima.core.agent.Model;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;




import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignmentSimple;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignmentSimulatedAnnealing;

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
public class TaskAssignmentRobotsGrid {
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
		viz.setInitialTransform(20, 1.85, 2.80);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);
		
		String yamlFile = "maps/map-empty.yaml";
		viz.setMap(yamlFile);
		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ArrayList <Task> taskQueue = new ArrayList <Task>();
		
		Random rand = new Random();
		
		//TEST NUMBER
		int a = 4;
		//FOLDER NUMBER 
		int b = 1;
		tec.setTestNumber(a);
		tec.setFolderNumber(b);
		
		TaskAssignment assignmentProblem = new TaskAssignment();
		
		//TaskAssignmentSimulatedAnnealing assignmentProblem = new TaskAssignmentSimulatedAnnealing();
		
		
		double [][][]optimalAllocation = {{{0.0},{0.0},{0.0},{1.0}},
				{{0.0},{0.0},{1.0},{0.0}},
				{{0.0},{1.0},{0.0},{0.0}},
				{{1.0},{0.0},{0.0},{0.0}},
		};
		
		//DO NOT USE THE COORDINATOR (in order to evaluate the nominal arrival time)
		//tec.setFakeCoordinator(true);
		//LOAD Optimal Assignment 
		//assignmentProblem.LoadScenarioAllocation(optimalAllocation);
		
		//LOAD ROADMAP
		assignmentProblem.LoadScenario("ProvaScenario1");
		int numPaths = 1;
		double delta = 0;
		for(int i = 1; i<= 6; i++) {
			Pose startPoseRobot = new Pose((4.0 + delta),30.0,-Math.PI/2);
			//Pose startPoseRobot = new Pose(4.0,(6.0 + delta),0.0);
			int robotType = 1;
			if (i==2) {
				robotType = 2;
			}
			else if(i==4){
				robotType = 2;
			}
			else if(i==6){
				robotType = 2;
			}
			Robot robot = new Robot(i,robotType);
			tec.addRobot(robot,startPoseRobot);
			Pose startPoseGoal = new Pose(15.0,(6.0 + delta),0.0);
			Pose goalPoseRobot = new Pose(30.0 ,(6.0 + delta) ,0.0);
	
			//int taskType = rand.nextInt(2)+1;
			int taskType = 1;
			if(i==1) {
				taskType = 2;
			}
			else if(i==4){
				taskType = 2;
			}
			else if(i==6){
				taskType = 2;
			}
			Task task = new Task(i,startPoseGoal,goalPoseRobot,taskType);
			assignmentProblem.addTask(task);
			taskQueue.add(task);
			delta += 3.0;
			
		}
		
		
		PrintStream fileStream = null;

		try {
			fileStream = new PrintStream(new File("ProvaTask.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		

		for (int robotID : tec.getIdleRobots()) {
			Coordinate[] footprint = tec.getFootprint(robotID);
	
			
			//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setRadius(0.2);
			rsp.setFootprint(footprint);
			rsp.setTurningRadius(4.0);
			rsp.setDistanceBetweenPathPoints(0.5);
			//rsp.setMap("maps/map-empty.yaml");
			rsp.setPlanningTimeInSecs(2);
			tec.setMotionPlanner(robotID, rsp);
		}
		//tec.setFakeCoordinator(true);
		//Solve the problem to find some feasible solution
		double alpha = 1.0;
		double timeOut = 15*1000;
		
		
		
		
		//Solve the problem to find some feasible solution
	
		
		
		tec.setBreakDeadlocks(true, false, true);
		
		
		
		assignmentProblem.setmaxNumPaths(numPaths);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.setFleetVisualization(viz);
		assignmentProblem.setCoordinator(tec);
		assignmentProblem.setLinearWeight(alpha);
		//assignmentProblem.setCostFunctionsWeight(0.8, 0.1, 0.1);	
		//assignmentProblem.setTimeOutinMin(timeOut);
		
		//FOR S AND GD
		//MPSolver solver = assignmentProblem.buildOptimizationProblemWithBNormalized(tec); 
		//double [][][] assignmentMatrix = assignmentProblem.solveOptimizationProblem(solver,tec);
		//double [][][] assignmentMatrix = assignmentProblem.solveOptimizationProblemLocalSearch(tec,-1);
		double [][][] assignmentMatrix = assignmentProblem.solveOptimizationProblemGreedyAlgorithm(tec);
		//FOR SA
		//double [][][] assignmentMatrix = assignmentProblem.simulatedAnnealingAlgorithm(tec,-1);
		
		
		for (int i = 0; i < assignmentMatrix.length; i++) {
			for (int j = 0; j < assignmentMatrix[0].length; j++) {
				for(int s = 0; s < numPaths; s++) {
					System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ assignmentMatrix[i][j][s]);
					if(assignmentMatrix[i][j][s] == 1) {
						System.out.println("Robot " +(i+1) +" is assigned to Task "+ (j+1)+" throw Path " + (s+1));
						
					}
				}
			} 
		}
		assignmentProblem.TaskAllocation(assignmentMatrix,tec);	
		
		
	}
}

