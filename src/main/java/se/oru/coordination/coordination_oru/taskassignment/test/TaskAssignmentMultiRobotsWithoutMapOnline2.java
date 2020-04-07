package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;

import aima.core.agent.Model;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
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
public class TaskAssignmentMultiRobotsWithoutMapOnline2 {
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

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		
		//Need to instantiate the fleetmaster interface
		tec.instantiateFleetMaster(0.1, false);
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 0, 0);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);

		String yamlFile = "maps/map-empty.yaml";
		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1,footprint2,footprint3,footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setPlanningTimeInSecs(2);
		
		
		ReedsSheppCarPlanner rsp2 = new ReedsSheppCarPlanner();
		rsp2.setRadius(0.2);
		rsp2.setFootprint(footprint1,footprint2,footprint3,footprint4);
		rsp2.setTurningRadius(4.0);
		rsp2.setDistanceBetweenPathPoints(0.5);
		rsp2.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res2 = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res2);
		
	   
		Pose startPoseRobot1 = new Pose(4.0,6.0,0.0);
		Pose startPoseRobot2 = new Pose(6.0,16.0,-Math.PI/4);
		Pose startPoseRobot3 = new Pose(9.0,6.0,Math.PI/2);
		Pose startPoseRobot4 = new Pose(16.0,30.0,-Math.PI/2);
		Pose startPoseRobot5 = new Pose(5.0,20.0,Math.PI/2);

		Robot robot1 = new Robot(1, 1);
		Robot robot2 = new Robot(2, 1);
		Robot robot3 = new Robot(3, 1);
		Robot robot4 = new Robot(4, 1);
		Robot robot5 = new Robot(5, 1);
		
		
		
		tec.addRobot(robot1,startPoseRobot1);
		tec.addRobot(robot2,startPoseRobot2);
		tec.addRobot(robot3,startPoseRobot3);
		//tec.addRobot(robot4, startPoseRobot4);
		//tec.addRobot(robot5, startPoseRobot5);

	
		Pose startPoseGoal1 = new Pose(16.0,25.0,0.0);
		Pose startPoseGoal2 = new Pose(25.0,7.0,0.0);
		Pose startPoseGoal3 = new Pose(4.0,8.0,0.0);
		Pose startPoseGoal4 = new Pose(8.0,16.0,-Math.PI/2);
		Pose startPoseGoal5 = new Pose(25.0,16.0,Math.PI/2);
		
		Pose startPoseGoal6 = new Pose(7.0,25.0,Math.PI/2);
		
		
		Pose goalPoseGoal1 = new Pose(16.0,15.0,Math.PI/4);
		Pose goalPoseGoal2 = new Pose(27.0,3.0,-Math.PI/4);
		Pose goalPoseGoal3 = new Pose(21.0,3.0,-Math.PI/2);
		Pose goalPoseGoal4 = new Pose(12.0,20.0,-Math.PI/2);
		Pose goalPoseGoal5 = new Pose(32.0,25.0,Math.PI/2);
		
		Pose goalPoseGoal6 = new Pose(12.0,45.0,Math.PI/2);

		
		Task task1 = new Task(1,startPoseGoal1,goalPoseGoal1,1);
		Task task2 = new Task(2,startPoseGoal2,goalPoseGoal2,1);
		Task task3 = new Task(3,startPoseGoal3,goalPoseGoal3,1);

		Task task4 = new Task(4,startPoseGoal4,goalPoseGoal4,1);
		Task task5 = new Task(5,startPoseGoal5,goalPoseGoal5,1);
		
		Task task6 = new Task(6,startPoseGoal6,goalPoseGoal6,1);
		
	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		double alpha = 0.7;
		double numPaths = 1;
		TaskAssignmentSimple assignmentProblem = new TaskAssignmentSimple();
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);
		assignmentProblem.addTask(task4);
		assignmentProblem.addTask(task5);
		
		
		
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.setDefaultMotionPlanner(rsp);
		//assignmentProblem.setDefaultMotionPlanner2(rsp2);
		assignmentProblem.setFleetVisualization(viz);
		tec.setDefaultMotionPlanner(assignmentProblem.getDefaultMotionPlanner());
		assignmentProblem.setCoordinator(tec);
		assignmentProblem.setLinearWeight(alpha);
		assignmentProblem.setCostFunctionsWeight(0.8, 0.1, 0.1);
		//assignmentProblem.setNumThreadToUse(2);
		
		assignmentProblem.startTaskAssignment(tec);
		
		
		
		Pose startPoseGoal1new = new Pose(18.0,12.0,0.0);
		Pose startPoseGoal2new = new Pose(30.0,9.0,0.0);
		Pose startPoseGoal3new = new Pose(35.0,37.0,0.0);
		
		Pose newPoseGoal1 = new Pose(20.0,15.0,0.0);
		Pose newPoseGoal2 = new Pose(32.0,10.0,0.0);
		Pose newPoseGoal3 = new Pose(42.0,45.0,0.0);
		Task task1new = new Task(7,startPoseGoal1new,newPoseGoal1,1);
		Task task2new = new Task(8,startPoseGoal2new,newPoseGoal2,1);
		Task task3new = new Task(9,startPoseGoal3new,newPoseGoal3,1);
		

		Thread.sleep(5000);
		
		
		//assignmentProblem.addTask(task1new);
		//assignmentProblem.addTask(task2new);
		//assignmentProblem.addTask(task3new);
		
	}
}
