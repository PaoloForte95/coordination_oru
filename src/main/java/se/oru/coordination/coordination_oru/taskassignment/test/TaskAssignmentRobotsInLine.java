package se.oru.coordination.coordination_oru.taskassignment.test;

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
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;




import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment;



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
public class TaskAssignmentRobotsInLine {
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
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 0, 0);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);

		
		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1,footprint2,footprint3,footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		
		
		Pose startPoseRobot1 = new Pose(20.0,6.0,0.0);
		Pose startPoseRobot2 = new Pose(16.0,6.0,0.0);
		Pose startPoseRobot3 = new Pose(12.0,6.0,0.0);
	

		Robot robot1 = new Robot(1);
		Robot robot2 = new Robot(2);
		Robot robot3 = new Robot(3);
		
		
		tec.addRobot(robot1, startPoseRobot1);
		tec.addRobot(robot2, startPoseRobot2);
		tec.addRobot(robot3, startPoseRobot3);
	

	
		Pose startPoseGoal1 = new Pose(24.0,6.0,0.0);
		Pose startPoseGoal2 = new Pose(24.0,6.0,0.0);
		Pose startPoseGoal3 = new Pose(24.0,6.0,0.0);
		Pose goalPoseRobot1 = new Pose(34.0,6.0,0.0);
		Pose goalPoseRobot2 = new Pose(30.0,6.0,0.0);
		Pose goalPoseRobot3 = new Pose(26.0,6.0,0.0);
		
		
		Task task1 = new Task(startPoseGoal1,goalPoseRobot1,1);
		Task task2 = new Task(startPoseGoal2,goalPoseRobot2,1);
		Task task3 = new Task(startPoseGoal3,goalPoseRobot3,1);

	
		
		viz.displayTask(startPoseGoal1, goalPoseRobot1,1);
		//viz.displayTask(startPoseGoal2, goalPoseRobot2,2);
		//viz.displayTask(startPoseGoal3, goalPoseRobot3,3);
		
	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		double alpha = 0.6;
		TaskAssignment assignmentProblem = new TaskAssignment();
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);

		
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.setDefaultMotionPlanner(rsp);
		tec.setDefaultMotionPlanner(assignmentProblem.getDefaultMotionPlanner());
		assignmentProblem.setFleetVisualization(viz);
		
		assignmentProblem.startTaskAssignment(tec);
		//assignmentProblem.startTaskAssignmentGreedyAlgorithm(alpha, tec);
	}
}
