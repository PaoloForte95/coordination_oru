package se.oru.coordination.coordination_oru.taskassignment;

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



import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;


import se.oru.coordination.coordination_oru.taskassignment.Task;




import com.google.ortools.linearsolver.*;
import com.google.ortools.linearsolver.MPSolver.ResultStatus;
import com.google.ortools.linearsolver.PartialVariableAssignment;
import com.google.ortools.constraintsolver.Solver;
import com.google.ortools.constraintsolver.Solver;
import com.google.ortools.sat.*;

@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TaskAssignmentThreeRobots2 {
	//load library used for optimization
	 static {
		    System.loadLibrary("jniortools");
		  }
	public static void main(String[] args) throws InterruptedException {
		//Max Vel and Acc for the robots
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
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
		
		
		tec.addRobot(1, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.addRobot(2, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.addRobot(3, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		
		//tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		//tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		//tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		
		//tec.setNominalTrajectoryParameters(1, MAX_VEL, MAX_VEL, false, -1, -1, -1, MAX_ACCEL, -1, -1);
		//tec.setNominalTrajectoryParameters(2, 1.25*MAX_VEL, 1.25*MAX_VEL, false, -1, -1, -1, 1.25*MAX_ACCEL, -1, -1);
		//tec.setNominalTrajectoryParameters(3, 0.5*MAX_VEL, 0.5*MAX_VEL, false, -1, -1, -1, 0.5*MAX_ACCEL, -1, -1);
		
		
		
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
		
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1,footprint2,footprint3,footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		
		Pose startPoseRobot1 = new Pose(4.0,6.0,0.0);
		Pose startPoseRobot2 = new Pose(6.0,16.0,-Math.PI/4);
		Pose startPoseRobot3 = new Pose(9.0,6.0,Math.PI/2);
		
		
		Pose startPoseRobot4 = new Pose(16.0,30.0,-Math.PI/2);
		Pose startPoseRobot5 = new Pose(-5.0,-5.0,Math.PI/2);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, startPoseRobot3);
		
		
		
		tec.addRobot(4, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.addRobot(5, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.placeRobot(4, startPoseRobot4);
		tec.placeRobot(5, startPoseRobot5);
		


		Pose startPoseGoal1 = new Pose(16.0,25.0,0.0);
		Pose startPoseGoal2 = new Pose(25.0,7.0,0.0);
		Pose startPoseGoal3 = new Pose(4.0,8.0,0.0);
		
		
		Pose goalPoseRobot1 = new Pose(16.0,15.0,Math.PI/4);
		Pose goalPoseRobot2 = new Pose(27.0,3.0,-Math.PI/4);
		Pose goalPoseRobot3 = new Pose(21.0,3.0,-Math.PI/2);
		
		
		Task task1 = new Task(startPoseGoal1,goalPoseRobot1,1);
		Task task2 = new Task(startPoseGoal2,goalPoseRobot2,1);
		Task task3 = new Task(startPoseGoal3,goalPoseRobot3,1);

		
		
		Pose startPoseGoal4 = new Pose(8.0,16.0,-Math.PI/2);
		Pose endPoseGoal4 = new Pose(12.0,20.0,-Math.PI/2);
		Pose startPoseGoal5 = new Pose(-7.0,-7.0,Math.PI/2);
		Pose endPoseGoal5 = new Pose(-15.0,-8.0,Math.PI/2);
		Task task4 = new Task(startPoseGoal4,endPoseGoal4,1);
		Task task5 = new Task(startPoseGoal5,endPoseGoal5,1);


		int numRobot = 5;
		double alpha = 1;
	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		TaskAssignment ll = new TaskAssignment();
		ll.addTask(task1);
		ll.addTask(task2);
		ll.addTask(task3);
		ll.addTask(task4);
		ll.addTask(task5);
		
		ll.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		
		
		ll.instantiateFleetMaster(0.1, false);
		ll.startTaskAssignment(numRobot,rsp, false, tec);
		
		
		
		Pose startPoseGoal1new = new Pose(18.0,12.0,0.0);
		Pose startPoseGoal2new = new Pose(30.0,9.0,0.0);
		Pose startPoseGoal3new = new Pose(4.0,4.0,0.0);
		
		Pose newPoseGoal1 = new Pose(20.0,15.0,0.0);
		Pose newPoseGoal2 = new Pose(32.0,10.0,0.0);
		Pose newPoseGoal3 = new Pose(8.0,10.0,0.0);
		Task task1new = new Task(startPoseGoal1new,newPoseGoal1,1);
		Task task2new = new Task(startPoseGoal2new,newPoseGoal2,1);
		Task task3new = new Task(startPoseGoal3new,newPoseGoal3,1);
		
		
		
		
		
		Thread.sleep(2000);
		System.out.print("new Task add");
		ll.addTask(task1new);
		ll.addTask(task2new);
		ll.addTask(task3new);
		
		
		
	}
}
