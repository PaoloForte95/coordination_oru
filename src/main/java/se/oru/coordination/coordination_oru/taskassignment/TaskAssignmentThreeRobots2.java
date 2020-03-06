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
		Pose goalPoseRobot1 = new Pose(16.0,15.0,Math.PI/4);
		Pose startPoseRobot2 = new Pose(6.0,16.0,-Math.PI/4);
		Pose goalPoseRobot2 = new Pose(25.0,3.0,-Math.PI/4);
		Pose startPoseRobot3 = new Pose(9.0,6.0,Math.PI/2);
		Pose goalPoseRobot3 = new Pose(21.0,3.0,-Math.PI/2);
		
		
		Pose startPoseRobot4 = new Pose(16.0,30.0,-Math.PI/2);
		Pose startPoseRobot5 = new Pose(-5.0,-5.0,Math.PI/2);
		
		
		
		Pose startPoseGoal1 = new Pose(16.0,25.0,0.0);
		Pose startPoseGoal2 = new Pose(25.0,7.0,0.0);
		Pose startPoseGoal3 = new Pose(4.0,8.0,0.0);
		

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, startPoseRobot3);
		
		//tec.addRobot(4, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		//tec.addRobot(5, 1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		//tec.placeRobot(4, startPoseRobot4);
		//tec.placeRobot(5, startPoseRobot5);
		
		
		ArrayList <Task> Tasks2 = new ArrayList <Task>();
		
		Task task1 = new Task(startPoseGoal1,goalPoseRobot1,1);
		Task task2 = new Task(startPoseGoal2,goalPoseRobot2,1);
		Task task3 = new Task(startPoseGoal3,goalPoseRobot3,1);
		Tasks2.add(task1);
		Tasks2.add(task2);
		Tasks2.add(task3);
		
		
		Task task4 = new Task(startPoseRobot4,startPoseGoal1,1);
		Task task5 = new Task(startPoseRobot5,startPoseGoal3,1);
		Tasks2.add(task4);
		Tasks2.add(task5);

		int num_robot = 3;
		double alpha = 1;
	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		TaskAssignment ll = new TaskAssignment();
		ll.instantiateFleetMaster(0.1, false);
		MPSolver solver = ll.buildOptimizationProblemWithB(num_robot, rsp, Tasks2, false, tec);
		double [][] prova3 = ll.solveOptimizationProblem(rsp, Tasks2,solver,tec,alpha);	
		//double [][] prova3 = TaskAssignment.solve_optimization_problem_exact(num_robot, num_task, rsp, startPose, goalPose,solver,tec,alpha);
		
		for (int i = 0; i < prova3.length; i++) {
			for (int j = 0; j < prova3[0].length; j++) {
					//System.out.println("aaaaaaaaaa>> "+ prova5[i][j]+"i>> "+i+" j>> "+j);
					System.out.println("cccccccccc>> "+prova3[i][j]+" i>> "+i+" j>> "+j);			
			} 
		}
		
	    ll.Task_Assignment(prova3, rsp, Tasks2, tec);	
	}
}
