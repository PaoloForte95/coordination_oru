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
public class TaskAssignmentOrebroWarehouse {
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
		Coordinate footprint2 = new Coordinate(0.5,0.5);
		Coordinate footprint3 = new Coordinate(0.5,-0.5);
		Coordinate footprint4 = new Coordinate(-0.5,-0.5);
		
		Coordinate[] fp2 = new Coordinate[] {
				new Coordinate(-1.0,0.0),
				new Coordinate(0.0,1.0),
				new Coordinate(1.0,0.0),
				new Coordinate(-0.0,-1.0),
		};
		
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(15, 0, 0);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);

		viz.setMap("maps/icra2016-basement-mod.yaml");
		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp2 = new ReedsSheppCarPlanner();
		rsp2.setRadius(0.2);
		rsp2.setFootprint(footprint1,footprint2,footprint3,footprint4);
		rsp2.setTurningRadius(4.0);
		rsp2.setDistanceBetweenPathPoints(0.5);
		rsp2.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/icra2016-basement-mod.yaml"));
		double res = Double.parseDouble(Missions.getProperty("resolution", "maps/icra2016-basement-mod.yaml"));
		//rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/map-corridors-vi.yaml"));
		//double res = Double.parseDouble(Missions.getProperty("resolution", "maps/map-corridors-vi.yaml"));
		rsp2.setMapResolution(res);
		rsp2.setPlanningTimeInSecs(5);
		
		Pose startPoseRobot1 = new Pose(33.0,6.0,Math.PI);
		Pose startPoseRobot2 = new Pose(3.0,28.0,0.0);
		Pose startPoseRobot3 = new Pose(3.0,20.0,0.0);
		Pose startPoseRobot4 = new Pose(3.0,25.0,0.0);
		Pose startPoseRobot5 = new Pose(8.0,1.0,Math.PI/2);	
		Pose startPoseRobot6 = new Pose(11.0,1.0,Math.PI/2);	
		Pose startPoseRobot7 = new Pose(20.0,1.0,Math.PI/2);	
		
		Robot robot1 = new Robot(1, 1 );
		Robot robot2 = new Robot(2,2,fp2 );
		Robot robot3 = new Robot(3,1 );
		Robot robot4 = new Robot(4,1);
		Robot robot5 = new Robot(5,1);
		
		Robot robot6 = new Robot(6,2,fp2);
		Robot robot7 = new Robot(7,1);
		
		
		tec.setFootprint(robot2.getRobotID(),fp2);
		tec.setFootprint(robot6.getRobotID(),fp2);
		
		Pose startPoseGoal1 = new Pose(7.0,4.0,Math.PI/2);	
		Pose startPoseGoal2 = new Pose(13.0,20.0,0.0);
		Pose startPoseGoal3 = new Pose(13.0,23.0,0.0);
		Pose startPoseGoal4 = new Pose(13.0,20.0,0.0);
		Pose startPoseGoal5 = new Pose(13.0,20.0,0.0);
		Pose startPoseGoal6 = new Pose(11.0,5.0,Math.PI/2);	
		Pose startPoseGoal7 = new Pose(18.0,5.0,Math.PI/2);	
		
		Pose goalPoseRobot1 = new Pose(7.0,15.5,Math.PI/2);
		Pose goalPoseRobot2 = new Pose(25.0,18.0,0.0);
		Pose goalPoseRobot3 = new Pose(22.0,23.0,0.0);
		Pose goalPoseRobot4 = new Pose(31.5,6.0,Math.PI/2);
		Pose goalPoseRobot5 = new Pose(26.0,15.0,0.0);
		Pose goalPoseRobot6 = new Pose(3.0,23.0,0.0);
		Pose goalPoseRobot7 = new Pose(25.0,10.0,0.0);
		
		tec.addRobot(robot1,startPoseRobot1);
		tec.addRobot(robot2,startPoseRobot2);
		tec.addRobot(robot3,startPoseRobot3);
		tec.addRobot(robot4,startPoseRobot4);
		tec.addRobot(robot5,startPoseRobot5);
		tec.addRobot(robot6,startPoseRobot6);
		tec.addRobot(robot7,startPoseRobot7);
		
		
		
		
		
		Task task1 = new Task(1,startPoseGoal1,goalPoseRobot1,1);
		Task task2 = new Task(2,startPoseGoal2,goalPoseRobot2,1);
		Task task3 = new Task(3,startPoseGoal3,goalPoseRobot3,1);
		Task task4 = new Task(4,startPoseGoal4,goalPoseRobot4,2);
		Task task5 = new Task(5,startPoseGoal5,goalPoseRobot5,1);
		Task task6 = new Task(4,startPoseGoal6,goalPoseRobot6,2);
		Task task7 = new Task(5,startPoseGoal7,goalPoseRobot7,1);

	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		double alpha = 1.0;
		TaskAssignmentSimple assignmentProblem = new TaskAssignmentSimple();
		assignmentProblem.addTask(task1);
		assignmentProblem.addTask(task2);
		assignmentProblem.addTask(task3);
		assignmentProblem.addTask(task4);
		assignmentProblem.addTask(task5);
		assignmentProblem.addTask(task6);
		assignmentProblem.addTask(task7);
		assignmentProblem.setFleetVisualization(viz);
		
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.setDefaultMotionPlanner(rsp2);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.setLinearWeight(alpha);
		//assignmentProblem.setCostFunctionsWeight(0.8, 0.1, 0.1);
	
		tec.setDefaultMotionPlanner(assignmentProblem.getDefaultMotionPlanner());
		
		assignmentProblem.startTaskAssignment(tec);
		//assignmentProblem.startTaskAssignmentGreedyAlgorithm(tec);
	}
}
