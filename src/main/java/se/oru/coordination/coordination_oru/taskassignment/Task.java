package se.oru.coordination.coordination_oru.taskassignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.sat4j.sat.SolverController;
import org.sat4j.sat.visu.SolverVisualisation;

import com.vividsolutions.jts.geom.Coordinate;

import aima.core.agent.Model;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;
import com.google.ortools.linearsolver.*;
import com.google.ortools.linearsolver.MPSolver.OptimizationProblemType;
import com.google.ortools.linearsolver.MPSolver.ResultStatus;
import com.google.ortools.linearsolver.PartialVariableAssignment;
import com.google.ortools.constraintsolver.Solver;
import com.google.ortools.constraintsolver.SolverParameters;
import com.google.ortools.constraintsolver.Solver;
import com.google.ortools.*;
import com.google.ortools.sat.*;

public class Task {
	
	protected PoseSteering StartPoint;
	protected PoseSteering GoalPoint;
	protected int decidedRobotID = -1;
	protected List<PoseSteering[]> paths = null;
	protected Set<Integer> robotTypes = null;
	protected double deadline = -1;
	protected double operationTime = 0;
	
	
	
	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 * @param deadline -> deadline of Task 
	 * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (Pose StartPose, Pose GoalPose, int ... robotTypes) {
		this(-1,new PoseSteering(StartPose, 0.0), new PoseSteering(GoalPose, 0.0),robotTypes);
	}

	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 * @param deadline -> deadline of Task 
	 * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (double deadline,Pose StartPose, Pose GoalPose, int ... robotTypes) {
		this(deadline,new PoseSteering(StartPose, 0.0), new PoseSteering(GoalPose, 0.0),robotTypes);
	}

	
	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	  * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param deadline -> deadline of Task
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (PoseSteering StartPose, PoseSteering GoalPose, int ... robotTypes) {
		this.StartPoint = StartPose;
		this.GoalPoint = GoalPose;

		if (robotTypes.length == 0) throw new Error("Need to specifiy at least one robot type!");
		this.robotTypes = new HashSet<Integer>();
		for (int rt : robotTypes) this.robotTypes.add(rt);
	}
	
	/**
 	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	  * @param StartPose -> Task Starting Position
	 * @param GoalPose -> Task Ending Position
	 * @param deadline -> deadline of Task
	 * @param robotTypes -> robot type that can execute this task
	 */
	public Task (double deadline, PoseSteering StartPose, PoseSteering GoalPose, int ... robotTypes) {
		this.StartPoint = StartPose;
		this.GoalPoint = GoalPose;
		this.deadline = deadline;
		if (robotTypes.length == 0) throw new Error("Need to specifiy at least one robot type!");
		this.robotTypes = new HashSet<Integer>();
		for (int rt : robotTypes) this.robotTypes.add(rt);
	}

	public List<PoseSteering[]> getPaths() {
		return paths;
	}
	
	public void setPaths(PoseSteering[] ... newPaths) {
		if (this.paths == null) this.paths = new ArrayList<PoseSteering[]>();
		for (int i = 0; i < newPaths.length-1; i++) {
			//if (!(newPaths[i][newPaths[i].length-1].equals(newPaths[i+1][0]))) throw new Error("Teletransport not supported yet!");
			if (!(ArrayUtils.isEquals(newPaths[i][newPaths[i].length-1].getPose().toString(),newPaths[i+1][0].getPose().toString()))) throw new Error("Teletransport not supported yet!");
			this.paths.add(newPaths[i]);
		}
		this.paths.add(newPaths[newPaths.length-1]);
	}
	
	public int getNumPaths() {
		if (this.paths == null) return 0;
		return this.paths.size();
	}
	
	public Pose getStartPose() {
		return this.StartPoint.getPose();		
	}
	
	public Pose getGoalPose() {
		return this.GoalPoint.getPose();	
	}

	public PoseSteering getStart() {
		return this.StartPoint;		
	}
	
	public PoseSteering getGoal() {
		return this.GoalPoint;	
	}


	public boolean isCompatible(Robot robot) {
		return this.robotTypes.contains(robot.getRobotType());
	}
	
	public boolean isTaskAssigned() {
		return this.decidedRobotID != -1;
		//return this.taskIsAssigned;
	}

	public boolean isDeadlineSpecified() {
		return this.deadline != -1;
		//return this.taskIsAssigned;
	}
	
	
	public void setDeadline(double deadline) {
		this.deadline = deadline;
	}
	
	public double getDeadline() {
		return this.deadline;
	}
	
	public void setOperationTime(double operationTime) {
		this.operationTime = operationTime;
	}
	
	public double getOperationTime() {
		return this.operationTime;
	}
	
	public void getInfo() {
		System.out.println("Starting Pose -> " +this.StartPoint + "\n Goal Pose ->"+ this.GoalPoint + "\n Robot Types ->"+ this.robotTypes
				+"\n Task is Assigned "+ this.isTaskAssigned());
	}
	
	public void assignRobot(int robotID) {
		this.decidedRobotID = robotID;
	}

	public Mission[] getMissions() {
		if (this.paths == null) throw new Error("No paths specified!");
		if (this.decidedRobotID == -1) throw new Error("No robot assigned!");
		Mission[] ret = new Mission[paths.size()];
		for (int i = 0; i < paths.size(); i++) {
			ret[i] = new Mission(this.decidedRobotID, paths.get(i));
			if (i == 0) ret[i].setFromLocation("Init for Robot" + this.decidedRobotID);
			else if (i ==  paths.size()-1) ret[i].setFromLocation("Goal for Robot" + this.decidedRobotID);
			else ret[i].setFromLocation("Waypoint " + i + " for Robot" + this.decidedRobotID);
		}
		return ret;
		//Now you can do this: Missions.enqueue(task.getMissions());
	}
	
	
	
}
