package se.oru.coordination.coordination_oru.taskassignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;

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
	
	protected Pose StartPoint;
	protected Pose GoalPoint;
	protected int taskType;
	protected boolean taskIsAssigned;
	
	
	/**
	 * Constructor. Generate a Task with a Starting Pose and an Ending Pose; the type is used to evaluate which 
	 * robot can perform this task
	 * @param StartPose -> Starting Pose for Task;
	 * @param GoalPose -> Ending Pose for Task
	 * @param taskType -> The type of the Task expressed as int
	 */
	public Task (Pose StartPose,Pose GoalPose,int taskType) {
		this.StartPoint = StartPose;
		this.GoalPoint = GoalPose;
		this.taskType = taskType;
		
		
	}

	public Pose getStartPose() {
		return this.StartPoint;
		
	}
	public Pose getGoalPose() {
		return this.GoalPoint;
		
	}
	
	public int getTaskType() {
		return this.taskType;
	}
	
	
	public boolean getTaskIsAssigned() {
		return this.taskIsAssigned;
	}
	
	public void setTaskIsAssigned(boolean taskIsAssigned) {
		this.taskIsAssigned = taskIsAssigned;
	
	}
	
	public void getInfo() {
		System.out.println("Starting Pose -> " +this.StartPoint + "\n Goal Pose ->"+ this.GoalPoint + "\n Task Type ->"+ this.taskType
				+"Task is Assigned "+ this.taskIsAssigned);
	}

	
	
	
}
