package se.oru.coordination.coordination_oru.taskassignment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintStream;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.sat4j.sat.SolverController;
import org.sat4j.sat.visu.SolverVisualisation;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import aima.core.agent.Model;
import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.IndexedDelay;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.CumulatedIndexedDelaysList;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.icaps2018.eval.TrajectoryEnvelopeCoordinatorSimulationICAPS;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
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
import se.oru.coordination.coordination_oru.taskassignment.Task;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment.SortByDeadline;





public class TaskAssignmentSimple{
	//Optimization Problem Parameters
	protected int numRobot;
	protected int numTask;
	protected int dummyRobot;
	protected int dummyTask;
	protected int numRobotAug;
	protected int numTaskAug;
	protected double linearWeight = 1;
	//Parameters of weights in Optimization Problem
	protected double pathLengthWeight = 1;
	protected double arrivalTimeWeight = 0;
	protected double tardinessWeight = 0;
	protected double [][] costValuesMatrix;
	
	
	protected ArrayList <Task> taskQueue = new ArrayList <Task>();
	//Number of Idle Robots
	protected ArrayList <Integer> IDsIdleRobots = new ArrayList <Integer>();
	//Path and arrival Time Parameters
	//Infinity cost if path to reach a goal note exists
	protected double MaxPathLength = 10000000;
	//This is the sum of max path length for each robot
	protected double sumMaxPathsLength = 1;
	//This is the sum of arrival time considering max path length for each robot
	protected double sumArrivalTime = 1;
	//Parameters of mininum Velocity and Acceleration considering all robots
	protected double minMaxVel;
	protected double minMaxAcc;
	//This is the sum of all tardiness 
	protected double sumTardiness = 1;
	
	
	//Motion planner and Coordinator Parameters
	protected AbstractTrajectoryEnvelopeCoordinator coordinator;
	protected AbstractMotionPlanner defaultMotionPlanner = null;
	protected HashMap<Integer,AbstractMotionPlanner> motionPlanners = new HashMap<Integer, AbstractMotionPlanner>();
	
	
	//Time required by function parameters
	protected long timeRequiretoEvaluatePaths;
	protected long timeRequiretofillInPall;
	protected long timeRequiretoComputeCriticalSection;
	protected long timeRequiretoComputePathsDelay;
	
	protected ArrayList<PoseSteering> pathsToTargetStart = new ArrayList <PoseSteering>();
	protected ArrayList <PoseSteering[]> pathsToTargetGoal = new ArrayList <PoseSteering[]>();
	protected ArrayList <SpatialEnvelope> pathsDrivingRobot = new ArrayList <SpatialEnvelope>();
	//FleetMaster Interface Parameters
	
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	protected boolean propagateDelays = false;
	protected static Logger metaCSPLogger = MetaCSPLogging.getLogger(TrajectoryEnvelopeCoordinator.class);
	
	
	//Task Allocation Thread Parameters 
	protected int CONTROL_PERIOD_Task = 15000;
	public static int EFFECTIVE_CONTROL_PERIOD_task = 0;
	
	protected FleetVisualization viz = null;

	
	/**
	 * Set a motion planner to be used for re-planning for a specific
	 * robot.
	 * @param robotID The robot for which the given motion planner should be used.
	 * @param mp The motion planner that will be called for re-planning.
	 */
	public void setMotionPlanner(int robotID, AbstractMotionPlanner mp) {
		this.motionPlanners.put(robotID, mp);
	}
	
	
	/**
	 * Get the motion planner used for re-planning for a specific robot.
	 * @param robotID The ID of a robot.
	 * @return The motion planner used for re-planning for the given robot.
	 */
	public AbstractMotionPlanner getMotionPlanner(int robotID) {
		return this.motionPlanners.get(robotID);
	}

	
	
	/**
	 * Set the weights of cost functions  in Optimization Problem. THese must be numbers between 0 and 1.
	 * @param The path length weight;
	 * @param The arrival time weight;
	 * @param The tardiness weight
	 */
	
	public void setCostFunctionsWeight(double pathLengthWeight,double arrivalTimeWeight,double tardinessWeight) {
		if(pathLengthWeight <0|| arrivalTimeWeight < 0 ||  tardinessWeight < 0) {
			throw new Error("Weights cannot be  numbers less than 0!");
		}
		double sumWeight = pathLengthWeight +arrivalTimeWeight + tardinessWeight;
		if(sumWeight != 1 || sumWeight < 0 ) {
			throw new Error("Weights sum must be equal to 1!");
		}
		this.pathLengthWeight = pathLengthWeight;
		this.arrivalTimeWeight = arrivalTimeWeight;
		this.tardinessWeight = tardinessWeight;
	}
	
	
	/**
	 * Set the linear weight used in Optimization Problem
	 * @param viz -> Visualization to use 
	 */
	
	public void setLinearWeight(double linearWeight) {
		this.linearWeight = linearWeight;
	}
	
	/**
	 * Set the Fleet Visualization 
	 * @param viz -> Visualization to use 
	 */
	
	public void setFleetVisualization(FleetVisualization viz) {
		this.viz = viz;
	}
	
	
	/**
	 * Set the Coordinator 
	 * @param viz -> Visualization to use 
	 */
	
	public void setCoordinator(AbstractTrajectoryEnvelopeCoordinator coordinator) {
		this.coordinator = coordinator;
	}
	
	
	/**
	 * Get the Coordinator 
	 * @param viz -> Visualization to use 
	 */
	
	public AbstractTrajectoryEnvelopeCoordinator getCoordinator() {
		return this.coordinator;
	}
	
	
	/**
	 * Check if a goal can be reached by at least one robot of the Fleet 
	 * @param PAll -> Initial PAll
	 * @return PAll incremented a task cannot be reach by any robot
	 */
	
	private  double [][] checkTargetGoals (double [][] PAll){
		for (int j= 0; j< PAll[0].length ; j++) {
			boolean targetEndCanBeReach = false;
			for (int i = 0; i < PAll.length; i++) {
				if(PAll[i][j] != MaxPathLength) {
					targetEndCanBeReach = true;
				}
				
			}
			//no robot can reach the target end -> need to introduce a dummy task and robot 
			if(!targetEndCanBeReach) {
				dummyRobot += 1 ;
				dummyTask += 1 ;
				numRobotAug += 1;
				numTaskAug += 1;
	
			}	
		}
		
		double [][] PAllAug = new double [numRobotAug][numTaskAug];
		for(int i = 0;i < numRobotAug; i++) {
			for(int j=0; j<  numTaskAug;j++) {
				if(i < PAll.length && j< PAll[0].length) {
					PAllAug[i][j] = PAll[i][j];
				}else {
					PAllAug[i][j] = 1;
					pathsToTargetGoal.add(i*numTaskAug + j, null);
			
				}	
			}
		}
		return PAllAug;
	}
	
	
	
	/**
	 * The default footprint used for robots if none is specified.
	 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
	 */
	public static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			new Coordinate(-1.7, 0.7),	//back left
			new Coordinate(-1.7, -0.7),	//back right
			new Coordinate(2.7, -0.7),	//front right
			new Coordinate(2.7, 0.7)	//front left
	};

	/**
	 * Set the minimum values of maximum velocity and acceleration considering all robots of the fleet
	 * @param MaxVel -> minimum of maximum velocity of all robot models;
	 * @param MaxAccel -> minimum of maximum acceleration of all robot models;
	 */
	
	public void setminMaxVelandAccel(double MaxVel,double MaxAccel) {
		this.minMaxVel = MaxVel;
		this.minMaxAcc = MaxAccel;
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * Note: this function should be called before placing the first robot.
	 * ATTENTION: If dynamic_size is <code>false</code>, then the user should check that all the paths will lay in the given area.
	 * @param origin_x The x coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta coordinate (in rads) of the lower-left pixel map (counterclockwise rotation). Many parts of the system currently ignore it.
	 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
	 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
	 * @param width Number of columns of the map (>= 1) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (>= 1) if dynamic sizing is not enabled.
	 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 * @param debug If <code>true</code>, it enables writing to screen debugging info.
	 */
	public void instantiateFleetMaster(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean propagateDelays, boolean debug) {
		this.fleetMasterInterface = new FleetMasterInterface(origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, debug);
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
		this.propagateDelays = propagateDelays;
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time
	 * while minimizing the computational requirements (bounding box are used to set the size of each path-image).
	 * Note: this function should be called before placing the first robot.
	 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
	 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 */
	public void instantiateFleetMaster(double resolution, boolean propagateDelays) {
		this.fleetMasterInterface = new FleetMasterInterface(0., 0., 0., resolution, 100, 100, true, false);
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
		this.propagateDelays = propagateDelays;
	}
	
	
	/**
	 * Add a path to the fleetmaster interface
	 * @param robotID -> The ID of the robot
	 * @param pathID -> the ID of the path
	 * @param pss -> the path expressed as a PoseSteering vector
	 * @param boundingBox -> the bounding box of the path
	 * @param coordinates -> footprint of the robot 
	 */
	protected void addPath(int robotID, int pathID, PoseSteering[] pss, Geometry boundingBox, Coordinate... coordinates) {
		if (!fleetMasterInterface.addPath(robotID, pathID, pss, boundingBox, coordinates)) 
			metaCSPLogger.severe("Unable to add the path to the fleetmaster gridmap. Check if the map contains the given path.");
	}
	
	
	
	/**
	 * Delete the path from the fleetmaster interface
	 * @param pathID -> The ID of the path to remove 
	 */
	protected void removePath(int pathID){
		if (!fleetMasterInterface.clearPath(pathID)) 
			metaCSPLogger.severe("Unable to remove the path to the fleetmaster gridmap. Check if the map contains the given path.");
	}
	
	
	protected CumulatedIndexedDelaysList toIndexedDelaysList(TreeSet<IndexedDelay> delays, int max_depth) {
		//Handle exceptions
		if (delays == null) {
			metaCSPLogger.severe("Invalid input in function toPropagationTCDelays!!");
			throw new Error("Invalid input in function toPropagationTCDelays!!");
		}
		if (delays.isEmpty() || max_depth < 1) return new CumulatedIndexedDelaysList();
			
		//Cast the type
		ArrayList<Long> indices = new ArrayList<Long>();
		ArrayList<Double> values = new ArrayList<Double>();
		Iterator<IndexedDelay> it = delays.descendingIterator();
		IndexedDelay prev = delays.last();
		while (it.hasNext()) {
			IndexedDelay current = it.next();
			//Check unfeasible values
			if (current.getValue() == Double.NaN) {
				metaCSPLogger.severe("NaN input in function toPropagationTCDelays!!");
				throw new Error("NaN input in function toPropagationTCDelays!!");
			}
			if (current.getValue() == Double.NEGATIVE_INFINITY) {
				metaCSPLogger.severe("-Inf input in function toPropagationTCDelays!!");
				throw new Error("-Inf input in function toPropagationTCDelays!!");
			}
			if (prev.getIndex() < current.getIndex()) {
				metaCSPLogger.severe("Invalid IndexedDelays TreeSet!!");
				throw new Error("Invalid IndexedDelays TreeSet!!");
			}
			
			//Update the value only if positive and only if the index is lower than the max depth
			if (current.getValue() > 0 && current.getValue() < Double.MAX_VALUE && current.getIndex() < max_depth) {
				if (values.size() == 0) {
					//Add the index the first time its value is positive
					indices.add(new Long(current.getIndex()));
					values.add(current.getValue());
				}
				else if (prev.getIndex() == current.getIndex())				
					//Handle multiple delays in the same critical point
					values.set(values.size()-1, values.get(values.size()-1) + current.getValue());
				else {
					//Add the cumulative value if it is not the first.
					indices.add(new Long(current.getIndex()));
					values.add(values.get(values.size()-1) + current.getValue());
				}
			}
			prev = current;
		}
		CumulatedIndexedDelaysList propTCDelays = new CumulatedIndexedDelaysList();
		if (indices.size() > 0) {
			propTCDelays.size = indices.size();
			propTCDelays.indices = ArrayUtils.toPrimitive((Long[]) indices.toArray(new Long[indices.size()]));
			ArrayUtils.reverse(propTCDelays.indices);
			propTCDelays.values = ArrayUtils.toPrimitive((Double[]) values.toArray(new Double[values.size()]));
			ArrayUtils.reverse(propTCDelays.values);
		}
		return propTCDelays;
	}
	
	
	protected Pair<Double,Double> estimateTimeToCompletionDelays(int path1ID,PoseSteering[] pss1, TreeSet<IndexedDelay> delaysRobot1, int path2ID,PoseSteering[] pss2, TreeSet<IndexedDelay> delaysRobot2, CriticalSection cs) {
		
		if (this.fleetMasterInterface != null && fleetMasterInterface.checkPathHasBeenAdded(path1ID)&& fleetMasterInterface.checkPathHasBeenAdded(path2ID)) {
			CumulatedIndexedDelaysList te1TCDelays = toIndexedDelaysList(delaysRobot1, pss1.length);
			//metaCSPLogger.info("[estimateTimeToCompletionDelays] te1TCDelays: " + te1TCDelays.toString());
			CumulatedIndexedDelaysList te2TCDelays = toIndexedDelaysList(delaysRobot2, pss2.length);
			//metaCSPLogger.info("[estimateTimeToCompletionDelays] te2TCDelays: " + te2TCDelays.toString());
			return fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays,pss1,pss2);
		}
		
		return new Pair<Double, Double> (Double.NaN, Double.NaN);
	}
	
	
	/**
	 * Set a motion planner to be used for planning for all robots
	 * @param mp The motion planner that will be called for planning
	 */
	public void setDefaultMotionPlanner(AbstractMotionPlanner mp) {
		this.defaultMotionPlanner = mp;
	}
	
	
	/**
	 * Get the motion planner to be used for planning for all robots.
	 * @return The motion planner used for planning for all robots
	 */
	public AbstractMotionPlanner getDefaultMotionPlanner() {
		return this.defaultMotionPlanner;
	}
	
	
	/**
	 * Add a Task to Mission set
	 * @param task -> the task to add
	 * @return -> true if task is added correctly, otherwise false
	 */
	public boolean addTask(Task task) {
		if (task == null) {
			metaCSPLogger.severe("No task to add. Please give a correct Task.");
			throw new Error("Cannot add the task");
		}
		boolean TaskisAdded = taskQueue.add(task);
		return TaskisAdded;
	}
	
	/**
	 * Remove a task from the queue
	 * @param task The task to remove
	 * @return <code>true</code> if task is removed correctly, otherwise false
	 */
	public boolean removeTask(Task task) {
		return taskQueue.remove(task);
	}
	
	
	
	/**
	 * Get the task from Mission set in the index position
	 * @param index -> the index position of the task 
	 * @return -> the task in index position
	 */
	public Task getTask(int index) {
		if (index < 0 || index > taskQueue.size()) {
			metaCSPLogger.severe("Wrong index.");
			throw new Error("The task" + index + "not exist");
		}else {
			return taskQueue.get(index);
		}
	}
	
	class SortByDeadline implements Comparator<Task>{
		@Override
		public int compare(Task task1, Task task2) {
			return (int) (task1.getDeadline()-task2.getDeadline());
		}
	}
	
	private void checkOnTaskDeadline() {
		ArrayList <Task>taskArray =new ArrayList <Task>();
		for(int j=0; j < taskQueue.size(); j++ ) {
			taskArray.add(taskQueue.get(j));
		}
		taskArray.sort(new SortByDeadline());
		for(int k= 0;k < taskArray.size();k++) {
		}
		for(int i=0;i < IDsIdleRobots.size(); i++) {
				int index = taskQueue.indexOf(taskArray.get(i));
				if(taskQueue.get(index).getDeadline() != -1) {
					taskQueue.get(index).setPriority(true);
				}
			
		}
	}
	
	/**
	 * Compute the number of Dummy Robots and/or Tasks. Consider the possibility to have a different number of robots (N) and tasks (M). If N > M, dummy tasks are 
	 * considered, where a dummy task is a task for which a robot stay in starting position; while if M > N dummy robots
	 * are considered, where a dummy robot is only a virtual robot. 
	 * @param numRobot Number of robots
	 * @param numTasks Number of tasks
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 */
	private void dummyRobotorTask(int numRobot, int numTasks,AbstractTrajectoryEnvelopeCoordinator tec) {
		numRobotAug = numRobot;
		numTaskAug = numTasks;
		//Restore initial value for dummy robot and task
		dummyTask = 0;
		dummyRobot = 0;
		//Considering the possibility to have n != m
		//If n > m -> we have dummy robot, so at some robot is assign the task to stay in starting position
		if (numRobot > numTasks) {
			dummyTask = numRobot - numTasks;
			numTaskAug = numTasks + dummyTask;
		}
		else if (numRobot < numTasks) {
			dummyRobot = numTasks - numRobot;
			numRobotAug = numRobot + dummyRobot;
		}
		//This second check is used when we have particular cases due to Robot Type and Task Type
		//Only if we have already a dummy task this check can be avoided
		//If A robot cannot be assigned to any task 
		if (dummyTask == 0 || dummyRobot != 0) {
			for (int i = 0; i < numRobot; i++) {
				boolean flagAllocateRobot = false;
				 for (int j = 0; j < numTasks; j++) {
					 //check if robot can be assigned to one task
					 //if (taskQueue.get(j).getTaskType() == tec.getRobotType(IDsIdleRobots[i])) {
					 if (taskQueue.get(j).isCompatible(tec.getRobot(IDsIdleRobots.get(i)))) {
						 flagAllocateRobot = true;
						 
					 }
				 }
				 //the robot cannot be assigned to any task -> add a dummy robot and task
				 if (!flagAllocateRobot) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
		//Only if we have already a dummy robot this check can be avoided
		//If A task cannot be assigned to any robot
		if (dummyRobot == 0 || dummyTask != 0) {
			for (int i = 0; i < numTasks; i++) {
				boolean flagAllocateTask = false;
				 for (int j = 0; j < numRobot; j++) {
					//check if task can be assigned to one robot
					 //if (taskQueue.get(i).getTaskType() == tec.getRobotType(IDsIdleRobots[j])) {
					 if (taskQueue.get(i).isCompatible(tec.getRobot(IDsIdleRobots.get(j)))) {
						 flagAllocateTask = true;
					 }
				 }
				 //the task cannot be assigned to any robot -> add a dummy robot and task
				 if (!flagAllocateTask) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
	}

	/**
	 * Transform a 1D array of MPVariable into a 2D MATRIX  
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblem}, {@link #buildOptimizationProblemWithB}  or {@link #buildOptimizationProblemWithBNormalized}
	 * @return 2D Matrix of Decision Variable of the input problem
	 */
	private MPVariable [][] tranformArray(MPSolver optimizationProblem) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][] decisionVariable = new MPVariable [numRobotAug][numTaskAug];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 decisionVariable[i][j] = array1D[i*numTaskAug+j];
			 }
	    }
		return decisionVariable;
	}
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem},{@link #buildOptimizationProblemWithB} or {@link #buildOptimizationProblemWithBNormalized} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on previous optimal solution found  
	 */
	private MPSolver constraintOnPreviousSolution(MPSolver optimizationProblem, double [][] assignmentMatrix) {
		//Take decision Variable from Optimization Problem
		MPVariable [][] DecisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    				if (assignmentMatrix[i][j] >0) {
	    				c2.setCoefficient(DecisionVariable[i][j],1);
	    			}else {
	    				c2.setCoefficient(DecisionVariable[i][j],0);
	    			}
    			}		
		 	}
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}
	
	
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution cost in order to not consider more solution that has a cost higher
	 * than this. In this manner is possible to avoid some cases.
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on optimal solution cost 
	 */
	private MPSolver constraintOnCostSolution(MPSolver optimizationProblem,double objectiveValue) {
		//Take the vector of Decision Variable from the input solver
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		MPConstraint c3 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,objectiveValue);
		//Define a constraint for which the next optimal solutions considering only B must have a cost less than objectiveValue
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			c3.setCoefficient(decisionVariable[i][j],costValuesMatrix[i][j]);
    			}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}

	/**
	 * Store the solution of a optimization problem in a Matrix 
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> A solved optimization problem
	 * @return Assignment matrix for the optimization problem given as input
	 */
	
	private double [][] saveAssignmentMatrix(int numRobot,int numTasks,MPSolver optimizationProblem){
		//Take the decision variable from the optimization problem
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem);
		double [][] assignmentMatrix = new double [numRobot][numTasks];	
		//Store decision variable values in a Matrix
		for (int i = 0; i < numRobot; i++) {
			for (int j = 0; j < numTasks; j++) {
				assignmentMatrix[i][j] = decisionVariable[i][j].solutionValue();
			}
		}
		return assignmentMatrix;	
	}
	
	/**
	 * Evaluate the cost associated to the path length for the a couple of robot and task.
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param robot -> The i-th Robot
	 * @param task -> The j-th the Task
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private double evaluatePathLength(int robot , int task, AbstractTrajectoryEnvelopeCoordinator tec){
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the path length to infinity
		double pathLength = MaxPathLength;
		// Only for real robots and tasks
		if (robot <= numRobot && task < numTask) {
			//Take the state for the i-th Robot
			RobotReport rr = tec.getRobotReport(IDsIdleRobots.get(robot-1));
			if (rr == null) {
				metaCSPLogger.severe("RobotReport not found for Robot" + robot + ".");
				throw new Error("RobotReport not found for Robot" + robot + ".");
			}
			//Evaluate the path from the Robot Starting Pose to Task End Pose
			AbstractMotionPlanner rsp = getMotionPlanner(robot);
			if(rsp == null) { // if there is not  a specific motion planner for the robot
				rsp = getDefaultMotionPlanner();
			}
			rsp.setStart(rr.getPose());
			rsp.setGoals(taskQueue.get(task).getStartPose(),taskQueue.get(task).getGoalPose());
			rsp.setFootprint(tec.getFootprint(robot));
		
			if (!rsp.plan() ) {
				System.out.println("Robot" + robot +" cannot reach the Target End of Task " + (task+1));
				//the path to reach target end not exits
				pathsToTargetGoal.add(null);
				//Infinity cost is returned 
				return pathLength;
			}
			
			//If the path exists
			//Take the Pose Steering representing the path
			PoseSteering[] pss = rsp.getPath();
			
			//Add the path to the FleetMaster Interface -> this is necessary for F function
			addPath(robot, pss.hashCode(), pss, null, tec.getFootprint(robot));
			//Save the path to Task in the path set
			pathsToTargetGoal.add(pss);
			//Take the Path Length
			pathLength = Missions.getPathLength(pss);
			System.out.println("PRovaaaaLLLLLL"+ pathLength + "robot>> "+ robot + " task>>" +task);
		
			
		} else { //There also virtual robot and task are considered 
			//There are considered real robot and dummy task
			if (numRobot >= numTask && robot <= numRobot){ //dummy task -> The Robot receive the task to stay in starting position
				//The second condition is used in the special case in which we have that one robot cannot be 
				//assigned to any tasks due to its type, so we must add a dummy robot and a dummy task, but we 
				//Create the task to stay in robot starting position
				PoseSteering[] dummyTask = new PoseSteering[1];
				//Take the state for the i-th Robot
				RobotReport rr = tec.getRobotReport(IDsIdleRobots.get(robot-1));
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robot + ".");
					throw new Error("RobotReport not found for Robot" + robot + ".");
				}
				//take the starting position of the robot
				dummyTask[0] = new PoseSteering(rr.getPose(),0);
				//Add the path to the FleetMaster Interface -> so it can be considered as an obstacle from 
				//the motion planner
				addPath(robot, dummyTask.hashCode(), dummyTask, null, tec.getFootprint(robot));
				//Save the path to Dummy Task 
				pathsToTargetGoal.add(dummyTask);
				//Consider a minimal pathLength
				pathLength = 1;
				return pathLength;
			}
			else { //There are considered dummy robot and real task
				//dummy robot -> Consider a only virtual Robot 
				pathsToTargetGoal.add(null);
				pathLength = 1;
				return pathLength;
			}	
		}	
		//Return the cost of path length
		return pathLength;
	}
	
	
	/**
	 * Evaluate the PAll matrix, that is a matrix that contains all path for each possible combination of robot
	 * and task
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return The PAll matrix
	 */
	private double[][] evaluatePAll(AbstractTrajectoryEnvelopeCoordinator tec){
		
		PrintStream fileStream1 = null;
		PrintStream fileStream2 = null;
		try {
			fileStream1 = new PrintStream(new File("PathPlanner.txt"));
			fileStream2 = new PrintStream(new File("PAll.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		long timeInitial2 = 0;
		
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the sum of max paths lengths and time to do it for each robot
		//This cost are used then for normalizing cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;
		//Initialize PAll
		double [][] PAll = new double[numRobotAug][numTaskAug];
		
		
		for (int robot = 0; robot < numRobotAug; robot++) {
			double maxPathLength = 1;
			for (int task = 0; task < numTaskAug; task++ ) {
					
				//Take time to understand how much time require this function
				long timeInitial = Calendar.getInstance().getTimeInMillis();
				double pathLength = this.MaxPathLength;
				
				
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (task < numTask && robot < numRobot ) {
				 typesAreEqual = taskQueue.get(task).isCompatible(tec.getRobot(robot+1));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 if(typesAreEqual) { // only if robot and typoe have the same types
					 pathLength = evaluatePathLength(robot+1,task,tec);
					 //Take time to evaluate the path
					 
					 long timeFinal = Calendar.getInstance().getTimeInMillis();
					 long timeRequired = timeFinal- timeInitial;
					 timeRequiretoEvaluatePaths = timeRequiretoEvaluatePaths + timeRequired;
					 fileStream1.println(timeRequired+"");
					 timeInitial2 = Calendar.getInstance().getTimeInMillis();
						
					if ( pathLength > maxPathLength && pathLength != this.MaxPathLength) {
							maxPathLength = pathLength;
						}
				 }else {
					 pathsToTargetGoal.add(null);
				 }
				 PAll[robot][task] = pathLength;
				 
				//Take the time to fill in the PAll Matrix
				long timeFinal2 = Calendar.getInstance().getTimeInMillis();
				long timeRequired2 = timeFinal2- timeInitial2;
				timeRequiretofillInPall = timeRequiretofillInPall + timeRequired2;
				fileStream2.println(timeRequired2+"");
			}//For Task
			//Sum the max path length for each robot
			
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.minMaxVel,this.minMaxAcc);
		}
		double [][] PAllAug =  checkTargetGoals(PAll);
		//Save the sum of max paths length to normalize path length cost
		this.sumMaxPathsLength = sumPathsLength;
		//Save the sum of arrival time considering max paths length to normalize delay cost
		this.sumArrivalTime = sumArrivalTime;
		//Return the cost of path length
		
		
		 
		return PAllAug;
		}
	
	/**
	 * Evaluate the cost associated to time delay on completion of a task for a specific robot, due to interference with other robot
	 * and precedence constraints. The cost is evaluated considering the intersection between the path of robot i-th
	 * with the paths of other robots, considering the actual Assignment, but also paths related to already driving robot
	 * are considered.
	 * @param robot -> The i-th Robot
	 * @param task -> The j-th Task
	 * @param assignmentMatrix -> The Assignment Matrix related to a solution of the optimization problem
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
	 */
	private double evaluatePathDelay(int robot ,int task,double [][] assignmentMatrix,AbstractTrajectoryEnvelopeCoordinator tec){
		

		PrintStream fileStream1 = null;
		PrintStream fileStream2 = null;
		try {
			fileStream1 = new PrintStream(new FileOutputStream("CriticalSections.txt",true));
			fileStream2 = new PrintStream(new FileOutputStream("PathDelay.txt",true));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		long timeInitial2 = 0;

		
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		double delay = 0;
		//Considering the Actual Assignment 
		if (assignmentMatrix[robot-1][task]>0) {
			// Only for real robots and tasks
			if (task < numTask && robot <= numRobot) {
				//Take the Pose steering relate to i-th robot and j-th task from path set
				PoseSteering[] pss1 = pathsToTargetGoal.get((robot-1)*assignmentMatrix[0].length + task);	
				if(pss1 == null) {
					return delay;
				}
				//Initialize Array of delays for the two robots
				TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
				TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
				//Compute the spatial Envelope for the i-th Robot
				SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,tec.getFootprint(robot));
				//Evaluate other path depending from the Assignment Matrix
				for(int m = 0; m < assignmentMatrix.length; m++) {
					for(int n = 0; n < assignmentMatrix[0].length; n++) {
						if (assignmentMatrix [m][n] > 0 && m+1 != robot && n != task && n < numTask && m < numRobot) {
							//Take the path of this second robot from path set
							PoseSteering[] pss2 = pathsToTargetGoal.get((m)*assignmentMatrix[0].length  + n);
							if (pss2 != null) {//is == null if robotType is different to Task type or path not exists
								//Evaluate the Spatial Envelope of this second Robot
								SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,tec.getFootprint(m+1));
								long timeInitial = Calendar.getInstance().getTimeInMillis();
								//Compute the Critical Section between this 2 robot
								CriticalSection [] css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robot).getArea(),tec.getFootprintPolygon(m+1).getArea()));
								
								
								//Evaluate the time to compute critical Section
								long timeFinal = Calendar.getInstance().getTimeInMillis();
								 long timeRequired = timeFinal- timeInitial;
								 timeRequiretoComputeCriticalSection = timeRequiretoComputeCriticalSection + timeRequired;
								 fileStream1.println(timeRequired+"");
								 
								 timeInitial2 = Calendar.getInstance().getTimeInMillis();
								//Compute the delay due to precedence constraint in Critical Section
								for (int g = 0; g < css.length; g++) {
									Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pss2.hashCode(),pss2,te2TCDelays, css[g]);
									double delayCriticalSection = a1.getFirst();
									if(delayCriticalSection < 0 ) {
										delay += 0;
									}else if(delayCriticalSection == Double.POSITIVE_INFINITY) {
										delay += 10000;
									}else {
										delay += delayCriticalSection;
									}
								}
							
						//Take the paths of driving robots from coordinator
					    pathsDrivingRobot = tec.getDrivingEnvelope();
					  //Evaluate the delay time due to already driving robots
					    for(int k = 0; k < pathsDrivingRobot.size(); k++) {
					    	CriticalSection [] cssDrivingRobot = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, pathsDrivingRobot.get(k),true, Math.min(tec.getFootprintPolygon(robot).getArea(),tec.getFootprintPolygon(m+1).getArea()));
					    	for (int b = 0; b < cssDrivingRobot.length; b++) {
								Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pathsDrivingRobot.get(k).getPath().hashCode(),pathsDrivingRobot.get(k).getPath(),te2TCDelays, cssDrivingRobot[b]);
								delay +=  a1.getFirst();
							}
					    }
					    
					    long timeFinal2 = Calendar.getInstance().getTimeInMillis();
						long timeRequired2 = timeFinal2- timeInitial2;
						timeRequiretoComputePathsDelay = timeRequiretoComputePathsDelay + timeRequired2;
						fileStream2.println(timeRequired2+"");
						}
					}
				}	
			}
				
			} else { //There also virtual robot and task are considered
				//the delay associated to dummy robot and task is considerd 0
				return delay;
			}
		}
		
		//return the delay for the i-th robot and the j-th task due to interference with other robots
		return delay;
		}
	
	
		
	/**
	 * Compute the arrival time to a task for a specified robot
	 * @param pathLength -> The max path for each robot
	 * @return The time to drive the path
	 */
	private double computeArrivalTime(double pathLength,double vel,double acc){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double arrivalTime = pathLength/vel + vel/acc;
		//Return the arrival time 
		return arrivalTime;
	}
	
	
	/**
	 * Compute the arrival for all robots in fleet to all possible tasks
	 * @param pathLength -> The max path for each robot
	 * @return The time to drive the path
	 */
	private double [][] computeArrivalTimeFleet(double[][]PAll,AbstractTrajectoryEnvelopeCoordinator tec){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double [][] arrivalTimeMatrix = new double [numRobotAug][numTaskAug];
		for (int i = 0 ; i < IDsIdleRobots.size(); i++) {
			for (int j = 0 ; j < taskQueue.size(); j++) {
				double vel = tec.getRobot(i+1).getForwardModel().getVel();
				double acc = tec.getRobot(i+1).getForwardModel().getAcc();
				double arrivalTime = computeArrivalTime(PAll[i][j],vel,acc);
				arrivalTimeMatrix[i][j] = arrivalTime;
			}
		}
		
		//Return the arrival time 
		return arrivalTimeMatrix;
	}
	
	/**
	 * Evaluate the tardiness in completion of a task . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param pathLength -> path length
	 * @param task -> the task j-th
	 * @return
	 */
	
	private double[][] computeTardiness(double [][]PAll,AbstractTrajectoryEnvelopeCoordinator tec) {
		double tardiness = 0;
		
		double [][] tardinessMatrix = new double [numRobotAug][numTaskAug];
		for (int i = 0 ; i < IDsIdleRobots.size(); i++) {
			for (int j = 0 ; j < taskQueue.size(); j++) {
				if (taskQueue.get(j).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
					double deadline = taskQueue.get(j).getDeadline();  //Expressed in seconds
					double vel = tec.getRobot(i+1).getForwardModel().getVel();
					double acc = tec.getRobot(i+1).getForwardModel().getVel();
					double completionTime = computeArrivalTime(PAll[i][j],vel,acc) + taskQueue.get(j).getOperationTime();
					tardiness = Math.max(0, (completionTime-deadline));
					
					tardinessMatrix[i][j] = tardiness;
					sumTardiness += tardiness;
				}	
			}
		}
		return tardinessMatrix;
	}

	/**
	 * Evaluate the overall B function, that is the function that consider interference free costs
	 * Costs considered:
	 * 1) Path Length
	 * 2) Tardiness
	 * Each cost is already normalized;
	 * @param PAll
	 * @return
	 */
	private double [][] evaluateBFunction(double [][]PAll,AbstractTrajectoryEnvelopeCoordinator tec){
		double [][] tardinessMatrix = computeTardiness(PAll,tec);
		double [][] BFunction = new double [numRobotAug][numTaskAug];
		costValuesMatrix = new double [numRobotAug][numTaskAug];
		if(linearWeight == 1) {
			double [][] arrivalTimeMatrix = computeArrivalTimeFleet(PAll,tec);
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					BFunction[i][j] = pathLengthWeight*PAll[i][j]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j]/sumTardiness + arrivalTimeWeight*arrivalTimeMatrix[i][j]/sumArrivalTime;
					costValuesMatrix[i][j] = pathLengthWeight*PAll[i][j]+ tardinessWeight*tardinessMatrix[i][j] + arrivalTimeWeight*arrivalTimeMatrix[i][j];
				}
			}
		}
		else {
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					BFunction[i][j] = pathLengthWeight*PAll[i][j]/sumMaxPathsLength + tardinessWeight*tardinessMatrix[i][j]/sumTardiness;
					costValuesMatrix[i][j] = PAll[i][j]+ tardinessMatrix[i][j];
				}
			}
		}
		
		return BFunction;
	}
	
	/**
	 * Evaluates the number of all feasible solutions for an optimization problem, with is defined with {@link #buildOptimizationProblem}
	 * @param numRobot -> Number of Robot of the optimization problem 
	 * @param numTasks -> Number of Tasks of the optimization problem
	 * @return The number of all feasible solutions for the Optimization Problem
	 */
 
	public int numberFeasibleSolution(int numRobot,int numTasks){
		//Create an optimization problem
		MPSolver optimizationProblemCopy = buildOptimizationProblem(numRobot,numTasks);
		//Solve the optimization problem
	    MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();
	    int numberFeasibleSolution = 0;
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = optimizationProblemCopy.solve();
    		//If The solution is feasible increment the number of feasible solution
    		if (resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
    			numberFeasibleSolution = numberFeasibleSolution+1;
    		}
    		double [][] assignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblemCopy);
			//Add the constraint to actual solution -> in order to consider this solution as already found  
    		optimizationProblemCopy = constraintOnPreviousSolution(optimizationProblemCopy,assignmentMatrix);
	    }
		//Return the Total number of feasible solution
	    optimizationProblemCopy.clear();
	    return numberFeasibleSolution;
	}

	/**
	 * Evaluates all possible feasible solutions for an optimization problem,with is defined with {@link #buildOptimizationProblem}. A feasible solution is a solution that verify constraints
	 * @param numRobot -> Number of Robots
	 * @param numTasks -> Number of Tasks
	 * @return A set containing all feasible solutions
	 */
	public double [][][] evaluateFeasibleSolution(int numRobot,int numTasks){
		//Define the optimization problem
		MPSolver optimizationProblemCopy = buildOptimizationProblem(numRobot,numTasks);
		//Evaluate the number of all feasible solution for the optimization problem
	    int feasibleSolutions = numberFeasibleSolution(numRobot,numTasks);
	    //Initialize a set to store all feasible solution
		double [][][] AssignmentMatrixOptimalSolutions = new double [feasibleSolutions][numRobot][numTasks]; 
	    ///////////////////////////////////////
	    for(int k=0; k < feasibleSolutions; k++) {
			//Solve the optimization problem
	    	MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();
			//Transform the Assignment Vector to Matrix
			double [][] AssignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblemCopy);
			//Store the optimal solution
			AssignmentMatrixOptimalSolutions[k]=AssignmentMatrix;
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationProblemCopy = constraintOnPreviousSolution(optimizationProblemCopy,AssignmentMatrix);
			}
		//Return the set of all Feasible solutions
	    return AssignmentMatrixOptimalSolutions;
		}

	/**
	 * Builds the optimization problem. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * @param numRobot -> Number of Robots
	 * @param numTasks -> Number of Tasks.
	 * @return A constrained optimization problem without the objective function
	 */
	private MPSolver buildOptimizationProblem(int numRobotAug,int numTasksAug) {
		//Initialize a linear solver 
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		//START DECISION VARIABLE VARIABLE
		MPVariable [][] decisionVariable = new MPVariable[numRobotAug][numTasksAug]  ;
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTasksAug; j++) {
				 decisionVariable[i][j] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+"]");
			 }
		}
		//END DECISION VARIABLE
		//////////////////////////
		// START CONSTRAINTS
		//Each Robot can be assign only to a Task	    
		 for (int i = 0; i < numRobotAug; i++) {
			 //Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY, 1);
			 for (int j = 0; j < numTasksAug; j++) {
				 //Build the constraint
				 c0.setCoefficient(decisionVariable[i][j], 1); 
			 }
		 }
		//Each task can be performed only by a robot
		 for (int j = 0; j < numTasksAug; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numRobotAug; i++) {
				//Build the constraint
				c0.setCoefficient(decisionVariable[i][j], 1); 		
			 }
		 }
	
		//END CONSTRAINTS
		//In case of having more task than robots, the task with a closest deadline are set with a higher priority
		 if(taskQueue.size() > IDsIdleRobots.size()) {
			 checkOnTaskDeadline();
			//Each task can be performed only by a robot
			 for (int j = 0; j < taskQueue.size(); j++) {
				//Initialize the constraint
				 if(taskQueue.get(j).isPriority()) {
					 MPConstraint c3 = optimizationProblem.makeConstraint(1, 1); 
					 for (int i = 0; i < IDsIdleRobots.size(); i++) {
							 //Build the constraint
							 c3.setCoefficient(decisionVariable[i][j], 1); 
						 }		
				 }
			 }
		 }
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}
	/**
	 *  * Builds the optimization problem complete with Objective Function. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m)
	 * with n = number of robot and m = number of tasks.
	 * Only the B function is considered in this case
	 * @param tec -> An AbstractTrajectoryEnvelopeCoordinator Coordinator
	 * @return A constrained optimization problem with the objective function
	 */
	public MPSolver buildOptimizationProblemWithB(AbstractTrajectoryEnvelopeCoordinator tec) {
		
		
		//Take the number of tasks
		numTask = taskQueue.size();
		//Get free robots and their IDs
		numRobot = tec.getIdleRobots().size();
		IDsIdleRobots = tec.getIdleRobots();
		//Evaluate dummy robot and dummy task
		dummyRobotorTask(numRobot,numTask,tec);
		//Build the solver and an objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem); 
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION
		
	    MPObjective objective = optimizationProblem.objective();
    	 for (int i = 0; i < numRobotAug; i++) {
    		 boolean typesAreEqual = false;
			 for (int j = 0; j < numTaskAug; j++) {
				 if (j < numTask && i < numRobot ) {
					 typesAreEqual = taskQueue.get(j).isCompatible(tec.getRobot(i+1));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
					}
				 if (typesAreEqual) {
					//Set the coefficient of the objective function with the normalized path length
					double pathLength  = evaluatePathLength(i+1,j,tec);
					if ( pathLength != MaxPathLength) {
						objective.setCoefficient(decisionVariable[i][j], pathLength); 
					}else {//the path to reach the task not exists
						//the decision variable is set to 0 -> this allocation is not valid
						MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
						 c3.setCoefficient(decisionVariable[i][j],1);  
					}
				 }else { //robotType != taskType
					//the decision variable is set to 0 -> this allocation is not valid
					 MPConstraint c2 = optimizationProblem.makeConstraint(0,0);
					 c2.setCoefficient(decisionVariable[i][j],1); 
				 } 
			 }
		 }
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}
	
	/**
	 * Builds the optimization problem complete with Objective Function. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m)
	 * with n = number of robot and m = number of tasks.
	 * Only the B function is considered in this case, and each cost is normalized with the max path length considering
	 * all missions.
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return A constrained optimization problem with the objective function and each cost is normalized
	 */
	public MPSolver buildOptimizationProblemWithBNormalized(AbstractTrajectoryEnvelopeCoordinator tec) {
		//Take the number of tasks
		numTask = taskQueue.size();
		//Get free robots and their IDs
		numRobot = tec.getIdleRobots().size();
		IDsIdleRobots = tec.getIdleRobots();
		//Evaluate dummy robot and dummy task
		dummyRobotorTask(numRobot,numTask,tec);
		double[][] PAll = evaluatePAll(tec);
		double[][] BFunction = evaluateBFunction(PAll,tec);
		//Build the solver and an objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem); 
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
    	
    	
    	 for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
			 double pathLength  =  PAll[i][j];
			 double costBFunction  =  BFunction[i][j];
			 if ( pathLength != MaxPathLength) {
				 //Set the coefficient of the objective function with the normalized path length
				 //objective.setCoefficient(decisionVariable[i][j], pathLength); 
				 objective.setCoefficient(decisionVariable[i][j],costBFunction); 	 
			 }else { // if the path does not exists or the robot type is different from the task type 
				//the path to reach the task not exists
				//the decision variable is set to 0 -> this allocation is not valid
				MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
				c3.setCoefficient(decisionVariable[i][j],1); 
			 }
		 
			 }			 
		 }
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}
	
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks. The solver first finds the optimal solution considering only B function and then
	 * for this each solution (that is an assignment) evaluates the cost of F function. Then a new optimal solution considering only B is 
	 * computed and it is consider only if the cost of this new assignment considering only B is less than the min cost of previous assignments
	 * considering both F and B function
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblemWithB}
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1-alpha)*F
	 * @return An Optimal Assignment that minimize the objective function
	 */
	
	public double [][] solveOptimizationProblem(MPSolver optimizationProblem,AbstractTrajectoryEnvelopeCoordinator tec,double alpha){
		
		PrintStream fileStream = null;
		PrintStream fileStream1 = null;
		try {
			fileStream = new PrintStream(new File("RequiredTime.txt"));
			fileStream1 = new PrintStream(new File("CriticalSections.txt"));
			PrintStream fileStream2 = new PrintStream(new File("PathDelay.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Initialize the optimal assignment and the cost associated to it
		double [][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug];
		double objectiveOptimalValue = 100000000;
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		int cont=0;
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate an optimal assignment that minimize only the B function
			resultStatus = optimizationProblem.solve();
			//Evaluate the Assignment Matrix
			double [][] AssignmentMatrix = saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblem);
			//Initialize cost of objective value
			double objectiveFunctionValue = 0;
			double costValue = 0; // -> is the cost of B function non normalized
			double costofAssignment = 0;
			double costofAssignmentForConstraint = 0;
			double costF = 0;
			//Evaluate the cost of F Function for this Assignment
			timeRequiretoComputeCriticalSection = 0;
			timeRequiretoComputePathsDelay = 0;
			
			//Take time to understand how much time require this function
			for (int i = 0; i < numRobotAug; i++) {
				for(int j = 0;j < numTaskAug; j++) {
					if ( AssignmentMatrix[i][j] > 0) {
						if (alpha != 1) {
							//Evaluate cost of F function only if alpha is not equal to 1
							costF = evaluatePathDelay(i+1,j,AssignmentMatrix,tec)/sumArrivalTime;
							double costB = optimizationProblem.objective().getCoefficient(optimizationProblem.variables()[i*numTaskAug+j]);
							costofAssignment = alpha*costB + (1-alpha)*costF + costofAssignment ;
							costofAssignmentForConstraint = costB + costF + costofAssignmentForConstraint;
							
						}
						else {
							//In order to solve the case with more optimal solution with the same cost, the pow of each cost is considered
							double costB = optimizationProblem.objective().getCoefficient(optimizationProblem.variables()[i*numTaskAug+j]);
							costofAssignment = Math.pow(alpha*costB, 2) + costofAssignment ;
							costofAssignmentForConstraint = Math.pow(costB,2) + costofAssignmentForConstraint;
	
						}
					}				
				}		
			}
			System.out.println("cost>>"+ costofAssignmentForConstraint);
			
			fileStream.println(timeRequiretoEvaluatePaths+"");
			fileStream.println(timeRequiretofillInPall+"");
			fileStream.println(timeRequiretoComputeCriticalSection+"");
			fileStream.println(timeRequiretoComputePathsDelay+"");
			//Compare actual solution and optimal solution finds so far
			if (costofAssignment < objectiveOptimalValue && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				objectiveOptimalValue = costofAssignment;
				optimalAssignmentMatrix = AssignmentMatrix;
				
				
				
			}
			//Add the constraint on cost for next solution
			optimizationProblem = constraintOnCostSolution(optimizationProblem,costofAssignmentForConstraint);
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationProblem = constraintOnPreviousSolution(optimizationProblem,AssignmentMatrix);
	
		}
		
		//Return the Optimal Assignment Matrix 
		return  optimalAssignmentMatrix;    
	}
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks
	 * Assignments are computed at each step in this case. In this case the solver consider all the feasible solutions for the problem.
	 * Exact Algorithm.  
	 * @param tec -> TrajectoryEnvelopeCoordinatorSimulation
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1 - alpha)*F 
	 * @return The Optimal Assignment that minimize the objective function
	 */

	public double [][] solveOptimizationProblemExactAlgorithm(AbstractTrajectoryEnvelopeCoordinator tec,double alpha){
		
		PrintStream fileStream = null;
		try {
			fileStream = new PrintStream(new File("RequiredTime.txt"));
			PrintStream fileStream1 = new PrintStream(new File("CriticalSections.txt"));
			PrintStream fileStream2 = new PrintStream(new File("PathDelay.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		numTask = taskQueue.size();
		//Get free robots
		numRobot = tec.getIdleRobots().size();
		IDsIdleRobots = tec.getIdleRobots();
		//Evaluate dummy robot and dummy task
		dummyRobotorTask(numRobot,numTask,tec);
		//Consider possibility to have dummy Robot or Tasks
		double [][] PAll = evaluatePAll(tec);
		double [][] BFunction = evaluateBFunction(PAll,tec);
		//Build the optimization Problem without the objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		//Initialize the optimal assignment and the cost associated to it
		double [][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug];
		double objectiveOptimalValue = 100000000;
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate a feasible assignment
			resultStatus = optimizationProblem.solve();
			//Evaluate the Assignment Matrix
			double [][] AssignmentMatrix = saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblem);
			//Initialize cost of objective value
			double objectiveFunctionValue = 0;
			double costBFunction = 0;
			double costFFunction = 0;
			//Evaluate the cost for this Assignment
			for (int i = 0; i < numRobotAug ; i++) {
				for(int j=0;j < numTaskAug ; j++) {
					if (AssignmentMatrix[i][j]>0) {
							//costBFunction = costBFunction + PAll[i][j]/sumMaxPathsLength;
							costBFunction = costBFunction + BFunction[i][j];
							if (alpha != 1) {
								costFFunction = costFFunction + evaluatePathDelay(i+1,j,AssignmentMatrix,tec)/sumArrivalTime;
							}	
					}
				}
			}
			
			
			fileStream.println(timeRequiretoEvaluatePaths+"");
			fileStream.println(timeRequiretofillInPall+"");
			fileStream.println(timeRequiretoComputeCriticalSection+"");
			fileStream.println(timeRequiretoComputePathsDelay+"");
			
			objectiveFunctionValue = alpha * costBFunction + (1-alpha)*costFFunction;
			//Compare actual solution and optimal solution finds so far
			if (objectiveFunctionValue < objectiveOptimalValue && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				objectiveOptimalValue = objectiveFunctionValue;
				optimalAssignmentMatrix = AssignmentMatrix;
			}
			//Add the constraint to actual solution in order to consider this solution as already found  
			optimizationProblem = constraintOnPreviousSolution(optimizationProblem,AssignmentMatrix);
		}
		//Return the Optimal Assignment Matrix
		return  optimalAssignmentMatrix;    
	}
	
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks
	 * The problem is resolved considering a Greedy algorithm, so each robot is assigned to the best task for him and then the robot and Task are
	 * removed from their set
	 * @param tec -> TrajectoryEnvelopeCoordinatorSimulation
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1 - alpha)*F 
	 * @return The Optimal Assignment that minimize the objective function
	 */

	public double [][] solveOptimizationProblemGreedyAlgorithm(AbstractTrajectoryEnvelopeCoordinator tec,double alpha){
		
		PrintStream fileStream = null;
		try {
			fileStream = new PrintStream(new File("RequiredTime.txt"));
			PrintStream fileStream1 = new PrintStream(new File("CriticalSections.txt"));
			PrintStream fileStream2 = new PrintStream(new File("PathDelay.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		numTask = taskQueue.size();
		//Get free robots
		numRobot = tec.getIdleRobots().size();
		IDsIdleRobots = tec.getIdleRobots();
		//Evaluate dummy robot and dummy task
		dummyRobotorTask(numRobot,numTask,tec);
		double [][] PAll = evaluatePAll(tec);
		double [][] BFunction = evaluateBFunction(PAll,tec);
		double [][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug];
		//Initialize optimal indexes 
		int iOtt = 0;
		int jOtt = 0;
		//Initialize a boolean vector related to task set in order to consider already allocate task
		boolean [] TasksMissionsAllocates = new boolean [numTaskAug];
		for (int i = 0; i < numRobotAug ; i++) {
			double costBFunction = 0;
			double OptimalValueBFunction = 100000000;
			boolean typesAreEqual = false;
			for(int j=0;j < numTaskAug ; j++) {
			 if (j < numTask && i < numRobot ) {
				 typesAreEqual = taskQueue.get(j).isCompatible(tec.getRobot(i+1));
			 }
			 else {
				 //Considering a dummy robot or  a dummy task -> they don't have type
				 typesAreEqual = true;
				}
			 if (typesAreEqual) {
					 //costBFunction = PAll[i][j]/sumMaxPathsLength;
				 costBFunction = BFunction[i][j];
					 if (costBFunction < OptimalValueBFunction  && !TasksMissionsAllocates[j] ) {
							OptimalValueBFunction = costBFunction;			
							iOtt = i;
							jOtt= j;
						}
				 }
			}
			
			fileStream.println(timeRequiretoEvaluatePaths+"");
			fileStream.println(timeRequiretofillInPall+"");
			fileStream.println(timeRequiretoComputeCriticalSection+"");
			fileStream.println(timeRequiretoComputePathsDelay+"");
			
			optimalAssignmentMatrix[iOtt][jOtt] = 1;
			//the task is already assigned
			TasksMissionsAllocates[jOtt] = true;
		}
		//Return the Optimal Assignment Matrix
		return  optimalAssignmentMatrix;    
	}

	/**
	 * Perform the task Assignment defining the mission for each robot
	 * @param AssignmentMatrix -> An Assignment Matrix of the optimization problem
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return An updated Trajectory Envelope Coordinator Simulation in which the mission for each
	 * robot is defined
	 */
	public boolean TaskAllocation(double [][] AssignmentMatrix,AbstractTrajectoryEnvelopeCoordinator tec){
		System.out.println("Number of Robot : " + numRobot);
		System.out.println("Number of Task : " + numTask);
		System.out.println("Number of dummy Robot : " + dummyRobot);
		System.out.println("Number of dummy Task : " + dummyTask);
		System.out.println("Total Number of Robot : " + numRobotAug);
		System.out.println("Total Number of Task : " + numTaskAug);
		for (int i = 0; i < AssignmentMatrix.length; i++) {
			 for (int j = 0; j < AssignmentMatrix[0].length; j++) {
				 if (AssignmentMatrix[i][j] > 0) {
					 if (i < IDsIdleRobots.size()) { //Considering only real Robot
						 PoseSteering[] pss = pathsToTargetGoal.get(i*AssignmentMatrix[0].length + j);	
						 //For Dispatch mission
						 if (j < taskQueue.size() && pss != null) {
							 taskQueue.get(j).assignRobot(i+1);
							 taskQueue.get(j).setPaths(pss);
							 Mission[] robotMissions = taskQueue.get(j).getMissions();
							 viz.displayTask(taskQueue.get(j).getStartPose(), taskQueue.get(j).getGoalPose(), (j+1), "red");
							 
							 //tec.addMissions(new Mission(IDsIdleRobots[i],pss));
							 System.out.println("Task # "+ (j+1) + " is Assigned");
							 
							 tec.addMissions(robotMissions);
						 }else {
							 System.out.println("Virtual Task # "+ (j+1) + " is Assigned to a real robot");
						 }
					 }else{
						 System.out.println("Task # "+ (j+1) + " is not Assigned to a real robot");
						 
					 }
				 } 
			 }
		 }
		//Remove Assigned Tasks from the set	
		int i = 0;
		int cont = 0;
		while (i < Math.min(IDsIdleRobots.size(), taskQueue.size())) {
			if (taskQueue.size() == 0 || taskQueue.size() <= i) {
				break;
			}
			if (taskQueue.get(i).isTaskAssigned()){
				taskQueue.remove(i);
				System.out.println("Task # "+ (cont+1) + " is removed ");
			}else {
				i = i+1;
			}
			cont +=1;	
			
		}
		 System.out.println("Remaining task: "+ taskQueue.size());
		//Remove all path from the path set
		pathsToTargetGoal.removeAll(pathsToTargetGoal);
		return true;
	}//End Task Assignment Function
	

	Callback cb = new Callback() {
		private long lastUpdate = Calendar.getInstance().getTimeInMillis();
		@Override
		public void performOperation() {
			long timeNow = Calendar.getInstance().getTimeInMillis();
			if (timeNow-lastUpdate > 1000) {
				lastUpdate = timeNow;
			}
		}
	};
	/**
	 * Start the Task Allocation Algorithm 
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param alpha -> the weight of B and F function in objective function. It is considered as
	 * B*alpha + (1-alpha)*F
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 */
	public void startTaskAssignment(AbstractTrajectoryEnvelopeCoordinator tec) {
		//Create meta solver and solver
		coordinator = tec;
		numRobot = coordinator.getIdleRobots().size();

		//Start a thread that checks and enforces dependencies at every clock tick
		this.setupInferenceCallback();

	}
	
	
	protected void setupInferenceCallback() {
		
		Thread TaskAssignmentThread = new Thread("Task Assignment") {
			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			
			public void run() {
				while (true) {
					System.out.println("Thread Running");
					if (!taskQueue.isEmpty() && coordinator.getIdleRobots().size() != 0 ) {
						MPSolver solverOnline = buildOptimizationProblemWithBNormalized(coordinator);
						double [][] assignmentMatrix = solveOptimizationProblem(solverOnline,coordinator,linearWeight);
						for (int i = 0; i < assignmentMatrix.length; i++) {
							for (int j = 0; j < assignmentMatrix[0].length; j++) {
									System.out.println("x"+"["+(i+1)+","+(j+1)+"]"+" is "+ assignmentMatrix[i][j]);
									if (assignmentMatrix[i][j] == 1) {
										System.out.println("Robot " +(i+1) +" is assigned to Task "+ (j+1));
									}
							} 
						}
						TaskAllocation(assignmentMatrix,coordinator);
						System.out.print("Task to be completed "+ taskQueue.size());
						solverOnline.clear();
					}

					//Sleep a little...
					if (CONTROL_PERIOD_Task > 0) {
						try { 
							System.out.println("Thread Sleeping");
							Thread.sleep(CONTROL_PERIOD_Task); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
					EFFECTIVE_CONTROL_PERIOD_task = (int)(threadCurrentUpdate-threadLastUpdate);
					threadLastUpdate = threadCurrentUpdate;
					
					if (cb != null) cb.performOperation();

				}
			}
		};
		TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
		TaskAssignmentThread.start();
	}
	
	/**
	 * Start the Task Allocation Algorithm with Greedy Algorithm
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param alpha -> the weight of B and F function in objective function. It is considered as
	 * B*alpha + (1-alpha)*F
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 */
	public void startTaskAssignmentGreedyAlgorithm(AbstractTrajectoryEnvelopeCoordinator tec) {
		//Create meta solver and solver
		coordinator = tec;
		numRobot = coordinator.getIdleRobots().size();
		//Start a thread that checks and enforces dependencies at every clock tick
		this.setupInferenceCallbackGreedy();

	}
	
	
	protected void setupInferenceCallbackGreedy() {
		
		Thread TaskAssignmentThread = new Thread("Task Assignment") {
			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			
			public void run() {
				while (true) {
					System.out.println("Thread Running");
					if (!taskQueue.isEmpty() && coordinator.getIdleRobots().size() != 0 ) {
						double [][] assignmentMatrix = solveOptimizationProblemGreedyAlgorithm(coordinator,linearWeight);
						for (int i = 0; i < assignmentMatrix.length; i++) {
							for (int j = 0; j < assignmentMatrix[0].length; j++) {
									System.out.println("x"+"["+(i+1)+","+(j+1)+"]"+" is "+ assignmentMatrix[i][j]);
									if (assignmentMatrix[i][j] == 1) {
										System.out.println("Robot " +(i+1) +" is assigned to Task "+ (j+1));
									}
							} 
						}
						TaskAllocation(assignmentMatrix,coordinator);
						System.out.print("Task to be completed "+ taskQueue.size());
					}
					//Sleep a little...
					if (CONTROL_PERIOD_Task > 0) {
						try { 
							System.out.println("Thread Sleeping");
							Thread.sleep(CONTROL_PERIOD_Task); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
					EFFECTIVE_CONTROL_PERIOD_task = (int)(threadCurrentUpdate-threadLastUpdate);
					threadLastUpdate = threadCurrentUpdate;
					
					if (cb != null) cb.performOperation();

				}
			}
		};
		TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
		TaskAssignmentThread.start();
	}
	
	
	
	/**
	 * Start the Task Allocation Algorithm with Exact Algorithm
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param alpha -> the weight of B and F function in objective function. It is considered as
	 * B*alpha + (1-alpha)*F
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 */
	public void startTaskAssignmentExactAlgorithm(double alpha,AbstractTrajectoryEnvelopeCoordinator tec) {
		//Create meta solver and solver
		coordinator = tec;
		numRobot = coordinator.getIdleRobots().size();
		linearWeight = alpha;
		//Start a thread that checks and enforces dependencies at every clock tick
		this.setupInferenceCallbackExact();

	}
	
	
	protected void setupInferenceCallbackExact() {
		
		Thread TaskAssignmentThread = new Thread("Task Assignment") {
			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			
			public void run() {
				while (true) {
					System.out.println("Thread Running");
					if (!taskQueue.isEmpty() && coordinator.getIdleRobots().size() != 0 ) {
						double [][] assignmentMatrix = solveOptimizationProblemExactAlgorithm(coordinator,linearWeight);
						for (int i = 0; i < assignmentMatrix.length; i++) {
							for (int j = 0; j < assignmentMatrix[0].length; j++) {
									System.out.println("x"+"["+(i+1)+","+(j+1)+"]"+" is "+ assignmentMatrix[i][j]);
									if (assignmentMatrix[i][j] == 1) {
										System.out.println("Robot " +(i+1) +" is assigned to Task "+ (j+1));
									}
							} 
						}
						TaskAllocation(assignmentMatrix,coordinator);
						System.out.print("Task to be completed "+ taskQueue.size());
					}

					//Sleep a little...
					if (CONTROL_PERIOD_Task > 0) {
						try { 
							System.out.println("Thread Sleeping");
							Thread.sleep(CONTROL_PERIOD_Task); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
					EFFECTIVE_CONTROL_PERIOD_task = (int)(threadCurrentUpdate-threadLastUpdate);
					threadLastUpdate = threadCurrentUpdate;
					
					if (cb != null) cb.performOperation();

				}
			}
		};
		TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
		TaskAssignmentThread.start();
	}
	}//End Class

