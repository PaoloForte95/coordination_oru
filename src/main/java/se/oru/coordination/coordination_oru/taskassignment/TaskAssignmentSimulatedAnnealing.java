package se.oru.coordination.coordination_oru.taskassignment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;


import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;


import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.IndexedDelay;
import se.oru.coordination.coordination_oru.Mission;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.CumulatedIndexedDelaysList;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import com.google.ortools.linearsolver.*;



public class TaskAssignmentSimulatedAnnealing {
	
		//Optimization Problem Parameters
		protected int numRobot;
		protected int numTask;
		protected int dummyRobot;
		protected int dummyTask;
		protected int numRobotAug;
		protected int numTaskAug;
		protected int maxNumPaths = 1;
		protected double linearWeight = 1;
		protected double [][][] costValuesMatrix;
		protected double timeOut = Double.POSITIVE_INFINITY;
		protected ArrayList <Task> taskQueue = new ArrayList <Task>();
		
		protected String scenario;
		protected double [][][] ScenarioAllocation;
		//Parameters of weights in Optimization Problem
		protected double pathLengthWeight = 1;
		protected double arrivalTimeWeight = 0;
		protected double tardinessWeight = 0;
		//Number of Idle Robots
		protected ArrayList <Integer> IDsIdleRobots = new ArrayList <Integer>();
		protected ArrayList <Integer> IDsRealTasks = new ArrayList <Integer>();
		protected ArrayList <Integer> IDsAllRobots = new ArrayList <Integer>();
		protected ArrayList <Integer> IDsAllTasks = new ArrayList <Integer>();
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
		
		protected int virtualRobotIDs = Integer.MAX_VALUE;
		protected int virtualTaskIDs = Integer.MAX_VALUE;
		
		//Motion planner and Coordinator Parameters
		protected AbstractTrajectoryEnvelopeCoordinator coordinator;
		
		//Time required by function parameters
		protected long timeRequiretoEvaluatePaths;
		protected long timeRequiretofillInPall;
		protected long timeRequiretoComputeCriticalSection;
		protected long timeRequiretoComputePathsDelay;
		protected long initialTime;
		
		protected HashMap<Integer, PoseSteering[]> pathsToTargetGoal =  new HashMap<Integer, PoseSteering[]>();
		protected ArrayList <SpatialEnvelope> pathsDrivingRobot = new ArrayList <SpatialEnvelope>();
		protected HashMap<Integer,CriticalSection [][][][]> criticalSections =  new HashMap<Integer, CriticalSection [][][][]>();
	
		//FleetMaster Interface Parameters
		
		protected AbstractFleetMasterInterface fleetMasterInterface = null;
		protected boolean propagateDelays = false;
		protected static Logger metaCSPLogger = MetaCSPLogging.getLogger(TrajectoryEnvelopeCoordinator.class);
		
		
		//Task Allocation Thread Parameters 
		protected int CONTROL_PERIOD_Task = 15000;
		public static int EFFECTIVE_CONTROL_PERIOD_task = 0;
		protected FleetVisualization viz = null;
		
		/**
		 * Set the number of paths to reach a goal.
		 * @param maxNumPaths -> number of path to reach a goal
		 */
		public void setmaxNumPaths(int maxNumPaths) {
			
			this.maxNumPaths = maxNumPaths;
		}
		
		public void writeMatrix(String filename, double[][][] optimalAssignmentMatrix) {
		    try {
		        BufferedWriter bw = new BufferedWriter(new FileWriter((filename),true));
		        bw.write("{{");
		        for (int i = 0; i < optimalAssignmentMatrix.length; i++) {
		        	for (int j = 0; j < optimalAssignmentMatrix[i].length; j++) {
		        		bw.write("{");
		        		for (int s = 0; s < maxNumPaths; s++) {
		        			bw.write(optimalAssignmentMatrix[i][j][s]+"");
		        		}
		        		bw.write("}");
		        		bw.write(",");
		        		
		        	}
		        	bw.write("}");
		        	bw.write(",");
		            bw.newLine();
		            bw.write("{");
		        }
		        bw.write("-------------");
		        bw.newLine();
		        bw.flush();
		    } catch (IOException e) {}
		}
		
		/**
		 * Load a  Scenario 
		 * @param scenario ->  Scenario to load
		 */
		
		public void LoadScenario(String scenario) {
			this.scenario = scenario;
		}
		
		/**
		 * Load an Optimal Task Allocation
		 * @param scenario ->  Scenario to load
		 */
		
		public void LoadScenarioAllocation(double [][][] Allocation) {
			this.ScenarioAllocation = Allocation;
		}
		
		/**
		 * Set the timeOut for the optimazion Problem in minutes. The algorithm will search a solution until this time.
		 * Use number from 0.1 to 0.6 for seconds
		 * @param timeOus -> timeout value in minutes
		 * 
		 */
		public void setTimeOutinMin(double timeOut) {
			if(timeOut < 0.1) {
				throw new Error("Timeout cannot be negative!");
			}
			if(timeOut < 0.6) {
				this.timeOut = timeOut*1000;
			}
			else {
				this.timeOut = timeOut*60*1000;
			}
			
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
	 * Create the IDs for virtual robot only; Them are created starting from IDs of last 
	 * @param viz -> Visualization to use 
	 */
	
	private void getAllRobotIDs() {
		int virtualRobotID = this.virtualRobotIDs;
		for(int i= 0; i < numRobotAug; i++) {
			if(i < IDsIdleRobots.size()) {
				IDsAllRobots.add(IDsIdleRobots.get(i));	
			}else {
				IDsAllRobots.add(virtualRobotID);
				virtualRobotID = virtualRobotID-1;
				this.virtualRobotIDs -=1;
			}	
		}
	}
	
	/**
	 * Get IDs of all tasks given in Optimization problem
	 * @return
	 */
	public ArrayList <Integer> getTaskIDs() {
		ArrayList <Integer> taskGivenIDs = new ArrayList <Integer>();
		for(int j=0; j < taskQueue.size();j++) {
			taskGivenIDs.add(taskQueue.get(j).getID());
		}
		return taskGivenIDs;
	}
	
	private void  getRealTaskIDs() {
		for(int j=0; j < taskQueue.size();j++) {
			IDsRealTasks.add(taskQueue.get(j).getID());
		}
	}
	
	
	private void getAllTaskIDs() {
		getRealTaskIDs();
		int virtaulTaskID = this.virtualTaskIDs;
		for(int i= 0; i < numTaskAug; i++) {
			if(i < IDsRealTasks.size()) {
				IDsAllTasks.add(IDsRealTasks.get(i));
			}else {
				IDsAllTasks.add(virtaulTaskID);
				virtaulTaskID = virtaulTaskID-1;
				this.virtualTaskIDs -=1;
			}	
		}
		
	}
	
	/**
	 * Check if a goal can be reached by at least one robot of the Fleet 
	 * @param PAll -> Initial PAll
	 * @return PAll incremented a task cannot be reach by any robot
	 */
	
	private  double [][][] checkTargetGoals (double [][][] PAll){
		for (int j= 0; j< PAll[0].length ; j++) {
			boolean targetEndCanBeReach = false;
			for (int i = 0; i < PAll.length; i++) {
				for (int s = 0; s < maxNumPaths; s++) {
					if(PAll[i][j][s] != MaxPathLength) {
						targetEndCanBeReach = true;
					}
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
		
		double [][][] PAllAug = new double [numRobotAug][numTaskAug][maxNumPaths];
		int robotID = 0;
		int taskID = 0;
		for(int i = 0;i < numRobotAug; i++) {
			for(int j = 0; j<  numTaskAug;j++) {
				for (int s = 0; s < maxNumPaths; s++) {
					
					if(i < PAll.length && j< PAll[0].length) {
						PAllAug[i][j][s] = PAll[i][j][s];
					}else {
						if(i < IDsAllRobots.size()) {
							robotID = IDsAllRobots.get(i);

							
						}else {
							robotID = this.virtualRobotIDs -1 ;
							this.virtualRobotIDs -= 1;
							IDsAllRobots.add(robotID);
						}
						if (j < IDsAllTasks.size()) {
							taskID = IDsAllTasks.get(j);
						}else {
							taskID = virtualTaskIDs - 1;
							this.virtualTaskIDs -= 1;
							IDsAllTasks.add(taskID);
						}
						PAllAug[i][j][s] = 1;
						pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s, null);
				
					}
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
		for(int i=0;i < IDsIdleRobots.size(); i++) {
			int index = taskQueue.indexOf(taskArray.get(i));
			if(taskQueue.get(index).getDeadline() != -1) {
				taskQueue.get(index).setPriority(true);
			}
			
		}
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
	 * Transform a 1D array of MPVariable into a 3D MATRIX  
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblem}, {@link #buildOptimizationProblemWithB}  or {@link #buildOptimizationProblemWithBNormalized}
	 * @return 3D Matrix of Decision Variable of the input problem
	 */
	private MPVariable [][][] tranformArray(MPSolver optimizationProblem) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][][] decisionVariable = new MPVariable [numRobotAug][numTaskAug][maxNumPaths];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for (int s = 0; s < maxNumPaths; s++) {
					 decisionVariable[i][j][s] = array1D[i*numTaskAug*maxNumPaths+j*maxNumPaths+s];
				 }
				 
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
	private MPSolver constraintOnPreviousSolution(MPSolver optimizationProblem, double [][][] assignmentMatrix) {
		//Take decision Variable from Optimization Problem
		MPVariable [][][] DecisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			for(int s = 0;s < maxNumPaths; s++) {
    					if (assignmentMatrix[i][j][s] >0) {
    						c2.setCoefficient(DecisionVariable[i][j][s],1);
    					}else {
    						c2.setCoefficient(DecisionVariable[i][j][s],0);
	    			}
    			}
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
	
	private double [][][] saveAssignmentMatrix(int numRobot,int numTasks,MPSolver optimizationProblem){
		//Take the decision variable from the optimization problem
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem);
		double [][][] assignmentMatrix = new double [numRobot][numTasks][maxNumPaths];	
		//Store decision variable values in a Matrix
		for (int i = 0; i < numRobot; i++) {
			for (int j = 0; j < numTasks; j++) {
				for(int s = 0;s < maxNumPaths; s++) {
					assignmentMatrix[i][j][s] = decisionVariable[i][j][s].solutionValue();
				}
			}
		}
		return assignmentMatrix;	
	}
	
	
	/**
	 * Evaluate the cost associated to the path length for the a couple of robot and task.
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param robot -> The  Robot ID
	 * @param task -> The Task ID
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private double evaluatePathLength(int robotID , int taskID, int path, AbstractTrajectoryEnvelopeCoordinator tec){
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the path length to infinity
		double pathLength = MaxPathLength;
		//take index positon of robotID in Robot set
		int robotindex = IDsIdleRobots.indexOf(robotID);
		// Only for real robots and tasks
		if (IDsIdleRobots.contains(robotID) && IDsRealTasks.contains(taskID)) {
			//Take the state for the i-th Robot
			
			RobotReport rr = tec.getRobotReport(IDsIdleRobots.get(robotindex));
			if (rr == null) {
				metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
				throw new Error("RobotReport not found for Robot" + robotID + ".");
			}
			//Evaluate the path from the Robot Starting Pose to Task End Pose
			int taskIndex = IDsRealTasks.indexOf(taskID);
			AbstractMotionPlanner rsp =  tec.getMotionPlanner(robotID).getCopy(false);
			rsp.setStart(rr.getPose());
			rsp.setGoals(taskQueue.get(taskIndex).getStartPose(),taskQueue.get(taskIndex).getGoalPose());
			rsp.setFootprint(tec.getFootprint(robotID));
			
			if (!rsp.plan()) {
				System.out.println("Robot" + robotID +" cannot reach the Target End of Task " + taskID);
				//the path to reach target end not exits
				pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, null);		
				//Infinity cost is returned 
				return pathLength;
			}
			
			//If the path exists
			//Take the Pose Steering representing the path
			PoseSteering[] pss = rsp.getPath();
			//Add the path to the FleetMaster Interface -> this is necessary for F function
			addPath(robotID, pss.hashCode(), pss, null, tec.getFootprint(robotID));
			//Save the path to Task in the path set
			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
			//Take the Path Length
			Mission m1 = new Mission(robotID,pss);
			Missions.enqueueMission(m1);
			pathLength = Missions.getPathLength(pss);
		} else { //There also virtual robot and task are considered 
			//There are considered real robot and dummy task
			if (numRobot >= numTask && IDsIdleRobots.contains(robotID)){ //dummy task -> The Robot receive the task to stay in starting position
				//The second condition is used in the special case in which we have that one robot cannot be 
				//assigned to any tasks due to its type, so we must add a dummy robot and a dummy task, but we 
				//Create the task to stay in robot starting position
				PoseSteering[] dummyTask = new PoseSteering[1];
				//Take the state for the i-th Robot
				RobotReport rr = tec.getRobotReport(robotID);
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
					throw new Error("RobotReport not found for Robot" + robotID + ".");
				}
				//take the starting position of the robot
				dummyTask[0] = new PoseSteering(rr.getPose(),0);
				//Add the path to the FleetMaster Interface -> so it can be considered as an obstacle from 
				//the motion planner
				addPath(robotID, dummyTask.hashCode(), dummyTask, null, tec.getFootprint(robotID));
				//Save the path to Dummy Task 
				pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, dummyTask);		
				//Consider a minimal pathLength
				pathLength = 1;
				Mission m1 = new Mission(robotID,dummyTask);
				Missions.enqueueMission(m1);
				return pathLength;
			}
			else { //There are considered dummy robot and real task
				//dummy robot -> Consider a only virtual Robot 
				PoseSteering[] dummyRobot = new PoseSteering[1];
				dummyRobot[0] = new PoseSteering(taskQueue.get(0).getGoalPose(),0);
				pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, dummyRobot);
				pathLength = 1;
				Mission m1 = new Mission(robotID,dummyRobot);
				Missions.enqueueMission(m1);
				return pathLength;
			}	
		}	
		
		return pathLength;
	}
	
	
	
	/**
	 * Evaluate the PAll matrix with a precomputed Scenario, that is a matrix that contains all path for each possible combination of robot
	 * and task
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param rsp -> The motion planner that will be called for planning for any
	 * robot. 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return The PAll matrix
	 */
	private double [][][] evaluatePAllWithScenario(AbstractTrajectoryEnvelopeCoordinator tec){
		
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the sum of max paths lengths and time to do it for each robot
		//This cost are used then for normalizing cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;
		Missions.loadScenario(scenario);
		
		double [][][] PAll = new double[numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : IDsAllRobots) {
			int robotindex = IDsAllRobots.indexOf(robotID);
			double maxPathLength = 1;
			for (int taskID : IDsAllTasks ) {
				int taskIndex = IDsAllTasks.indexOf(taskID);
				double pathLength = MaxPathLength;
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (taskIndex < numTask && robotindex < numRobot ) {
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(tec.getRobot(robotID));
				 
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < maxNumPaths; path++) {
					 if(typesAreEqual) { 
						 	if(IDsIdleRobots.contains(robotID)) {
						 		
						 	}else {
						 		
						 	}
						 	
						 	int cont= 0;
						 	
						 	
						 	if(IDsIdleRobots.contains(robotID)&& taskIndex < taskQueue.size()) {
						 		
						 		while(cont < Missions.getMissions(robotID).size() ) {
						 			 if( ArrayUtils.isEquals(Missions.getMission(robotID, cont).getToPose().toString(),taskQueue.get(taskIndex).getGoalPose().toString())) {
						 				PoseSteering[] pss = Missions.getMission(robotID, cont).getPath();
							 			addPath(robotID, pss.hashCode(), pss, null, tec.getFootprint(robotID));
							 			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
										pathLength = Missions.getPathLength(pss);
							 		 }
							 		 cont +=1;
							 		
							 	}
						 	}else {
						 		if (numRobot >= numTask && IDsIdleRobots.contains(robotID)){
						 			PoseSteering[] pss = new PoseSteering[] {new PoseSteering(tec.getRobotReport(robotID).getPose(),0)};
						 			pathLength =1 ;
						 			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
						 			
						 		}else {
						 			PoseSteering[] pss = new PoseSteering[] {new PoseSteering(taskQueue.get(0).getGoalPose(),0)};
						 			pathLength =1 ;
						 			pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, pss);
						 		}
						 	}

							
							
						
						 if ( pathLength > maxPathLength && pathLength != MaxPathLength) {
								maxPathLength = pathLength;
							}
					}else {
						 pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, null);		
					}						 
					 PAll[robotindex][taskIndex][path] = pathLength;
				}//For path
			}//For task
			//Sum the max path length for each robot
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.minMaxVel,this.minMaxAcc);
			}
		double [][][] PAllAug =  checkTargetGoals(PAll);
		//Save the sum of max paths length to normalize path length cost
		this.sumMaxPathsLength = sumPathsLength;
		//Save the sum of arrival time considering max paths length to normalize delay cost
		this.sumArrivalTime = sumArrivalTime;
		
		//Return the cost of path length	

		return PAllAug;
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
	private double [][][] evaluatePAll(AbstractTrajectoryEnvelopeCoordinator tec){
		
		//Evaluate the path length for the actual couple of task and ID
		//Initialize the sum of max paths lengths and time to do it for each robot
		//This cost are used then for normalizing cost
		double sumPathsLength = 0;
		double sumArrivalTime = 0;
		double [][][] PAll = new double[numRobotAug][numTaskAug][maxNumPaths];
		
		long timeInitial = Calendar.getInstance().getTimeInMillis();
		if(scenario != null) {
			double [][][] PAllScenario = evaluatePAllWithScenario(tec);
			return PAllScenario;
		}
		for (int robotID : IDsAllRobots) {
			double maxPathLength = 1;
			int robotIndex = IDsAllRobots.indexOf(robotID);
			for (int taskID : IDsAllTasks ) {
				int taskIndex = IDsAllTasks.indexOf(taskID);
				
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (IDsIdleRobots.contains(robotID) && IDsRealTasks.contains(taskID) ) {
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(tec.getRobot(robotID));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < maxNumPaths; path++) {
					 final int pathID = path;
					  if(typesAreEqual) { // only if robot and typoe have the same types
							new Thread("Robot" + robotID) {
								public void run() {
										evaluatePathLength(robotID,taskID,pathID,tec);								
								}		
							}.start();
							//Take time to evaluate the path
				
					}			 
					 else {
							 pathsToTargetGoal.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path, null);		
					 }	
					}
			}//For Task
	
		}
		boolean allResultsReady = false;
		while (!allResultsReady) {
			allResultsReady = true;
			
			for (int robotID : IDsIdleRobots) {
				for (int task : IDsAllTasks) {
					for(int path=0;path < maxNumPaths ; path ++) {
						if (!pathsToTargetGoal.containsKey(robotID*numTaskAug*maxNumPaths+task*maxNumPaths+path) )  {
							allResultsReady = false;
						}
					}
					
				}
				try { Thread.sleep(500); }
				catch (InterruptedException e) {
					e.printStackTrace();
				}
				}
				
		}
		long timeFinal = Calendar.getInstance().getTimeInMillis();
		long timeRequired = timeFinal- timeInitial;
		timeRequiretoEvaluatePaths = timeRequiretoEvaluatePaths + timeRequired;
		long timeInitial2 = Calendar.getInstance().getTimeInMillis();
		for (int robotID : IDsAllRobots) {
			int robotindex = IDsAllRobots.indexOf(robotID);
			double maxPathLength = 1;
			for (int taskID : IDsAllTasks ) {
				int taskIndex = IDsAllTasks.indexOf(taskID);
				double pathLength = MaxPathLength;
				//Evaluate path Length
				boolean typesAreEqual = false;
				 if (taskIndex < numTask && robotindex < numRobot ) {
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(tec.getRobot(robotID));
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> they don't have type
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < maxNumPaths; path++) {
					 if(typesAreEqual) { 
						if(pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path)==null) {
							pathLength = MaxPathLength;
						}else {
							 pathLength=Missions.getPathLength(pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+path));
						}
						
						 if ( pathLength > maxPathLength && pathLength != MaxPathLength) {
								maxPathLength = pathLength;
							}
					}
					 
					 PAll[robotindex][taskIndex][path] = pathLength;
				}//For path
			}//For task
			//Sum the max path length for each robot
			sumPathsLength += maxPathLength;
			//Sum the arrival time for the max path length
			sumArrivalTime += computeArrivalTime(maxPathLength,this.minMaxVel,this.minMaxAcc);
			}
		//Take the time to fill in the PAll Matrix
		long timeFinal2 = Calendar.getInstance().getTimeInMillis();
		long timeRequired2 = timeFinal2- timeInitial2;
		timeRequiretofillInPall = timeRequiretofillInPall + timeRequired2;
		double [][][] PAllAug =  checkTargetGoals(PAll);
		//Save the sum of max paths length to normalize path length cost
		this.sumMaxPathsLength = sumPathsLength;
		//Save the sum of arrival time considering max paths length to normalize delay cost
		this.sumArrivalTime = sumArrivalTime;
		//Return the cost of path length	
		Missions.saveScenario("ProvaScenario");
		return PAllAug;
		}
	/**
	 * Evaluate the cost associated to time delay on completion of a task for a specific robot, due to interference with other robot
	 * and precedence constraints. The cost is evaluated considering the intersection between the path of robot i-th
	 * with the paths of other robots, considering the actual Assignment, but also paths related to already driving robot
	 * are considered.
	 * @param robot -> The i-th Robot
	 * @param task -> The j-th Task
	 * @param pathID -> The s-th path
	 * @param assignmentMatrix -> The Assignment Matrix related to a solution of the optimization problem
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
	 */
	private double evaluatePathDelay(int robotID ,int taskID,int pathID,double [][][] assignmentMatrix,AbstractTrajectoryEnvelopeCoordinator tec){
		CriticalSection[][][][] cssMatrix = new CriticalSection [IDsIdleRobots.size()][IDsRealTasks.size()][maxNumPaths][1];
		Integer [] arrayIDs = new Integer[2] ;
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

		int robotIndex = IDsAllRobots.indexOf(robotID);
		int taskIndex = IDsAllTasks.indexOf(taskID);
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		double delay = 0;
		//Considering the Actual Assignment 
		if (assignmentMatrix[robotIndex][taskIndex][pathID]>0) {
			// Only for real robots and tasks
			if (IDsIdleRobots.contains(robotID) && IDsRealTasks.contains(taskID)) {
				//Take the Pose steering relate to i-th robot and j-th task from path set
				//PoseSteering[] pss1 = pathsToTargetGoalTotal.get((robot-1)*numTaskAug*maxNumPaths + task*maxNumPaths +pathID);
				PoseSteering[] pss1 = pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID);
				if(pss1 == null) {
					return delay;
				}
				//Initialize Array of delays for the two robots
				TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
				TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
				//Compute the spatial Envelope for the i-th Robot
				SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,tec.getFootprint(robotID));
				//Evaluate other path depending from the Assignment Matrix
				for(int secondRobotID : IDsIdleRobots) {
					int secondRobotIndex = IDsIdleRobots.indexOf(secondRobotID);
					for(int secondTaskID: IDsRealTasks) {
						int secondTaskIndex = IDsRealTasks.indexOf(secondTaskID);
						 for(int s = 0;s < maxNumPaths; s++) {
							 if (assignmentMatrix [secondRobotIndex][secondTaskIndex][s] > 0 && secondRobotID != robotID && secondTaskID != taskID) {
									//Take the path of this second robot from path set
									
								 	PoseSteering[] pss2 = pathsToTargetGoal.get(secondRobotID*numTaskAug*maxNumPaths  + secondTaskID*maxNumPaths+s);
									if (pss2 != null) {//is == null if robotType is different to Task type
										//Evaluate the Spatial Envelope of this second Robot
										SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,tec.getFootprint(secondRobotID));
										long timeInitial = Calendar.getInstance().getTimeInMillis();
										//Compute the Critical Section between this 2 robot
										arrayIDs[0] = robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID;
										arrayIDs[1] = secondRobotID*numTaskAug*maxNumPaths  + secondTaskID*maxNumPaths+s;
										CriticalSection [] css = new CriticalSection[1];
										/*
										if(criticalSections.containsKey(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID) ) {
											css= criticalSections.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID)[secondRobotIndex][secondTaskIndex][s];
											if(css.length == 1) {
												css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
												
											}
										}else {
											css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
											cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
											criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
											//Evaluate the time to compute critical Section
											long timeFinal = Calendar.getInstance().getTimeInMillis();
											 long timeRequired = timeFinal- timeInitial;
											 timeRequiretoComputeCriticalSection = timeRequiretoComputeCriticalSection + timeRequired;
											 fileStream1.println(timeRequired+"");
					
										}
										*/
										css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
										cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
										criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
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
							    	CriticalSection [] cssDrivingRobot = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, pathsDrivingRobot.get(k),true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
							    	for (int b = 0; b < cssDrivingRobot.length; b++) {
										Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pathsDrivingRobot.get(k).getPath().hashCode(),pathsDrivingRobot.get(k).getPath(),te2TCDelays, cssDrivingRobot[b]);
										double delayCriticalSection = a1.getFirst();
										if(delayCriticalSection < 0 ) {
											delay += 0;
										}else if(delayCriticalSection == Double.POSITIVE_INFINITY) {
											delay += 10000;
										}else {
											delay += delayCriticalSection;
										}
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
	private double [][][] computeArrivalTimeFleet(double[][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double [][][] arrivalTimeMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : IDsIdleRobots ) {
			int i = IDsAllRobots.indexOf(robotID);
			for (int taskID : IDsRealTasks ) {
				int j = IDsAllTasks.indexOf(taskID);
				 for(int path = 0;path < maxNumPaths; path++) {
					 double vel = tec.getRobot(robotID).getForwardModel().getVel();
					 double acc = tec.getRobot(robotID).getForwardModel().getAcc();
					 double arrivalTime = computeArrivalTime(PAll[i][j][path],vel,acc);
					 arrivalTimeMatrix[i][j][path] = arrivalTime;
				 }
				
			}
		}
		
		//Return the arrival time 
		return arrivalTimeMatrix;
	}
	
	
	/**
	 * Evaluate the tardiness in completion of a task for a single robot . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param pathLength -> path length
	 * @param task -> the task j-th
	 * @return
	 */
	
	private double computeTardiness(int robotID,int taskID,double pathLength, AbstractTrajectoryEnvelopeCoordinator tec) {
		double tardiness = 0;
		if(IDsRealTasks.contains(taskID)) {
			int taskIndex = IDsRealTasks.indexOf(taskID);
			if (taskQueue.get(taskIndex).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
				double deadline = taskQueue.get(taskIndex).getDeadline();  //Expressed in seconds
				double vel = tec.getRobot(robotID).getForwardModel().getVel();
				double acc = tec.getRobot(robotID).getForwardModel().getVel();
				double completionTime = computeArrivalTime(pathLength,vel,acc) + taskQueue.get(taskIndex).getOperationTime();
				tardiness = Math.max(0, (completionTime-deadline));
			}	
		}
		return tardiness;
	}
	
	
	/**
	 * Evaluate the tardiness in completion of a task . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param pathLength -> path length
	 * @param task -> the task j-th
	 * @return
	 */
	
	private double[][][] computeTardinessFleet(double [][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec) {
		double tardiness = 0;
		
		double [][][] tardinessMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		for (int robotID : IDsIdleRobots ) {
			int i = IDsAllRobots.indexOf(robotID);
			for (int taskID : IDsRealTasks ) {
				int j = IDsAllTasks.indexOf(taskID);
					for(int path = 0;path < maxNumPaths; path++) {
					if (taskQueue.get(j).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
						double deadline = taskQueue.get(j).getDeadline();  //Expressed in seconds
						double vel = tec.getRobot(robotID).getForwardModel().getVel();
						 double acc = tec.getRobot(robotID).getForwardModel().getVel();
						double completionTime = computeArrivalTime(PAll[i][j][path],vel,acc) + taskQueue.get(j).getOperationTime();
						tardiness = Math.max(0, (completionTime-deadline));
						tardinessMatrix[i][j][path] = tardiness;
						sumTardiness += tardiness;
					}	
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
	private double [][][] evaluateBFunction(double [][][]PAll,AbstractTrajectoryEnvelopeCoordinator tec){
		double [][][] tardinessMatrix = computeTardinessFleet(PAll,tec);
		double [][][] BFunction = new double [numRobotAug][numTaskAug][maxNumPaths];
		costValuesMatrix = new double [numRobotAug][numTaskAug][maxNumPaths];
		if(linearWeight == 1) {
			double [][][] arrivalTimeMatrix = computeArrivalTimeFleet(PAll,tec);
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					for(int path = 0;path < maxNumPaths; path++) {
						
						BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeWeight*arrivalTimeMatrix[i][j][path]/sumArrivalTime;
						costValuesMatrix[i][j][path] = BFunction[i][j][path];
						//costValuesMatrix[i][j][path] =  PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeMatrix[i][j][path]/sumArrivalTime;
					}
					
				}
			}
		}
		else {
			for (int i = 0 ; i < numRobotAug; i++) {
				for (int j = 0 ; j < numTaskAug; j++) {
					for(int path = 0;path < maxNumPaths; path++) {
						BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness;
						//costValuesMatrix[i][j][path] = PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness;
						costValuesMatrix[i][j][path] = BFunction[i][j][path] ;
							
					
					}
					

	
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
    		double [][][] assignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblemCopy);
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
	public double [][][][] evaluateFeasibleSolution(int numRobot,int numTasks){
		//Define the optimization problem
		MPSolver optimizationProblemCopy = buildOptimizationProblem(numRobot,numTasks);
		//Evaluate the number of all feasible solution for the optimization problem
	    int feasibleSolutions = numberFeasibleSolution(numRobot,numTasks);
	    //Initialize a set to store all feasible solution
		double [][][][] AssignmentMatrixOptimalSolutions = new double [feasibleSolutions][numRobot][numTasks][maxNumPaths]; 
	    ///////////////////////////////////////
	    for(int k=0; k < feasibleSolutions; k++) {
			//Solve the optimization problem
	    	MPSolver.ResultStatus resultStatus = optimizationProblemCopy.solve();
			//Transform the Assignment Vector to Matrix
			double [][][] AssignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblemCopy);
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
	 * @param numTasksAug -> Number of Tasks.
	 * @return A constrained optimization problem without the objective function
	 */
	private MPSolver buildOptimizationProblem(int numRobotAug,int numTasksAug) {
		//Initialize a linear solver 
		
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		//START DECISION VARIABLE VARIABLE
		MPVariable [][][] decisionVariable = new MPVariable[numRobotAug][numTasksAug][maxNumPaths];
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTasksAug; j++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 decisionVariable[i][j][s] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+","+s+"]");
				 }
				
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
				 for(int s = 0; s < maxNumPaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 }
				
			 }
		 }
		//Each task can be performed only by a robot
		 for (int j = 0; j < numTasksAug; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numRobotAug; i++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 } 		
			 }
		 }
		 
		 
		 
		 for (int robotID : IDsAllRobots ) {
				int i = IDsAllRobots.indexOf(robotID);
				for (int taskID : IDsAllTasks ) {
					int j = IDsAllTasks.indexOf(taskID);
					for(int s = 0; s < maxNumPaths; s++) {
							 if (i < numRobot) { //Considering only real Robot
								 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s);
								 if(pss==null) {
									 MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
									 c3.setCoefficient(decisionVariable[i][j][s],1); 
								 }
							 }
					}
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
						 for(int s = 0; s < maxNumPaths; s++) {
							 //Build the constraint
							 c3.setCoefficient(decisionVariable[i][j][s], 1); 
						 } 		
					 }
				 }
			 }
		 }
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}
	
	
	public static double probability(double f1, double f2, int iteration) {
        if (f2 < f1) return 1;
        return 1-Math.exp((f1 - f2) / iteration);
    }
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions with a Local Search Algorithm. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks. Starting from a feasible solution, all his neighbors are computed; then the new solution 
	 * is the optimal solutions considering the neighbors. The algorithm go on until the specified number of iterations
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @param iteration -> number of iteration to consider ( if -1 all feasible solution are considered)
	 * @return An Optimal Assignment that minimize the objective function
	 */
	
	public double [][][] simulatedAnnealingAlgorithm(AbstractTrajectoryEnvelopeCoordinator tec,int iteration){
		long initialTime = 	Calendar.getInstance().getTimeInMillis();
		PrintStream fileStream = null;
		PrintStream fileStream1 = null;
		PrintStream fileStream3 = null;
		PrintStream fileStream4 = null;
		PrintStream fileStream5 = null;
		PrintStream fileStream7 = null;
		PrintStream fileStream8 = null;
		PrintStream fileStream9 = null;
		Random rand = new Random(3455343);
		
		
		
		int a = tec.getTestNumber();
		int b = tec.getFolderNumber();
		String ppRequiredTime = "Test"+b+"/SA/RequiredTime-T" + a +".txt";
		String ppCriticalSections = "Test"+b+"/SA/CriticalSections-T" + a +".txt";
		String ppPathDelay = "Test"+b+"/SA/PathDelay-T" + a +".txt";
		String ppCostOptimalSolution = "Test"+b+"/SA/CostOptimalSolution-T" + a +".txt";
		String ppAssignMatrix = "Test"+b+"/SA/AssignMatrix-T" + a +".txt";
		String ppTotalTime = "Test"+b+"/SA/TotalTime-T" + a +".txt";
		String ppAnalysisProblem = "Test"+b+"/SA/AnalysisProblem-T" + a +".txt";
		String ppAssignMatrix2 = "Test"+b+"/SA/AssignMatrix2-T" + a +".txt";
		String ppCostObjectiveFunction = "Test"+b+"/SA/CostObjectiveFunction-T" + a +".txt";
		
		try {
			/*
			fileStream = new PrintStream(new FileOutputStream("Test1/SA/RequiredTime.txt",true));
			fileStream1 = new PrintStream(new FileOutputStream("Test1/SA/CriticalSections.txt",true));
			PrintStream fileStream2 = new PrintStream(new FileOutputStream("Test1/SA/PathDelay.txt",true));
			fileStream3 = new PrintStream(new FileOutputStream("Test1/SA/CostOptimalSolution.txt",true));
			fileStream4 = new PrintStream(new FileOutputStream("Test1/SA/AssignMatrix.txt",true));
			fileStream5 = new PrintStream(new FileOutputStream("Test1/SA/TotalTime.txt",true));
			fileStream8 = new PrintStream(new FileOutputStream("Test1/SA/AssignMatrix2.txt",true));
			fileStream7 = new PrintStream(new FileOutputStream("Test1/SA/AnalysisProblem.txt",true));
			fileStream9 = new PrintStream(new FileOutputStream("Test1/SA/CostObjectiveFunction.txt",true));
			*/
			
			
			fileStream = new PrintStream((new FileOutputStream(ppRequiredTime,true)));
			fileStream1 = new PrintStream(new File(ppCriticalSections));
			PrintStream fileStream2 = new PrintStream(new File(ppPathDelay));
			fileStream3 = new PrintStream(new File(ppCostOptimalSolution));
			fileStream4 = new PrintStream(new FileOutputStream(ppAssignMatrix,true));
			fileStream5 = new PrintStream(new FileOutputStream(ppTotalTime,true));
			
			//fileStream5 = new PrintStream(new File("TotalTime.txt"));
			fileStream7 = new PrintStream(new FileOutputStream(ppAnalysisProblem,true));
			fileStream8 = new PrintStream(new FileOutputStream(ppAssignMatrix2,true));
			fileStream9 = new PrintStream(new FileOutputStream(ppCostObjectiveFunction,true));
			
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
		getAllRobotIDs();
		getAllTaskIDs();
	
		//Consider possibility to have dummy Robot or Tasks
		double [][][] PAll = evaluatePAll(tec);
		
		double[][][] BFunction = evaluateBFunction(PAll,tec);
		//Build the optimization Problem without the objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		MPObjective objective = optimizationProblem.objective();
		objective.setMinimization();
		//Initialize the optimal assignment and the cost associated to it
		double [][][] optimalAssignmentMatrix = new double[numRobotAug][numTaskAug][maxNumPaths];
		double objectiveOptimalValue = 100000000;
		double costBOptimal = 0;
		double costFOptimal = 0;
		//Solve the optimization problem
		optimizationProblem.solve();
		double [][][] AssignmentMatrix = saveAssignmentMatrix(numRobotAug,numTaskAug,optimizationProblem);
		int prova1 = 0;
		int prova2 = 0;
		
		int numIteration =1;
		for(int fat= 1 ; fat <= numRobotAug; fat++) {
			numIteration *= fat;
		}
		if(iteration > numIteration || iteration == -1) {
			iteration =  numIteration;
		}else {
			numIteration = iteration;
		}
		int index1i = 0;
		int index1j = 0;
		int index1s = 0;
		int index2i = 0;
		int index2j = 0;
		int index2s = 0;
		double [][][] newAssignmentMatrix = new double[numRobotAug][numTaskAug][maxNumPaths];
		for(int i=0; i< AssignmentMatrix.length;i ++) {
			for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
				for(int s = 0; s < maxNumPaths; s++) {
					newAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];
				}
			}
		}
		boolean solutionAlreadyFound = false;
		double[][][][] matrixSolutionFound= new double[iteration+1][numRobotAug][numTaskAug][maxNumPaths];
		boolean [] solutionFound = new boolean [numRobotAug*numTaskAug*maxNumPaths];
		double [] costSolutions = new double [iteration+1];
		int indexSolutionFound = 0;
		ArrayList <Integer> IDsRandomRobots = new ArrayList <Integer>();
		ArrayList <Integer> IDsRandomRobots2 = new ArrayList <Integer>();
		IDsRandomRobots.addAll(IDsAllRobots);
		//IDsRandomRobots2.addAll(IDsAllRobots);
		//IDsRandomRobots2.addAll(IDsAllRobots);
		int ind  = (int) Math.floor(rand.nextDouble()*IDsRandomRobots.size());
		prova1 = IDsRandomRobots.get(ind);

		for(int m: IDsAllRobots) {
			if(IDsIdleRobots.contains(m) && tec.getRobot(prova1)!=null){
				tec.getRobot(m).getRobotType();
				tec.getRobot(prova1).getRobotType();
				if(tec.getRobot(m).getRobotType() == tec.getRobot(prova1).getRobotType()) {
					IDsRandomRobots2.add(m);
					
				}
			}else {//virtual robots
				IDsRandomRobots2.add(m);
			}
		}
		int index = IDsRandomRobots2.indexOf(prova1);
		IDsRandomRobots2.remove(index);
		//IDsRandomRobots.remove(ind);
		if(ScenarioAllocation!= null) {
			return this.ScenarioAllocation;
		}
		fileStream7.println("Tasks>> "+ IDsRealTasks.size() + " Robot>> " + IDsIdleRobots.size() );
		
		long timeOffsetInitial = Calendar.getInstance().getTimeInMillis();
		long timeOffset = 0;
		
		for(int k=0; k <=  iteration ;k++){
			if(timeOffset < timeOut) {
			//Evaluate an optimal assignment that minimize only the B function
			//Initialize cost of objective value
			
			double costofAssignment = 0;
			double costofAssignmentForConstraint = 0;
			double costBFunction = 0;
			double costFFunction = 0;
			if(k>0) {	
					if(IDsRandomRobots2.size() == 0) {
						//prova1 += 1;
						if(IDsRandomRobots.size()==0) {
							IDsRandomRobots.addAll(IDsAllRobots);
						}
						ind  = (int) Math.floor(rand.nextDouble()*IDsRandomRobots.size());
						prova1 = IDsRandomRobots.get(ind);
						IDsRandomRobots.remove(ind);
						for(int m: IDsAllRobots) {
							if(IDsIdleRobots.contains(m) && tec.getRobot(prova1)!=null){
								tec.getRobot(m).getRobotType();
								tec.getRobot(prova1).getRobotType();
								if(tec.getRobot(m).getRobotType() == tec.getRobot(prova1).getRobotType()) {
									IDsRandomRobots2.add(m);
									
								}
							}else {//virtual robots
								IDsRandomRobots2.add(m);
							}
						}
						index = IDsRandomRobots2.indexOf(prova1);
						IDsRandomRobots2.remove(index);
			
					}
					//prova2 +=1;
					int ind2 = (int) Math.floor(rand.nextDouble()*IDsRandomRobots2.size());
					if(IDsRandomRobots2.size()>0) {
						prova2 = IDsRandomRobots2.get(ind2);
						IDsRandomRobots2.remove(ind2);
					}
				for(int i=0; i< AssignmentMatrix.length;i ++) {
					for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
						for(int s = 0; s < maxNumPaths; s++) {
							newAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];
						}
					}
				}
				
				for (int robotID : IDsAllRobots ) {
					int i = IDsAllRobots.indexOf(robotID);
					for (int taskID : IDsAllTasks ) {
						int j = IDsAllTasks.indexOf(taskID);
						for(int s = 0; s < maxNumPaths; s++) {
							if (AssignmentMatrix[i][j][s]==1  && robotID == prova1) {
								index1i = i;	
								index1j = j;
								index1s = s;
							}
							if(AssignmentMatrix[i][j][s]==1 && robotID == prova2) {
								index2i = i;	
								index2j = j;
								index2s = s;		
							}
						}
					}
				}
				
				newAssignmentMatrix[index1i][index1j][index1s] = 0;
				newAssignmentMatrix[index1i][index2j][index1s] = 1;
				newAssignmentMatrix[index2i][index1j][index2s] = 1;
				newAssignmentMatrix[index2i][index2j][index2s] = 0;
				
				for(int it=0; it < k;it ++) {
					for(int i=0; i< AssignmentMatrix.length;i ++) {
						for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
							for(int s = 0; s < maxNumPaths; s++) {
								if(matrixSolutionFound[it][i][j][s] ==  newAssignmentMatrix[i][j][s]) {
									solutionFound[i*numTaskAug*maxNumPaths+j*maxNumPaths+s] = true;
								}else {
									solutionFound[i*numTaskAug*maxNumPaths+j*maxNumPaths+s] = false;
								}
							}
						}
					}
					solutionAlreadyFound = true;
					for(int f=0; f < solutionFound.length;f++) {
						if(!solutionFound[f]) {
							solutionAlreadyFound = false;
							break;
						}
					}
					if(solutionAlreadyFound) {
						indexSolutionFound = it;
						break;
					}
				}
			}
			solutionAlreadyFound = false;
			if(!solutionAlreadyFound) {
				fileStream4.println("-------------------");
				fileStream4.println("Solution not already found" + solutionAlreadyFound);
				fileStream4.println("Switch" + prova1 + "--" + prova2 );
				fileStream4.println("-------------------");
				
				//Evaluate the cost of F Function for this Assignment
				timeRequiretoComputeCriticalSection = 0;
				timeRequiretoComputePathsDelay = 0;
				//Take time to understand how much time require this function
				for (int robotID : IDsAllRobots ) {
					int i = IDsAllRobots.indexOf(robotID);
					for (int taskID : IDsAllTasks ) {
						int j = IDsAllTasks.indexOf(taskID);
						for(int s = 0; s < maxNumPaths; s++) {
							if ( newAssignmentMatrix[i][j][s] > 0) {
								if (linearWeight != 1) {
									//Evaluate cost of F function only if alpha is not equal to 1
									costBFunction = BFunction[i][j][s];
									costFFunction = evaluatePathDelay(robotID,taskID,s,newAssignmentMatrix,tec)/sumArrivalTime;
									costofAssignment = linearWeight*costBFunction + (1-linearWeight)*costFFunction + costofAssignment ;
									costofAssignmentForConstraint = costValuesMatrix[i][j][s] + costFFunction + costofAssignmentForConstraint;
					
								}
								else {
									costBFunction= BFunction[i][j][s];
									costofAssignment = Math.pow(linearWeight*costBFunction, 2) + costofAssignment ;
									costofAssignmentForConstraint = costValuesMatrix[i][j][s]  + costofAssignmentForConstraint;
								}

							}
							matrixSolutionFound[k][i][j][s] =  newAssignmentMatrix[i][j][s];
						}
										
					}		
				}
				costSolutions[k] = costofAssignment;
			}else {
				fileStream4.println("-------------------");
				fileStream4.println("Solution already found" + solutionAlreadyFound);
				fileStream4.println("Switch was" + prova1 + "--" + prova2 );
				fileStream4.println("-------------------");
				costofAssignment =  costSolutions[indexSolutionFound];
				costSolutions[k] = costofAssignment;
			}
			
			fileStream4.println("-------------------");
			fileStream4.println(costofAssignment+"");
			fileStream4.println("-------------------");
			for(int i=0; i< AssignmentMatrix.length;i ++) {
				for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
					for(int s = 0; s < maxNumPaths; s++) {
						fileStream4.println(newAssignmentMatrix[i][j][s]+"--" +(i+1) + "--"+(j+1)+"--"+(s+1));
					}
				}
			}
			fileStream4.println("-------------------");
			for(int i=0; i< AssignmentMatrix.length;i ++) {
				for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
					for(int s = 0; s < maxNumPaths; s++) {
						fileStream4.println(AssignmentMatrix[i][j][s]+"--" +(i+1) + "--"+(j+1)+"--"+(s+1));
					}
				}
			}
			fileStream.println(timeRequiretoEvaluatePaths+"");
			fileStream.println(timeRequiretofillInPall+"");
			fileStream.println(timeRequiretoComputeCriticalSection+"");
			fileStream.println(timeRequiretoComputePathsDelay+"");
			//Compare actual solution and optimal solution finds so far
			if(k> 0) {
				
				double kk= rand.nextDouble();
				double kk2 = probability(costSolutions[k-1],costofAssignment,k);

				if(kk < kk2) {
					
					for(int i=0; i< AssignmentMatrix.length;i ++) {
						for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
							for(int s = 0; s < maxNumPaths; s++) {
								AssignmentMatrix[i][j][s] = newAssignmentMatrix[i][j][s];
							}
						}
					}
				}
			}
			
			if (costofAssignment < objectiveOptimalValue ) {
				objectiveOptimalValue = costofAssignment;
				//optimalAssignmentMatrix = AssignmentMatrix;
				costBOptimal = costofAssignmentForConstraint;
				costFOptimal =  costofAssignmentForConstraint- costBOptimal;
				for(int i=0; i< AssignmentMatrix.length;i ++) {
					for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
						for(int s = 0; s < maxNumPaths; s++) {
							optimalAssignmentMatrix[i][j][s] = newAssignmentMatrix[i][j][s];
						}
					}
				}
			}
			long timeOffsetFinal = Calendar.getInstance().getTimeInMillis();
			timeOffset = timeOffsetFinal - timeOffsetInitial;
			
		}
			
		}
		fileStream9.println(objectiveOptimalValue);
		for (int robotID : IDsAllRobots ) {
			int i = IDsAllRobots.indexOf(robotID);
			for (int taskID : IDsAllTasks ) {
				int j = IDsAllTasks.indexOf(taskID);
				for(int s = 0; s < maxNumPaths; s++) {
					if(optimalAssignmentMatrix[i][j][s] == 1) {
						fileStream8.println(optimalAssignmentMatrix[i][j][s]+"--" + robotID + "--"+ taskID +"--" +(s+1));
					}
				}
			}
	}

		fileStream8.println("");
		 long timeFinal = Calendar.getInstance().getTimeInMillis();
		 long timeRequired = timeFinal- initialTime;
		 fileStream5.println(timeRequired+"");
		//Return the Optimal Assignment Matrix 
		//writeMatrix("Test1/SA/MatrixOptimal.txt",optimalAssignmentMatrix);
		String ppMatrixOptimal = "Test"+b+"/SA/MatrixOptimal-T" + a +".txt";
		writeMatrix(ppMatrixOptimal,optimalAssignmentMatrix);
		this.ScenarioAllocation = null;
		this.scenario = null;
		return  optimalAssignmentMatrix;    
	}
	
	
	/**
	 * Perform the task Assignment defining the mission for each robot
	 * @param AssignmentMatrix -> An Assignment Matrix of the optimization problem
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return An updated Trajectory Envelope Coordinator Simulation in which the mission for each
	 * robot is defined
	 */
	public boolean TaskAllocation(double [][][] AssignmentMatrix,AbstractTrajectoryEnvelopeCoordinator tec){
		System.out.println("Number of Robot : " + numRobot);
		System.out.println("Number of Task : " + numTask);
		System.out.println("Number of dummy Robot : " + dummyRobot);
		System.out.println("Number of dummy Task : " + dummyTask);
		System.out.println("Total Number of Robot : " + numRobotAug);
		System.out.println("Total Number of Task : " + numTaskAug);
		for (int robotID : IDsAllRobots ) {
			int i = IDsAllRobots.indexOf(robotID);
			for (int taskID : IDsAllTasks ) {
				int j = IDsAllTasks.indexOf(taskID);
				for(int s = 0; s < maxNumPaths; s++) {
					 if (AssignmentMatrix[i][j][s] > 0) {
						 if (i < numRobot) { //Considering only real Robot
							 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s);
							 //For Dispatch mission
							 if (j < numTask && pss != null) {
								 taskQueue.get(j).assignRobot(robotID);
								 taskQueue.get(j).setPaths(pss);
								 Mission[] robotMissions = taskQueue.get(j).getMissions();
								 viz.displayTask(taskQueue.get(j).getStartPose(), taskQueue.get(j).getGoalPose(),taskID, "red");
								 //tec.addMissions(new Mission(IDsIdleRobots[i],pss));
								 System.out.println("Task # "+ taskID + " is Assigned");
								 tec.setTaskAssigned(robotID,taskID);
								 tec.addMissions(robotMissions);
							 }else {
								 System.out.println("Virtual Task # "+ taskID + " is Assigned to a real robot");
							 }
						 }else{
							
							 
							
							 System.out.println("Task # "+ taskID + " is not Assigned to a real robot");
							 
						 }
					 }
					//Remove path from the path set
					pathsToTargetGoal.remove(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+s);
				 }
				 
			 }
		 }
		//Remove Assigned Tasks from the set	
		int i = 0;
		int cont = 0;
		while (i < Math.min(numRobot, numTask)) {
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
		 IDsAllRobots.removeAll(IDsAllRobots);
		 IDsAllTasks.removeAll(IDsAllTasks);
		 IDsRealTasks.removeAll(IDsRealTasks);
		 IDsIdleRobots.removeAll(IDsIdleRobots);
		 
		
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
						double [][][] assignmentMatrix = simulatedAnnealingAlgorithm(coordinator,-1);
						for (int i = 0; i < assignmentMatrix.length; i++) {
							int robotID = IDsAllRobots.get((i));
							for (int j = 0; j < assignmentMatrix[0].length; j++) {
								int taskID = IDsAllTasks.get((j));
								for(int s = 0; s < maxNumPaths; s++) {
									System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ assignmentMatrix[i][j][s]);
									if (assignmentMatrix[i][j][s] == 1) {
										System.out.println("Robot " + robotID +" is assigned to Task "+ taskID +" throw Path " + (s+1));
									}
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
	}

