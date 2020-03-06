package se.oru.coordination.coordination_oru.taskassignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
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
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.tests.icaps2018.eval.TrajectoryEnvelopeCoordinatorSimulationICAPS;
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


public class TaskAssignment{
	protected static int numRobot;
	protected static int numTask;
	protected static double infinity = java.lang.Double.POSITIVE_INFINITY;
	protected static double totalPathsLength;
	protected static double maxPathLength = 1;
	protected static boolean approximationFlag;
	protected static int dummyRobot;
	protected static int dummyTask;
	protected static int numRobotAug;
	protected static int numTaskAug;
	protected static Task [] TasksMissions;
	protected static Integer[] IdleRobots;
	//FleetMaster Interface Parameters
	ArrayList <PoseSteering[]> pathsToTargetGoal = new ArrayList <PoseSteering[]>();
	
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	protected boolean propagateDelays = false;
	protected static Logger metaCSPLogger = MetaCSPLogging.getLogger(TrajectoryEnvelopeCoordinator.class);
	

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
	 * @param boundingBox 
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
			metaCSPLogger.info("[estimateTimeToCompletionDelays] te1TCDelays: " + te1TCDelays.toString());
			CumulatedIndexedDelaysList te2TCDelays = toIndexedDelaysList(delaysRobot2, pss2.length);
			metaCSPLogger.info("[estimateTimeToCompletionDelays] te2TCDelays: " + te2TCDelays.toString());
			return fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays,pss1,pss2);
		}
		
		return new Pair<Double, Double> (Double.NaN, Double.NaN);
	}
	
	/**
	 * Considering the possibility to have a different number of robots (N) and tasks (M). If N > M, dummy tasks are 
	 * considered, where a dummy task is a task for which a robot stay in starting position; while if M > N dummy robots
	 * are considered, where a dummy robot is only a virtual robot
	 * @param numRobot -> Number of Robots
	 * @param numTasks -> Number of Tasks
	 * @return DummyVector -> A vector that contains the number of dummy robots (First Position) and tasks(Second Position)
	 */

	private static int[]  dummyRobotorTask(int numRobot, int numTasks) {
		int [] DummyVector = {numRobot,numTasks};
		numRobotAug = numRobot;
		numTaskAug = numTasks;
		//Considering the possibility to have n != m
		//If n > m -> we have dummy robot, so at some robot is assign the task to stay in starting position
		if(numRobot > numTasks) {
			dummyTask = numRobot - numTasks;
			numTaskAug = numTasks + dummyTask;
			DummyVector[1] = dummyTask;
		}
		else if(numRobot < numTasks) {
			dummyRobot = numTasks - numRobot;
			numRobotAug = numRobot + dummyRobot;
			DummyVector[0] = dummyRobot;
		}
		return DummyVector;
	}

	
	/**
	 * Transform a 1D array of MPVariable into a 2D MATRIX  
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblem}
	 * @return 2D Matrix of Decision_Variable
	 */
	private static MPVariable [][] tranformArray(MPSolver optimizationProblem) {
		int numRobot = numRobotAug;
		int numTasks = numTaskAug;
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][] decisionVariable = new MPVariable [numRobot][numTasks];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobot; i++) {
			 for (int j = 0; j < numTasks; j++) {
				 decisionVariable[i][j] = array1D[i*numTasks+j];
			 }
	    }
		return decisionVariable;
	}
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on previous optimal solution found  
	 */
	private static MPSolver constraintOnPreviousSolution(MPSolver optimizationProblem, double [][] assignmentMatrix) {
		//Take decision Variable from Optimization Problem
		MPVariable [][] DecisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		MPConstraint c2 = optimizationProblem.makeConstraint(-infinity,1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    				if(assignmentMatrix[i][j] >0) {
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
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on previous optimal solution found  
	 */
	private static MPSolver constraintOnCostSolution(MPSolver optimizationProblem,double objectiveValue) {
		//Take the vector of Decision Variable from the input solver
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		MPConstraint c3 = optimizationProblem.makeConstraint(-infinity,objectiveValue);
		//Define a constraint for which the new optimal solution considering only B must have a cost less than the min of 
		//previous assignment considering the sum of B and F
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			c3.setCoefficient(decisionVariable[i][j],1);
    			}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}
	
	
	/**
	 * Compute the Assignment matrix given a solved optimization problem
	 * @param numRobot -> Number of robots
	 * @param numTasks -> Number of tasks
	 * @param optimizationProblem -> A solved optimization problem
	 * @return Assignment matrix for the optimization problem given as input
	 */
	
	private static double [][] saveAssignmentMatrix(int numRobot,int numTasks,MPSolver optimizationProblem){
		//Take the decision variable from the optimization problem
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem);
		double [][] assignmentMatrix = new double [numRobot][numTasks];	
		//Store the Assignment Matrix
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
	 * @param robot_ID -> The ID of the Robot
	 * @param tasks_ID -> The ID of the Task
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private double evaluatePathLength(int robot , int task, ReedsSheppCarPlanner rsp, ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec){
		
		//Evaluate the path length for the actual couple of task and ID
		double pathLength = 10000000;
		// N = M -> Number of Robot = Number of Tasks
		//Take the state for the i-th Robot
		RobotReport rr = tec.getRobotReport(robot);
		if (rr == null) {
			metaCSPLogger.severe("RobotReport not found for Robot" + robot + ".");
			throw new Error("RobotReport not found for Robot" + robot + ".");
		}
		if (robot <= numRobot && task < numTask) {
			//Evaluate the path from the Robot Starting Pose to Task End Pose
			rsp.setStart(rr.getPose());
			rsp.setGoals(tasks.get(task).getStartPose(),tasks.get(task).getGoalPose());
			if (!rsp.plan()) {
				//the path to reach target end not exits
				return pathLength;
			}
			//Take the Pose Steering representing the path
			PoseSteering[] pss = rsp.getPath();
			//Add the path to the FleetMaster Interface 
			addPath(robot, pss.hashCode(), pss, null, tec.getFootprint(robot));
			//Save the path to Task 
			pathsToTargetGoal.add(pss);
			//Take the Path Length
			pathLength = Missions.getPathLength(pss);
		} else { // N != M
			if (numRobot > numTask){ //dummy task -> The Robot receive the task to stay in starting position
				//Create the task to stay in robot starting position
				PoseSteering[] dummyTask = new PoseSteering[1];
				dummyTask[0] = new PoseSteering(rr.getPose(),0);
				//Add the path to the FleetMaster Interface 
				addPath(robot, dummyTask.hashCode(), dummyTask, null, tec.getFootprint(robot));
				//Save the path to Dummy Task 
				pathsToTargetGoal.add(dummyTask);
				pathLength = 1;
				return pathLength;
			}
			else { //dummy robot -> Consider a only virtual Robot 
				pathLength = 1;
				return pathLength;
			}	
		}	
		//Return the cost of path length
		return pathLength;
	}
	
	
	/**
	 * Evaluate the cost associated to the path length for the a couple of robot and task.
	 * If a path between a couple of robot and task does not exists the cost is consider infinity.
	 * @param robot_ID -> The ID of the Robot
	 * @param tasks_ID -> The ID of the Task
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private double[][] evaluatePAll(ReedsSheppCarPlanner rsp, ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec){
		//Evaluate the path length for the actual couple of task and ID
		double pathLength = 10000000;
		double biggerPathLength = 0;
		double [][] PAll = new double[numRobotAug][numTaskAug];
		for (int robot= 0; robot < numRobotAug; robot++) {
			for (int task = 0; task < numTaskAug; task++ ) {
				//Take the state for the i-th Robot
				RobotReport rr = tec.getRobotReport(robot);
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robot + ".");
					throw new Error("RobotReport not found for Robot" + robot + ".");
				}
				
				// N = M -> Number of Robot = Number of Tasks
				if (robot <= numTask && task < numRobot) {
					//Evaluate the path from the Robot Starting Pose to Task End Pose
					rsp.setStart(rr.getPose());
					rsp.setGoals(tasks.get(task).getStartPose(),tasks.get(task).getGoalPose());
					if (!rsp.plan()) {
						//the path to reach target end not exits
						PAll[robot][task] = pathLength;
					}
					//Take the Pose Steering representing the path
					PoseSteering[] pss = rsp.getPath();
					//Add the path to the FleetMaster Interface 
					addPath(robot, pss.hashCode(), pss, null, tec.getFootprint(robot));
					//Save the path to Task 
					pathsToTargetGoal.add(pss);
					//Take the Path Length
					pathLength = Missions.getPathLength(pss);
					if ( pathLength > biggerPathLength) {
						biggerPathLength = pathLength;
					}
					PAll[robot][task] = pathLength;
				}
				else { // N != M
					if (numRobot > numTask){ //dummy task -> The Robot receive the task to stay in starting position
						PoseSteering[] dummyTask = new PoseSteering[1];
						dummyTask[0] = new PoseSteering(rr.getPose(),0);
						//Add the path to the FleetMaster Interface 
						addPath(robot, dummyTask.hashCode(), dummyTask, null, tec.getFootprint(robot));
						//Save the path to Dummy Task 
						pathsToTargetGoal.add(dummyTask);
						PAll[robot][task] =  1;
					}
					else { //dummy robot -> Consider a only virtual Robot 
						PAll[robot][task] =  1;
					}	
				}	
			}
		}
		//Save the max path length to normalize path length cost
		maxPathLength = biggerPathLength;
		//Return the cost of path length
		return PAll;
		}
	
	/**
	 * Evaluate the cost associated to time delay on completion of a task for a specific robot, due to interference with other robot
	 * and precedence constraints
	 * @param robot_ID -> The ID of the Robot
	 * @param tasks_ID -> The ID of the Task
	 * @param Assignment_Matrix -> The Assignment Matrix of a solution
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose 
	 * @param tec -> an TrajectoryEnvelopeCoordinatorSimulation
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblemWithB}
	 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
	 */
	
	private double evaluatePathDelay(int robot ,int task,double [][] assignmentMatrix,ReedsSheppCarPlanner rsp,ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec,MPSolver optimizationProblem){
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		double delay = 0;
		//Considering the Actual Assignment 
		if (assignmentMatrix[robot-1][task]>0) {
			// N = M
			if(task < numTask && robot <= numRobot) {
				//Evaluate the path for this couple of robot and task
				//Evaluate the path length from Robot Starting Position to Task End Position
				//rsp.setStart(tec.getRobotReport(robot).getPose());
				//rsp.setGoals(tasks.get(task).getStartPose(),tasks.get(task).getGoalPose());
				//rsp.plan();
				//PoseSteering[] pss1 = rsp.getPath();
				PoseSteering[] pss1 = pathsToTargetGoal.get((robot-1)*assignmentMatrix[0].length + task);	
				//If the B Matrix is evaluate considering an approximation, update the values of B with correct values
				if(approximationFlag) {
					double Path_length = Missions.getPathLength(pss1);
					optimizationProblem.objective().setCoefficient(optimizationProblem.variables()[robot*assignmentMatrix[0].length+task], Path_length);
				}
				
				//Initialize Array of delays for thw two robots
				TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
				TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
				//Compute the spatial Envelope of this Robot
				SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,tec.getFootprint(robot));
				for(int m = 0; m < assignmentMatrix.length; m++) {
					for(int n = 0; n < assignmentMatrix[0].length; n++) {
						if (assignmentMatrix [m][n] > 0 && m+1 != robot && n != task && n < numTask && m < numRobot) {
							//Evaluate the path length from Robot Starting Position to Task End Position
							//rsp.setStart(tec.getRobotReport(m+1).getPose());
							//rsp.setGoals(tasks.get(n).getStartPose(),tasks.get(n).getGoalPose());
							//rsp.plan();
							//PoseSteering[] pss2 = rsp.getPath();
							PoseSteering[] pss2 = pathsToTargetGoal.get((m)*assignmentMatrix[0].length  + n);
							//addPath(robot, pss2.hashCode(), pss2, null, tec.getFootprint(robot));
							//Evaluate the Spatial Envelope of this second Robot
							SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,tec.getFootprint(m+1));
							//Compute the Critical Section between this 2 robot
							CriticalSection [] css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robot).getArea(),tec.getFootprintPolygon(m+1).getArea()));
							//Compute the delay due to precedence constraint in Critical Section
							for (int g = 0; g < css.length; g++) {
								Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pss2.hashCode(),pss2,te2TCDelays, css[g]);
								System.out.println("Prova Delay" + a1.getFirst() +" >> "+ a1.getSecond());
							}
						}
					}
				}
				
			} else { // N != M
				if(numRobot > numTask){ //dummy task
					return delay;			
				}else { //dummy robot
					
					return delay;
				}
			}
		}
		//return the delay for the i-th robot and the j-th task due to interference with other robots
		return delay;
		
		}
	/**
	 * * Evaluate an approximation of the cost associated to the path length for the a couple of robot and task considering the Euclidean distance .
	 * If a path between a couple of robot and task does not exists the cost is consider infinity
	 * @param robot_ID -> The ID of the Robot
	 * @param tasks_ID -> The ID of the Task
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private static double evaluateApproximatePathLength(int robot ,int task, ReedsSheppCarPlanner rsp, ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec){
		//Take the state for the i-th Robot
		RobotReport rr = tec.getRobotReport(robot);
		if (rr == null) {
			metaCSPLogger.severe("RobotReport not found for Robot" + robot + ".");
			throw new Error("RobotReport not found for Robot" + robot + ".");
		}
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double x1 = 0;
		double y1 = 0;
		double x2 = 0;
		double y2 = 0;
		double pathLength = 0;
		double pathToTaskStart = 0;
		if(task < numTask && robot < numRobot) {
			// N = M
			//Evaluate the path length to reach the Target Start
			x1 = tec.getRobotReport(robot+1).getPose().getX();
			y1 = tec.getRobotReport(robot+1).getPose().getY();
			x2 = tasks.get(task).getStartPose().getX();
			y2 = tasks.get(task).getStartPose().getY();
			pathToTaskStart = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
			//Evaluate the path length from Target Start to Target End
			x1 = tasks.get(task).getStartPose().getX();
			y1 = tasks.get(task).getStartPose().getY();
			x2 = tasks.get(task).getGoalPose().getX();
			y2 = tasks.get(task).getGoalPose().getY();
			//Evaluate the Euclidean distance
			pathLength = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2)) + pathToTaskStart;
	
		}else {// N != M
			if(numRobot > numTask){
				//dummy task
				pathLength = 1;
				return pathLength;
			}else {
				//dummy robot
				pathLength = 1;
				return pathLength;
			}
		}
		//Return the cost of path length
		return pathLength;
		}
	
	
	/**
	 * * Evaluate an approximation of the cost associated to the path length for the a couple of robot and task considering the Euclidean distance .
	 * If a path between a couple of robot and task does not exists the cost is consider infinity
	 * @param robot_ID -> The ID of the Robot
	 * @param tasks_ID -> The ID of the Task
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @return The cost associated to the path length for the couple of robot and task given as input
	 */
	private static double[][] evaluatePAllApproximation(ReedsSheppCarPlanner rsp, ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec){
		//Take the state for the i-th Robot
		double pathLength = 10000000;
		double biggerPathLength = 0;
		double x1 = 0;
		double y1 = 0;
		double x2 = 0;
		double y2 = 0;
		double pathToTaskStart = 0;
		double [][] PAll = new double[numRobotAug][numTaskAug];
		for (int robot= 0; robot < numRobotAug; robot++) {
			for (int task = 0; task < numTaskAug; task++ ) {
				RobotReport rr = tec.getRobotReport(robot);
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robot + ".");
					throw new Error("RobotReport not found for Robot" + robot + ".");
				}
				//Evaluate the path length for the actual couple of task and ID
				//Set the starting and the arrival point
				
				if(task < numTask && robot < numRobot) {
					// N = M
					//Evaluate the path length to reach the Target Start
					x1 = tec.getRobotReport(robot+1).getPose().getX();
					y1 = tec.getRobotReport(robot+1).getPose().getY();
					x2 = tasks.get(task).getStartPose().getX();
					y2 = tasks.get(task).getStartPose().getY();
					pathToTaskStart = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
					//Evaluate the path length from Target Start to Target End
					x1 = tasks.get(task).getStartPose().getX();
					y1 = tasks.get(task).getStartPose().getY();
					x2 = tasks.get(task).getGoalPose().getX();
					y2 = tasks.get(task).getGoalPose().getY();
					//Evaluate the Euclidean distance
					pathLength = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2)) + pathToTaskStart;
					PAll[robot][task] = pathLength;
			
				}else {// N != M
					if(numRobot > numTask){
						//dummy task
						pathLength = 1;
						PAll[robot][task] =  pathLength;
					}else {
						//dummy robot
						pathLength = 1;
						PAll[robot][task] =  pathLength;
					}
				}
			}
		}
		//Return the cost of path length
		return PAll;
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
    		if(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
    			numberFeasibleSolution = numberFeasibleSolution+1;
    		}
			//Transform the Assignment Vector to Matrix
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
	 * @return A constrained optimization problem
	 */
	private MPSolver buildOptimizationProblem(int numRobot,int numTasks) {
		

		//Initialize a linear solver 
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		
		//START DECISION VARIABLE VARIABLE
		MPVariable [][] Decision_Variable = new MPVariable[numRobot][numTasks]  ;
		for (int i = 0; i < numRobot; i++) {
			 for (int j = 0; j < numTasks; j++) {
				 Decision_Variable[i][j] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+"]");
			 }
		}
		//END DECISION VARIABLE
		//////////////////////////
		// START CONSTRAINTS
		//Each Robot can be assign only to a Task	    
		 for (int i = 0; i < numRobot; i++) {
			 //Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(-infinity, 1);
			 for (int j = 0; j < numTasks; j++) {
				 //Build the constraint
				 c0.setCoefficient(Decision_Variable[i][j], 1); 
			 }
		 }
		//Each task can be performed only by a robot
		 for (int j = 0; j < numRobot; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numTasks; i++) {
				//Build the constraint
				c0.setCoefficient(Decision_Variable[i][j], 1); 		
			 }
		 }
	
		//END CONSTRAINTS
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}
	/**
	 *  * Builds the optimization_problem complete with Objective Function. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m)
	 * with n = number of robot and m = number of tasks.
	 * Only the B function is considered in this case
	 * @param numRobot -> Number of Robots
	 * @param rsp -> A motion planner
	 * @param tasks -> Vector of Tasks
	 * @param approximation -> a boolean variable that allow to choose if use an approximation (where cost are
	 * computed with Euclidean distance) of B or not. Set to true to use approximation.
	 * @param tec -> An AbstractTrajectoryEnvelopeCoordinator Coordinator
	 * @return An optimization problem 
	 */
	public MPSolver buildOptimizationProblemWithB(int numRobotInitial,ReedsSheppCarPlanner rsp,ArrayList<Task> tasks,boolean approximation,AbstractTrajectoryEnvelopeCoordinator tec) {
		//Save the initial number of Robot and Task
		numTask = tasks.size();
		
		//Get free robots
		numRobot = tec.getIdleRobots(numRobotInitial).length;
		
		//Evaluate dummy robot and dummy task
		dummyRobotorTask(numRobot,numTask);
		//Consider possibility to have dummy Robot or Tasks
		//Build the solver and an objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem); 
		double sumPathLength = 0;
		int taskType = 0;
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
	    approximationFlag = approximation;
	    if (approximation == false) {
	    	 for (int i = 0; i < numRobotAug; i++) {
	    		 int robotType = tec.getRobotType(i+1);
	    		 
				 for (int j = 0; j < numTaskAug; j++) {
					 if(j < numTask) {
						 taskType = tasks.get(j).getTaskType();
					 }else {
						 taskType = robotType;
					 }
					 
					 if (robotType == taskType) {
						 objective.setCoefficient(decisionVariable[i][j], evaluatePathLength(i+1,j,rsp,tasks,tec));
						 sumPathLength = sumPathLength + objective.getCoefficient(decisionVariable[i][j]);
					 }
					 
				 }			 
			 }
	    }else {
	    	 for (int i = 0; i < numRobotAug; i++) {
	    		 int robotType = tec.getRobotType(i+1);
				 for (int j = 0; j < numTaskAug; j++) {
					 if(j < numTask) {
						 taskType = tasks.get(j).getTaskType();
					 }else {
						 taskType = robotType;
					 }
					 if (robotType == taskType) {
						 objective.setCoefficient(decisionVariable[i][j], evaluateApproximatePathLength(i+1,j,rsp,tasks,tec));
						 sumPathLength = sumPathLength + objective.getCoefficient(decisionVariable[i][j]);
					 } 
				 }			 
			 }	
	    }
	    totalPathsLength = sumPathLength;
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}
	
	/**
	 *  * Builds the optimization_problem complete with Objective Function. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m)
	 * with n = number of robot and m = number of tasks.
	 * Only the B function is considered in this case
	 * @param numRobot -> Number of Robots
	 * @param rsp -> A motion planner
	 * @param tasks -> Vector of Tasks
	 * @param approximation -> a boolean variable that allow to choose if use an approximation (where cost are
	 * computed with Euclidean distance) of B or not. Set to true to use approximation.
	 * @param tec -> An AbstractTrajectoryEnvelopeCoordinator Coordinator
	 * @return An optimization problem 
	 */
	public MPSolver buildOptimizationProblemWithBNormalized(int numRobotInitial,ReedsSheppCarPlanner rsp,ArrayList<Task> tasks,boolean approximation,AbstractTrajectoryEnvelopeCoordinator tec) {
		//Save the initial number of Robot and Task
		numTask = tasks.size();
		//Get free robots
		numRobot = tec.getIdleRobots(numRobotInitial).length;
		//Evaluate dummy robot and dummy task
		dummyRobotorTask(numRobot,numTask);
		//Consider possibility to have dummy Robot or Tasks
		//Build the solver and an objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTaskAug);
		MPVariable [][] decisionVariable = tranformArray(optimizationProblem); 
		
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
	    approximationFlag = approximation;
	    if (approximation == false) {
	    	double[][] PAll = evaluatePAll(rsp,tasks,tec);
	    	 for (int i = 0; i < numRobotAug; i++) {
	    		 int robotType = tec.getRobotType(i+1);
				 for (int j = 0; j < numTaskAug; j++) {
					 int taskType = tasks.get(j).getTaskType();
					 if (robotType == taskType) {
						 objective.setCoefficient(decisionVariable[i][j], PAll[i][j]/maxPathLength);
					 }
				 }			 
			 }
	    }else {
	    	double[][] PAll = evaluatePAllApproximation(rsp,tasks,tec);
	    	 for (int i = 0; i < numRobotAug; i++) {
	    		 int robotType = tec.getRobotType(i+1);
				 for (int j = 0; j < numTaskAug; j++) {
					 int taskType = tasks.get(j).getTaskType();
					 if (robotType == taskType) {
						 objective.setCoefficient(decisionVariable[i][j], PAll[i][j]/maxPathLength);
					 } 
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
	 * @param rsp -> A motion planner
	 * @param tasks -> Vector of Tasks 
	 * @param optimizationProblem -> An optimization problem defined with {@link #buildOptimizationProblemWithB}
	 * @param tec -> an Abstract Trajectory Envelope Coordinator
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1-alpha)*F
	 * @return An Optimal Assignment that minimize the objective function
	 */
	
	public double [][] solveOptimizationProblem(ReedsSheppCarPlanner rsp,ArrayList<Task> tasks,MPSolver optimizationProblem,AbstractTrajectoryEnvelopeCoordinator tec,double alpha){
		//Initialize the optimal assignment and the cost associated to it
		int numTasks = numTaskAug;
		int numRobot = numRobotAug;
		double [][] optimalAssignmentMatrix = new double[numRobot][numTasks];
		double objectiveOptimalValue = 100000000;
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate an optimal assignment that minimize only the B function
			resultStatus = optimizationProblem.solve();
			//Evaluate the Assignment Matrix
			double [][] AssignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblem);
			//Initialize cost of objective value
			double objectiveFunctionValue = 0;
			double costFFunction = 0;
			double costBFunction = optimizationProblem.objective().value();
			//Evaluate the cost for this Assignment
			for (int i = 0; i < numRobot; i++) {
				for(int j = 0;j < numTasks; j++) {
					if(AssignmentMatrix[i][j]>0) {
						costFFunction = costFFunction + evaluatePathDelay(i+1,j,AssignmentMatrix,rsp,tasks,tec,optimizationProblem);
					}				
				}		
			}
			objectiveFunctionValue = alpha*costBFunction + (1-alpha)*costFFunction;
			//Compare actual solution and optimal solution finds so far
			if (objectiveFunctionValue < objectiveOptimalValue && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				objectiveOptimalValue = objectiveFunctionValue;
				optimalAssignmentMatrix = AssignmentMatrix;
				//Add the constraint on cost for next solution
				optimizationProblem = constraintOnCostSolution(optimizationProblem,objectiveFunctionValue);
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
	 * Assignments are computed at each step in this case. In this case the solver consider all the feasible solutions for the problem.
	 * Exact Algorithm.  
	 * @param rsp -> A motion planner
	 * @param tasks -> Vector of Tasks 
	 * @param tec -> TrajectoryEnvelopeCoordinatorSimulation
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1-alpha)*F 
	 * @return The Optimal Assignment that minimize the objective function
	 */

	public double [][] solveOptimizationProblemExactAlgorithm(ReedsSheppCarPlanner rsp,ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec,double alpha){
		approximationFlag = false;
		int numTasks = numTaskAug;
		int numRobot = numRobotAug;
		//Initialize the optimal assignment and the cost associated to it
		double [][] optimalAssignmentMatrix = new double[numRobot][numTasks];
		double objectiveOptimalValue = 100000000;
		//Initialize optimization problem
		MPSolver optimizationProblem = buildOptimizationProblem(numRobot,tasks.size());
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
		
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate a feasible assignment
			resultStatus = optimizationProblem.solve();
			//Evaluate the Assignment Matrix
			double [][] AssignmentMatrix = saveAssignmentMatrix(numRobot,numTasks,optimizationProblem);
			//Initialize cost of objective value
			double objectiveFunctionValue = 0;
			double costBFunction = 0;
			double costFFunction = 0;
			//Evaluate the cost for this Assignment
			for (int i = 0; i < numRobot ; i++) {
				for(int j=0;j < numTasks; j++) {
					if(AssignmentMatrix[i][j]>0) {
							costBFunction = costBFunction + evaluatePathLength(i+1,j,rsp,tasks,tec);
							costFFunction = costFFunction + evaluatePathDelay(i+1,j,AssignmentMatrix,rsp,tasks,tec,optimizationProblem);
						
					}
				}

			}
			objectiveFunctionValue = alpha * costBFunction + (1- alpha)*costFFunction;
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
	 * Perform the task Assignment defining the mission for each robot
	 * @param AssignmentMatrix -> An Assignment Matrix of the optimization problem
	 * @param rsp -> A motion planner
	 * @param tasks -> Vector of Tasks 
	 * @param tec -> An Abstract Trajectory Envelope Coordinator
	 * @return An updated Trajectory Envelope Coordinator Simulation in which the mission for each
	 * robot is defined
	 */
	public ArrayList<Task> Task_Assignment(double [][] AssignmentMatrix,ReedsSheppCarPlanner rsp,ArrayList<Task> tasks,AbstractTrajectoryEnvelopeCoordinator tec){
		int robotIDs = AssignmentMatrix.length;
		int numTasks = AssignmentMatrix[0].length;
		PoseSteering[] pathToTaskStart = new PoseSteering[1];
		PoseSteering[] pathFinal = new PoseSteering[1];
		
				
		for (int i = 0; i < robotIDs; i++) {
			 for (int j = 0; j < numTasks; j++) {
				
				 if (AssignmentMatrix[i][j]>0) {
					 if (j < numTask && i < numRobot) {
						 //Evaluate the path length from Robot Starting Position to Task End Position
						 //rsp.setStart(tec.getRobotReport(i+1).getPose());
						 //rsp.setGoals(Tasks.get(j).getStartPose(),Tasks.get(j).getGoalPose());
					     //if (!rsp.plan()) {
					    	// throw new Error ("No path between Robot " + Tasks.get(j).getStartPose() + " and task" + Tasks.get(j).getStartPose());
					     //}
						 //PoseSteering[] pss = rsp.getPath();
						 PoseSteering[] pss = pathsToTargetGoal.get(i*numTask+j);
						 System.out.print("Path between Robot " + i + " and task" + j);
						 tec.addMissions(new Mission(i+1,pss));
						 tasks.get(j).setTaskIsAssigned(true);
						 
					 }
				 } else {
					 System.out.println("Nope");	
				 }
			 }
		 }
		//Remove Assigned Tasks from the set
		if(numRobot >= numTask){
			for (int i = 0; i < numRobot; i++) {
				if(i < tasks.size()) {
					tasks.remove(0);
				}
			}
		}
			else {// NumTask > NumRobot 

		}
		return tasks;
	}//End Task Assignment Function

	}//End Class

