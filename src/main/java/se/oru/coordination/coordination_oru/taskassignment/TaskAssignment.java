package se.oru.coordination.coordination_oru.taskassignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
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
	protected static MPVariable [][] decision_variables;
	protected static double Objective_Function_Value;
	protected static double infinity = java.lang.Double.POSITIVE_INFINITY;
	protected static double Sum_Path_Length;
	protected static boolean Approximation_Flag;
	protected static int Dummy_Robot;
	protected static int Dummy_Task;
	protected static int numRobotAug;
	protected static int numTaskAug;
	protected static Task [] TasksMissions;
	protected static Integer[] IdleRobots;
	//FleetMaster Interface Parameters
	
	
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
	protected void addPathProva(int robotID, int pathID, PoseSteering[] pss, Geometry boundingBox, Coordinate... coordinates) {
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
			return fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays);
		}
		return new Pair<Double, Double> (Double.NaN, Double.NaN);
	}
	
	/**
	 * Considering the possibility to have a different number of robots (N) and tasks (M). If N > M, dummy tasks are 
	 * considered, where a dummy task is a task for which a robot stay in starting position; while if M > N dummy robots
	 * are considered, where a dummy robot is only a virtual robot
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks
	 * @return DummyVector -> A vector that contains the number of dummy robots (First Position) and tasks(Second Position)
	 */

	private static int[]  DummyRobotorTask(int num_robot, int num_tasks) {
		int [] DummyVector = {num_robot,num_tasks};
		numRobotAug = num_robot;
		numTaskAug = num_tasks;
		//Considering the possibility to have n != m
		//If n > m -> we have dummy robot, so at some robot is assign the task to stay in starting position
		if(num_robot > num_tasks) {
			Dummy_Task = num_robot - num_tasks;
			numTaskAug = num_tasks + Dummy_Task;
			DummyVector[1] = Dummy_Task;
		}
		else if(num_robot < num_tasks) {
			Dummy_Robot = num_tasks - num_robot;
			numRobotAug = num_robot + Dummy_Robot;
			DummyVector[0] = Dummy_Robot;
		}
		return DummyVector;
	}

	
	/**
	 * Transform a 1D array of MPVariable into a 2D MATRIX  
	 * @param num_robot -> Number of robots
	 * @param num_tasks -> Number of tasks
	 * @param solver -> An optimization problem defined with {@link #optimization_problem}
	 * @return 2D Matrix of Decision_Variable
	 */
	private static MPVariable [][] tranform_array(MPSolver solver) {
		int num_robot = numRobotAug;
		int num_tasks = numTaskAug;
		//Take the vector of Decision Variable from the input solver
		MPVariable [] array1D = solver.variables();
		MPVariable [][] Decision_Variable = new MPVariable [num_robot][num_tasks];
		//Store them in a 2D Matrix
	    for (int i = 0; i < num_robot; i++) {
			 for (int j = 0; j < num_tasks; j++) {
				 Decision_Variable[i][j] = array1D[i*num_tasks+j];
			 }
	    }
		return Decision_Variable;
	}
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param solver -> An optimization problem  defined with {@link #optimization_problem} in which a solution is found
	 * @param Assignment_Matrix -> The Assignment Matrix of the actual optimal solution
	 * @return Solver updated with the new constraint on previous optimal solution found  
	 */
	private static MPSolver constraint_on_previous_solution(MPSolver solver, double [][] Assignment_Matrix) {
		//Take the vector of Decision Variable from the input solver
		int num_robot = Assignment_Matrix.length;
		int num_tasks = Assignment_Matrix[0].length;
		MPVariable [][] Decision_Variable = tranform_array(solver);
		//Initialize a Constraint
		MPConstraint c2 = solver.makeConstraint(-infinity,1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < num_robot; i++) {
    		for (int j = 0; j < num_tasks; j++) {
    				if(Assignment_Matrix[i][j] >0) {
	    				c2.setCoefficient(Decision_Variable[i][j],1);
	    			}else {
	    				c2.setCoefficient(Decision_Variable[i][j],0);
	    			}
    			}		
		 	}
    	//Return the updated solver
    	return solver;
	}
	
	
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param solver -> An optimization problem  defined with {@link #optimization_problem} in which a solution is found
	 * @param Assignment_Matrix -> The Assignment Matrix of the actual optimal solution
	 * @return Solver updated with the new constraint on previous optimal solution found  
	 */
	private static MPSolver constraint_on_cost_solution(MPSolver solver,double objective_value) {
		//Take the vector of Decision Variable from the input solver
		int num_robot = numRobotAug;
		int num_tasks = numTaskAug;
		MPVariable [][] Decision_Variable = tranform_array(solver);
		//Initialize a Constraint
		MPConstraint c3 = solver.makeConstraint(-infinity,objective_value);
		//Define a constraint for which the new optimal solution considering only B must have a cost less than the min of 
		//previous assignment considering the sum of B and F
    	for (int i = 0; i < num_robot; i++) {
    		for (int j = 0; j < num_tasks; j++) {
    			c3.setCoefficient(Decision_Variable[i][j],1);
    			}		
		 }
    	//Return the updated solver
    	return solver;
	}
	
	
	/**
	 * Compute the Assignment matrix given a solved optimization problem
	 * @param num_robot -> Number of robots
	 * @param num_tasks -> Number of tasks
	 * @param Solver -> A solved optimization problem
	 * @return Assignment matrix for the optimization problem given as input
	 */
	
	private static double [][] Assignment_Matrix(int num_robot,int num_tasks,MPSolver solver){
		MPVariable [][] Decision_Variable = tranform_array(solver);
		double [][] Assignment_Matrix = new double [num_robot][num_tasks];	
		//Store the Assignment Matrix
		for (int i = 0; i < num_robot; i++) {
			for (int j = 0; j < num_tasks; j++) {
				Assignment_Matrix[i][j] = Decision_Variable[i][j].solutionValue();
			}
		}
		return Assignment_Matrix;	
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
	private double evaluate_B_function(int robot_ID , int tasks_ID, ReedsSheppCarPlanner rsp, ArrayList<Task> Tasks,TrajectoryEnvelopeCoordinatorSimulation tec){
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double Path_length = 0;
		
		double pathToTaskStart = 0;
		//Evaluate the path length from Target Start to Target End
		// N = M
		if(tasks_ID < numTask && robot_ID < numRobot) {
			//Evaluate the path length to reach the Target Start
			rsp.setStart(tec.getRobotReport(robot_ID+1).getPose());
			
			rsp.setGoals(Tasks.get(tasks_ID).getStartPose());
			rsp.plan();
			pathToTaskStart = Missions.getPathLength(rsp.getPath());
			rsp.setStart(Tasks.get(tasks_ID).getStartPose());
			rsp.setGoals(Tasks.get(tasks_ID).getGoalPose());
			
		}
		
		else { // N != M
			if(numRobot > numTask){ //dummy task
				Path_length = 1;
				return Path_length;
			}else { //dummy robot
				
				Path_length = 1;
				return Path_length;
			}
			
		}
		rsp.setStart(Tasks.get(tasks_ID).getStartPose());
		rsp.setGoals(Tasks.get(tasks_ID).getGoalPose());
		//Evaluate the path length
		if (!rsp.plan()) {
			Path_length = 100000000;
		}
		else {
			PoseSteering[] pss = rsp.getPath();
			System.out.print("Start add path");
			//addPathProva(robot_ID+1, pss.hashCode(), pss, null, tec.getFootprint(robot_ID+1));
			System.out.print("Done add path");
			Path_length = Missions.getPathLength(pss) + pathToTaskStart;		
		}
		
		//Return the cost of path length
		return Path_length;
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
	 * @param solver -> an optimization problem defined with {@link #optimization_problem_complete}
	 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
	 */
	
	private double evaluate_F_function(int robot_ID ,int tasks_ID,double [][] Assignment_Matrix,ReedsSheppCarPlanner rsp,ArrayList<Task> Tasks,TrajectoryEnvelopeCoordinatorSimulation tec,MPSolver solver){
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		double delay = 0;
		PoseSteering[] pathToTaskStart = new PoseSteering[1];
		PoseSteering[] pathToTaskStart2 = new PoseSteering[1];
		if(Assignment_Matrix[robot_ID][tasks_ID]>0) {
			
			// N = M
			if(tasks_ID < numTask && robot_ID < numRobot) {
				//Evaluate the path for this couple of robot and task
				//Evaluate the path length to reach the Target Start
				rsp.setStart(tec.getRobotReport(robot_ID+1).getPose());
				rsp.setGoals(Tasks.get(tasks_ID).getStartPose());
				rsp.plan();
				pathToTaskStart = rsp.getPath();
				//Evaluate the path length from Target Start to Target End
				rsp.setStart(Tasks.get(tasks_ID).getStartPose());
				rsp.setGoals(Tasks.get(tasks_ID).getGoalPose());
				
				
				
			} else { // N != M
				if(numRobot > numTask){ //dummy task
					return delay;			
				}else { //dummy robot
					
					return delay;
				}
				
			}
			
			//Evaluate the path between the robot i-th and the path j-th
			rsp.plan();
			PoseSteering[] pss1 = rsp.getPath();
			PoseSteering[] totalPath = (PoseSteering[]) ArrayUtils.addAll(pathToTaskStart, pss1);
			
			TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
			TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
			//Compute the spatial Envelope
			
			SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,tec.getFootprint(robot_ID+1));
			
			if(Approximation_Flag) {
				double Path_length = Missions.getPathLength(pss1);
				solver.objective().setCoefficient(solver.variables()[robot_ID*Assignment_Matrix[0].length+tasks_ID], Path_length);
			}
			for(int m=0; m < Assignment_Matrix.length; m++) {
				for(int n=0; n < Assignment_Matrix[0].length; n++) {
					if (Assignment_Matrix[m][n]>0 && m!=robot_ID && n!=tasks_ID && n < numTask && m < numRobot) {
						//Evaluate the path length to reach the Target Start
						rsp.setStart(tec.getRobotReport(m+1).getPose());
						rsp.setGoals(Tasks.get(n).getStartPose());
						rsp.plan();
						pathToTaskStart2 = rsp.getPath();
						//Evaluate the path length from Target Start to Target End
						rsp.setStart(Tasks.get(n).getStartPose());
						rsp.setGoals(Tasks.get(n).getGoalPose());
						rsp.plan();
						PoseSteering[] pss2 = rsp.getPath();
						//flint.addPath(m+1, 1, rsp.getPath(), tec.getFootprintPolygon(m+1), tec.getFootprint(m+1));
						SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,tec.getFootprint(m+1));
						CriticalSection [] css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, 2.0);
						
						for (int g = 0; g < css.length; g++) {
							//Pair<Double, Double> a1 =estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pss2.hashCode(),pss2,te2TCDelays, css[g]);
							//System.out.println("Prova Delay" + a1.getFirst());
						}
						
						break;
						
					}
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
	private static double evaluate_B_function_approx(int robot_ID ,int tasks_ID, ReedsSheppCarPlanner rsp, ArrayList<Task> Tasks,TrajectoryEnvelopeCoordinatorSimulation tec){
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double x1 = 0;
		double y1 = 0;
		double x2 = 0;
		double y2 = 0;
		double Path_length = 0;
		double pathToTaskStart = 0;
		if(tasks_ID < numTask && robot_ID < numRobot) {
			// N = M
			//Evaluate the path length to reach the Target Start
			x1 = tec.getRobotReport(robot_ID+1).getPose().getX();
			y1 = tec.getRobotReport(robot_ID+1).getPose().getY();
			x2 = Tasks.get(tasks_ID).getStartPose().getX();
			y2 = Tasks.get(tasks_ID).getStartPose().getY();
			pathToTaskStart = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
			//Evaluate the path length from Target Start to Target End
			x1 = Tasks.get(tasks_ID).getStartPose().getX();
			y1 = Tasks.get(tasks_ID).getStartPose().getY();
			x2 = Tasks.get(tasks_ID).getGoalPose().getX();
			y2 = Tasks.get(tasks_ID).getGoalPose().getY();
			
		}
		
		else {// N != M
			if(numRobot > numTask){
				//dummy task
				Path_length = 1;
				return Path_length;
			}else {
				//dummy robot
				Path_length = 1;
				return Path_length;
			}
		}
		

		//Evaluate the Euclidean distance
		Path_length = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
		//Return the cost of path length
		return Path_length;
		}

	/**
	 * Evaluates the number of all feasible solutions for an optimization problem, with is defined with {@link #optimization_problem}
	 * @param num_robot -> Number of Robot of the optimization problem 
	 * @param num_tasks -> Number of Tasks of the optimization problem
	 * @return The number of all feasible solutions for the Optimization Problem
	 */
 
	public int number_of_Feasible_Solution(int num_robot,int num_tasks){
		//Create an optimization problem
		MPSolver solver_copy = optimization_problem(num_robot,num_tasks);
		//Solve the optimization problem
	    MPSolver.ResultStatus resultStatus = solver_copy.solve();
	    int number_feasible_solution = 0;
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = solver_copy.solve();
    		if(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
    			number_feasible_solution = number_feasible_solution+1;
    		}
			//Transform the Assignment Vector to Matrix
    		double [][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver_copy);
			//Add the constraint to actual solution -> in order to consider this solution as already found  
    		solver_copy = constraint_on_previous_solution(solver_copy,Assignment_Matrix);
	    		
	    }
		//Return the Total number of feasible solution
	    solver_copy.clear();
	    return number_feasible_solution;
	}
	
	
	
	/**
	 * Evaluates all possible feasible solutions for an optimization problem,with is defined with {@link #optimization_problem}. A feasible solution is a solution that verify constraints
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks
	 * @return A set containing all feasible solutions
	 */
	public double [][][] feasible_solution(int num_robot,int num_tasks){
		//Define the optimization problem
		MPSolver solver_copy = optimization_problem(num_robot,num_tasks);
		//Evaluate the number of all feasible solution for the optimization problem
	    int feasible_solutions = number_of_Feasible_Solution(num_robot,num_tasks);
	    //Initialize a set to store all feasible solution
		double [][][] Assignment_Matrix_OptimalSolutions = new double [feasible_solutions][num_robot][num_tasks]; 
	    ///////////////////////////////////////
	    for(int k=0; k < feasible_solutions; k++) {
			//Solve the optimization problem
	    	MPSolver.ResultStatus resultStatus = solver_copy.solve();
			//Transform the Assignment Vector to Matrix
			double [][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver_copy);
			//Store the optimal solution
			Assignment_Matrix_OptimalSolutions[k]=Assignment_Matrix;
			//Add the constraint to actual solution in order to consider this solution as already found  
			solver_copy = constraint_on_previous_solution(solver_copy,Assignment_Matrix);
			}
		//Return the set of all Feasible solutions
	    return Assignment_Matrix_OptimalSolutions;
		}

	/**
	 * Builds the optimization problem. Define a decision variable X_ij as a binary variable in which i indicate
	 * the robot id, j the tasks. Also constraints are defined:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks.
	 * @return A constrained optimization problem
	 */
	private MPSolver optimization_problem(int num_robot,int num_tasks) {
		
		
		
		instantiateFleetMaster(0.1, false);
		//Initialize a linear solver 
		MPSolver solver = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		
		//START DECISION VARIABLE VARIABLE
		MPVariable [][] Decision_Variable = new MPVariable[num_robot][num_tasks]  ;
		for (int i = 0; i < num_robot; i++) {
			 for (int j = 0; j < num_tasks; j++) {
				 Decision_Variable[i][j] = solver.makeBoolVar("x"+"["+i+","+j+"]");
			 }
		}
		//END DECISION VARIABLE
		//////////////////////////
		// START CONSTRAINTS
		//Each Robot can be assign only to a Task	    
		 for (int i = 0; i < num_robot; i++) {
			 //Initialize the constraint
			 MPConstraint c0 = solver.makeConstraint(-infinity, 1);
			 for (int j = 0; j < num_tasks; j++) {
				 //Build the constraint
				 c0.setCoefficient(Decision_Variable[i][j], 1); 
			 }
		 }
		//Each task can be performed only by a robot
		 for (int j = 0; j < num_robot; j++) {
			//Initialize the constraint
			 MPConstraint c0 = solver.makeConstraint(1, 1); 
			 for (int i = 0; i < num_tasks; i++) {
				//Build the constraint
				c0.setCoefficient(Decision_Variable[i][j], 1); 		
			 }
		 }
	
		//END CONSTRAINTS
		/////////////////////////////////////////////////
		return solver;	
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
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks.
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @param Approximation -> a boolean variable that allow to choose if use an approximation (where cost are
	 * computed with Euclidean distance) of B or not. Set to true to use approximation.
	 * @return An optimization problem 
	 */
	public MPSolver optimization_problem_complete(int num_robot,ReedsSheppCarPlanner rsp,ArrayList<Task> Tasks,boolean Approximation,TrajectoryEnvelopeCoordinatorSimulation tec) {
		//Save the initial number of Robot and Task
		numTask = Tasks.size();
		//Get free robots
		numRobot = tec.getIdleRobots(num_robot).length;
		//Evaluate dummy robot and dummy task
		DummyRobotorTask(numRobot,numTask);
		//Consider possibility to have dummy Robot or Tasks
		//Build the solver and an objective function
		MPSolver solver = optimization_problem(numRobotAug,numTaskAug);
		MPVariable [][] Decision_Variable = tranform_array(solver); 
		double sum_path_length = 0;
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = solver.objective();
	    Approximation_Flag = Approximation;
	    if (Approximation == false) {
	    	 for (int i = 0; i < numRobotAug; i++) {
	    		 int robotType = tec.getRobotType(i+1);
				 for (int j = 0; j < numTaskAug; j++) {
					 int taskType = Tasks.get(j).getTaskType();
					 if (robotType == taskType) {
						 objective.setCoefficient(Decision_Variable[i][j], evaluate_B_function(i,j,rsp,Tasks,tec));
						 sum_path_length = sum_path_length + objective.getCoefficient(Decision_Variable[i][j]);
					 }
					 
				 }			 
			 }
	    }else {
	    	 for (int i = 0; i < numRobotAug; i++) {
	    		 int robotType = tec.getRobotType(i+1);
				 for (int j = 0; j < numTaskAug; j++) {
					 int taskType = Tasks.get(j).getTaskType();
					 if (robotType == taskType) {
						 objective.setCoefficient(Decision_Variable[i][j], evaluate_B_function_approx(i,j,rsp,Tasks,tec));
						 sum_path_length = sum_path_length + objective.getCoefficient(Decision_Variable[i][j]);
					 } 
				 }			 
			 }	
	    }
	   
	     Sum_Path_Length = sum_path_length;
		 //Define the problem as a minimization problem
		 objective.setMinimization();
		 //END OBJECTIVE FUNCTION
		 return solver;	
	}
	
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks. The solver first finds the optimal solution considering only B function and then
	 * for this each solution (that is an assignment) evaluates the cost of F function. Then a new optimal solution considering only B is 
	 * computed and it is consider only if the cost of this new assignment considering only B is less than the min cost of previous assignments
	 * considering both F and B function
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks.
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @param solver -> An optimization problem defined with {@link #optimization_problem_complete}
	 * @param tec -> TrajectoryEnvelopeCoordinatorSimulation
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1-alpha)*F
	 * @return An Optimal Assignment that minimize the objective function
	 */
	
	public double [][] solve_optimization_problem(ReedsSheppCarPlanner rsp,ArrayList<Task> Tasks,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		//Initialize the optimal assignment and the cost associated to it
		
		int num_tasks = numTaskAug;
		int num_robot = numRobotAug;
		double [][] Assignment_Matrix_ott = new double[num_robot][num_tasks];
		double obj_value_ott = 100000000;
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = solver.solve();
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate an optimal assignment that minimize only the B function
			resultStatus = solver.solve();
			//Evaluate the Assignment Matrix
			double [][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver);
			//Initialize cost of objective value
			double obj_value = 0;
			double cost_F = 0;
			double cost_B = solver.objective().value();
			//Evaluate the cost for this Assignment
			for (int i = 0; i < num_tasks; i++) {
				for(int j=0;j < num_robot; j++) {
					if(Assignment_Matrix[i][j]>0) {
						cost_F = cost_F + evaluate_F_function(i,j,Assignment_Matrix,rsp,Tasks,tec,solver);
					}				
				}		
			}
			obj_value = alpha*cost_B + (1-alpha)*cost_F;
			//Compare actual solution and optimal solution finds so far
			if (obj_value < obj_value_ott && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				obj_value_ott = obj_value;
				Assignment_Matrix_ott = Assignment_Matrix;
				//Add the constraint on cost for next solution
				solver = constraint_on_cost_solution(solver,obj_value);
			}
			//Add the constraint to actual solution in order to consider this solution as already found  
			solver = constraint_on_previous_solution(solver,Assignment_Matrix);
		}
		//Return the Assignment Matrix 
		return  Assignment_Matrix_ott;    
	}
	
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks
	 * Assignments are computed at each step in this case. In this case the solver consider all the feasible solutions for the problem.
	 * Exact Algorithm.  
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks.
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @param solver -> An optimization problem defined with {@link #optimization_problem}
	 * @param tec -> TrajectoryEnvelopeCoordinatorSimulation
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1-alpha)*F 
	 * @return The Optimal Assignment that minimize the objective function
	 */

	public double [][] solve_optimization_problem_exact(ReedsSheppCarPlanner rsp,ArrayList<Task> Tasks,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		Approximation_Flag = false;
		int num_tasks = numTaskAug;
		int num_robot = numRobotAug;
		//Initialize the optimal assignment and the cost associated to it
		double [][] Assignment_Matrix_ott = new double[num_robot][num_tasks];
		double obj_value_ott = 100000000;
		//Initialize optimization problem
		MPSolver solver = optimization_problem(num_robot,Tasks.size());
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = solver.solve();
		
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate a feasible assignment
			resultStatus = solver.solve();
			//Evaluate the Assignment Matrix
			double [][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver);
			//Initialize cost of objective value
			double obj_value = 0;
			double cost_B = 0;
			double cost_F = 0;
			//Evaluate the cost for this Assignment
			for (int i = 0; i < num_robot ; i++) {
				for(int j=0;j < num_tasks; j++) {
					if(Assignment_Matrix[i][j]>0) {
						cost_B = cost_B + evaluate_B_function(i,j,rsp,Tasks,tec);
						cost_F = cost_F + evaluate_F_function(i,j,Assignment_Matrix,rsp,Tasks,tec,solver);
						
					}
				}

			}
			
			obj_value = alpha * cost_B + (1- alpha)*cost_F;
			//Compare actual solution and optimal solution finds so far
			if (obj_value < obj_value_ott && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				obj_value_ott = obj_value;
				Assignment_Matrix_ott = Assignment_Matrix;
			}
			//Add the constraint to actual solution in order to consider this solution as already found  
			solver = constraint_on_previous_solution(solver,Assignment_Matrix);

		}
		//Return the Optimal Assignment Matrix
		return  Assignment_Matrix_ott;    
	}
	
	
	/**
	 * Perform the task Assignment defining the mission for each robot
	 * @param Assignment_Matrix -> An Assignment Matrix of the optimization problem
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @param tec -> A Trajectory Envelope Coordinator Simulation
	 * @return An updated Trajectory Envelope Coordinator Simulation in which the mission for each
	 * robot is defined
	 */
	public static ArrayList<Task> Task_Assignment(double [][] Assignment_Matrix,ReedsSheppCarPlanner rsp,ArrayList<Task> Tasks,TrajectoryEnvelopeCoordinatorSimulation tec){
		int robot_IDs = Assignment_Matrix.length;
		int num_tasks = Assignment_Matrix[0].length;
		PoseSteering[] pathToTaskStart = new PoseSteering[1];
		PoseSteering[] pathFinal = new PoseSteering[1];
		
				
		for (int i = 0; i < robot_IDs; i++) {
			 for (int j = 0; j < num_tasks; j++) {
				 if(Assignment_Matrix[i][j]>0) {
					 if(j < numTask && i < numRobot) {
						 //Evaluate the path length to reach the Target Start
						 rsp.setStart(tec.getRobotReport(i+1).getPose());
						 
						 rsp.setGoals(Tasks.get(j).getStartPose());
						 rsp.plan();
						 pathToTaskStart = rsp.getPath();
					     rsp.setStart(Tasks.get(j).getStartPose());
					     rsp.setGoals(Tasks.get(j).getGoalPose());
					     if (!rsp.plan()) throw new Error ("No path between Robot " + Tasks.get(j).getStartPose() + " and task" + Tasks.get(j).getStartPose());
						 PoseSteering[] ff = rsp.getPath();
						 pathFinal = (PoseSteering[]) ArrayUtils.addAll(pathToTaskStart, ff);
						 tec.addMissions(new Mission(i+1,pathFinal));
					 }else {
						 if(j > numTask){
							//rsp.setStart(tec.getRobotReport(i+1).getPose());
							//rsp.setGoals(tec.getRobotReport(i+1).getPose());
						 }else {
							 //rsp.setStart(Tasks.get(j).getStartPose());
							 //rsp.setGoals(Tasks.get(j).getStartPose());
						 }
						 	
					 }
				 } else {
					 System.out.println("Nope");
					
				 }
			 }
		 }
		//Remove Assigned Tasks from the set
		for (int i = 0; i < numRobot; i++) {
			
			if(i < Tasks.size()) {
				Tasks.remove(0);
				
			}
		}
		return Tasks;
	}//End Task Assignment Function
	}//End Class

