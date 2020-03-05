package se.oru.coordination.coordination_oru.taskassignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.sat4j.sat.SolverController;
import org.sat4j.sat.visu.SolverVisualisation;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import aima.core.agent.Model;
import aima.core.util.datastructure.Pair;
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

public class TaskAssignment_MorePath {
	public static int numRobot;
	public static int numTask;
	public static MPVariable [][] decision_variables;
	public static double Objective_Function_Value;
	static int max_num_paths=3;
	static double infinity = java.lang.Double.POSITIVE_INFINITY;
	public static double Sum_Path_Length;
	public static boolean Approximation_Flag;
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
	
	
	/**
	 * Transform a 1D array of MPVariable into a 3D MATRIX  
	 * @param num_robot -> Number of robots
	 * @param num_tasks -> Number of tasks
	 * @param solver -> An optimization problem defined with {@link #optimization_problem}
	 * @return 3D Matrix of Decision_Variable
	 */
	private static MPVariable [][][] tranform_array(int num_robot,int num_tasks,MPSolver solver) {
		int num_paths = max_num_paths;
		//Take the vector of Decision Variable from the input solver
		MPVariable [] array1D = solver.variables();
		MPVariable [][][] Decision_Variable = new MPVariable [num_robot][num_tasks][num_paths];	
		//Store them in a 2D Matrix
		for (int i = 0; i < num_robot; i++) {
			for (int j = 0; j < num_tasks; j++) {
				for (int s = 0; s < num_tasks; s++) {
					Decision_Variable[i][j][s] = array1D[i*num_tasks*num_paths+j*num_tasks+s];
				}
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
	private static MPSolver constraint_on_previous_solution(MPSolver solver, double [][][] Assignment_Matrix) {
		//Take the vector of Decision Variable from the input solver
		int num_robot = Assignment_Matrix.length;
		int num_tasks = Assignment_Matrix[0].length;
		int num_paths = max_num_paths;
		MPVariable [][][] Decision_Variable = tranform_array(num_robot,num_tasks,solver);
		//Initialize a Constraint
		MPConstraint c2 = solver.makeConstraint(-infinity,1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
		for (int i = 0; i < num_robot; i++) {
    		for (int j = 0; j < num_tasks; j++) {
    			for(int s=0;s < num_paths; s++) {
    				if(Assignment_Matrix[i][j][s]>0) {
	    				c2.setCoefficient(Decision_Variable[i][j][s],1); //for second solver
	    			}else {
	    				c2.setCoefficient(Decision_Variable[i][j][s],0);
	    			}
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
	private static MPSolver constraint_on_cost_solution(MPSolver solver, double [][][] Assignment_Matrix,double objective_value) {
		//Take the vector of Decision Variable from the input solver
		int num_robot = Assignment_Matrix.length;
		int num_tasks = Assignment_Matrix[0].length;
		MPVariable [][][] Decision_Variable = tranform_array(num_robot,num_tasks,solver);
		//Initialize a Constraint
		MPConstraint c3 = solver.makeConstraint(-infinity,objective_value);
		//Define a constraint for which the new optimal solution considering only B must have a cost less than the min of 
		//previous assignment considering the sum of B and F
    	for (int i = 0; i < num_robot; i++) {
    		for (int j = 0; j < num_tasks; j++) {
    			for(int s=0;s < max_num_paths; s++) {
    				c3.setCoefficient(Decision_Variable[i][j][s],1);
	    			
    			}		
    		}		
		 }
    	//Return the updated solver
    	return solver;
	}
	
	
	/**
	 * Evaluate the Assignment matrix given a solved optimization problem
	 * @param num_robot -> Number of robots
	 * @param num_tasks -> Number of tasks
	 * @param Solver -> A solved optimization problem
	 * @return Assignment matrix for the optimization problem given as input
	 */
	
	private static double [][][] Assignment_Matrix(int num_robot,int num_tasks,MPSolver solver){
		int num_paths = max_num_paths;
		MPVariable [][][] Decision_Variable = tranform_array(num_robot,num_tasks,solver);
		double [][][] Assignment_Matrix = new double [num_robot][num_tasks][num_paths];	
		//Store the Assignment Matrix
		for (int i = 0; i < num_robot; i++) {
			for (int j = 0; j < num_tasks; j++) {
				for(int s=0;s < max_num_paths; s++) {
					Assignment_Matrix[i][j][s] = Decision_Variable[i][j][s].solutionValue();
				}		
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
	private static double evaluate_B_function(int robot_ID ,int tasks_ID,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose){
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double Path_length = 0;
		// N = M
		if(tasks_ID < numTask && robot_ID < numRobot) {
			rsp.setStart(startsPose[robot_ID]);
			rsp.setGoals(goalsPose[tasks_ID]);
			
		}else { // N != M
			if(numRobot > numTask){
				//dummy task
				rsp.setStart(startsPose[robot_ID]);
				rsp.setGoals(startsPose[robot_ID]);			
			}else {
				//dummy robot
				rsp.setStart(goalsPose[tasks_ID]);
				rsp.setGoals(goalsPose[tasks_ID]);
			}
			
		}

		//Evaluate the path length
		if (!rsp.plan()) {
			Path_length = 100000000;
		}
		PoseSteering[] pss = rsp.getPath();
		Path_length = Missions.getPathLength(pss);
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
	
	private static double evaluate_F_function(int robot_ID ,int tasks_ID,int robot_path,double [][][] Assignment_Matrix,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,TrajectoryEnvelopeCoordinatorSimulation tec,MPSolver solver){
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		double delay = 0;
		if(Assignment_Matrix[robot_ID][tasks_ID][robot_path]>0) {
			//Evaluate the path for the actual couple of task and ID
			//Evaluate the path for this couple of robot and task
			// N = M
			if(tasks_ID < numTask && robot_ID < numRobot) {
				rsp.setStart(startsPose[robot_ID]);
				rsp.setGoals(goalsPose[tasks_ID]);
				
			}else { // N != M
				if(numRobot > numTask){
					//dummy task
					rsp.setStart(startsPose[robot_ID]);
					rsp.setGoals(startsPose[robot_ID]);			
				}else {
					//dummy robot
					rsp.setStart(goalsPose[tasks_ID]);
					rsp.setGoals(goalsPose[tasks_ID]);
				}
				
			}
			//rsp.setStart(startsPose[robot_ID]);
			//rsp.setGoals(goalsPose[tasks_ID]);
			rsp.plan();
			PoseSteering[] pss1 = rsp.getPath();
			double Path_length = Missions.getPathLength(pss1);
			if(Approximation_Flag) {
				solver.objective().setCoefficient(solver.variables()[robot_ID*Assignment_Matrix[0].length*max_num_paths + tasks_ID*Assignment_Matrix[0].length + robot_path], Path_length);
			}
			for(int m=0; m < Assignment_Matrix.length; m++) {
				for(int n=0; n < Assignment_Matrix[0].length; n++) {
					for(int k = 0; k < max_num_paths; k++) {
						if (Assignment_Matrix[m][n][k]>0 && m!=robot_ID && n!=tasks_ID && k!= robot_path) {
							delay += 10;
					}
					
					}
				}
			}
		}
		
		//return the delay for the i-th robot and the j-th task considering path s due to interference with other robots
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
	private static double evaluate_B_function_approx(int robot_ID ,int tasks_ID,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose){
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double x1 = 0;
		double y1 = 0;
		double x2 = 0;
		double y2 = 0;
		if(tasks_ID < numTask && robot_ID < numRobot) {
			// N = M
			x1 = startsPose[robot_ID].getX();
			y1 = startsPose[robot_ID].getY();
			x2 = goalsPose[tasks_ID].getX();
			y2 = goalsPose[tasks_ID].getY();
			
		}else {// N != M
			if(numRobot > numTask){
				//dummy task
				x1 = startsPose[robot_ID].getX();
				y1 = startsPose[robot_ID].getY();
				x2 = startsPose[robot_ID].getX();
				y2 = startsPose[robot_ID].getY();
			}else {
				//dummy robot
				x1 = goalsPose[tasks_ID].getX();
				y1 = goalsPose[tasks_ID].getY();
				x2 = goalsPose[tasks_ID].getX();
				y2 = goalsPose[tasks_ID].getY();
			}
		}
		//double x1 = startsPose[robot_ID].getX();
		//double y1 = startsPose[robot_ID].getY();
		//double x2 = goalsPose[tasks_ID].getX();
		//double y2 = goalsPose[tasks_ID].getY();
		//Evaluate the Euclidean distance
		double Path_length = Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
		//Return the cost of path length
		return Path_length;
		}
	
	
	/**
	 * Evaluates the number of all feasible solutions for an optimization problem, with is defined with {@link #optimization_problem}
	 * @param num_robot -> Number of Robot of the optimization problem 
	 * @param num_tasks -> Number of Tasks of the optimization problem
	 * @return The number of all feasible solutions for the Optimization Problem
	 */
 
	public static int number_of_Feasible_Solution(int num_robot,int num_tasks){
		//Create an optimization problem
		MPSolver solver_copy = optimization_problem(num_robot,num_tasks);
		//Solve the optimization problem
	    MPSolver.ResultStatus resultStatus = solver_copy.solve();
	    int number_feasible_solution = 0;
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = solver_copy.solve();
    		if(resultStatus !=MPSolver.ResultStatus.INFEASIBLE) {
    			number_feasible_solution = number_feasible_solution+1;
    		}
			//Transform the Assignment Vector to Matrix
    		double [][][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver_copy);
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
	public static double [][][][] feasible_solution(int num_robot,int num_tasks){
		int num_paths = max_num_paths;
		//Define the optimization problem
		MPSolver solver_copy = optimization_problem(num_robot,num_tasks);
		//Evaluate the number of all feasible solution for the optimization problem
	    int feasible_solutions = number_of_Feasible_Solution(num_robot,num_tasks);
	    //Initialize a set to store all feasible solution
		double [][][][] Assignment_Matrix_OptimalSolutions = new double [feasible_solutions][num_robot][num_tasks][num_paths]; 
	    ///////////////////////////////////////
	    for(int k=0; k < feasible_solutions; k++) {
			//Solve the optimization problem
	    	MPSolver.ResultStatus resultStatus = solver_copy.solve();
			//Transform the Assignment Vector to Matrix
			double [][][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver_copy);
			//Store the optimal solution
			Assignment_Matrix_OptimalSolutions[k] = Assignment_Matrix;
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
	private static MPSolver optimization_problem(int num_robot,int num_tasks) {
		int num_paths = max_num_paths;
		//Initialize a linear solver 
		MPSolver solver = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);

		
		//START DECISION VARIABLE VARIABLE
		MPVariable [][][] Decision_Variable = new MPVariable[num_robot][num_tasks][num_paths]  ;
		for (int i = 0; i < num_robot; i++) {
			 for (int j = 0; j < num_tasks; j++) {
				 for(int s=0;s < max_num_paths;s++) {
					 Decision_Variable[i][j][s] = solver.makeBoolVar("x"+"["+i+","+j+","+s+"]");
				 }
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
				 for(int s = 0;s < max_num_paths;s++) {
					 //Build the constraint
					 c0.setCoefficient(Decision_Variable[i][j][s], 1); 
				 }
			 }
		 }
		//Each task can be performed only by a robot
		 for (int j = 0; j < num_robot; j++) {
			//Initialize the constraint
			 MPConstraint c0 = solver.makeConstraint(1, 1); 
			 for (int i = 0; i < num_tasks; i++) {
				 for(int s = 0;s < max_num_paths;s++) {
					 //Build the constraint
					 c0.setCoefficient(Decision_Variable[i][j][s], 1); 
				 }		
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
	public static MPSolver optimization_problem_complete(int num_robot,int num_tasks,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,boolean Approximation) {
		numRobot = num_robot;
		numTask = num_tasks;
		//Build the solver and an objective function
		//Considering the possibility to have n != m
		//If n > m -> we have dummy robot, so at some robot is assign the task to stay in starting position
		if(num_robot > num_tasks) {
			Dummy_Task = num_robot-num_tasks;
			num_tasks = num_robot;		
		}
		
		if(num_robot < num_tasks) {
			Dummy_Robot = num_tasks - num_robot;
			num_robot = num_tasks;		
		}
		MPSolver solver = optimization_problem(num_robot,num_tasks);
		MPVariable [][][] Decision_Variable = tranform_array(num_robot,num_tasks,solver);
		double sum_path_length = 0;
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = solver.objective();
	    if(Approximation == false) {
	    	 for (int i = 0; i < num_robot; i++) {
				 for (int j = 0; j < num_tasks; j++) {
					 for(int s = 0;s < max_num_paths;s++) {
						 objective.setCoefficient(Decision_Variable[i][j][s], evaluate_B_function(i,j,rsp,startsPose,goalsPose));
						 sum_path_length = sum_path_length + objective.getCoefficient(Decision_Variable[i][j][s]);
					 }	 			 
				 }			 
			 }
	    }else {
	    	for (int i = 0; i < num_robot; i++) {
				 for (int j = 0; j < num_tasks; j++) {
					 for(int s = 0;s < max_num_paths;s++) {
						 objective.setCoefficient(Decision_Variable[i][j][s], evaluate_B_function_approx(i,j,rsp,startsPose,goalsPose));
						 sum_path_length = sum_path_length + objective.getCoefficient(Decision_Variable[i][j][s]);
					 }	 			 
				 }			 
			 }
	    }
		Sum_Path_Length = sum_path_length;
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		//objective.setOptimizationDirection(false);
		return solver;	
	}
	
	
	/** 
	 * Solve the optimization problem given as input considering both B and F Functions. The objective function is defined as sum(c_ij * x_ij) for (i = 1...n)(j = 1...m).
	 * with n = number of robot and m = number of tasks
	 * Assignments are precomputed in this case  
	 * @param num_robot -> Number of Robots
	 * @param num_tasks -> Number of Tasks.
	 * @param rsp -> A motion planner
	 * @param startsPose -> Vector of Starting Pose
	 * @param goalsPose -> Vector of goals Pose
	 * @param solver -> An optimization problem defined with {@link #optimization_problem}
	 * @param alpha -> the linear parameter used to weights B and F function expressed in percent( 1-> 100%). The objective function is
	 * considered as B*alpha + (1-alpha)*F 
	 * @return The Optimal Assignment that minimize the objective function
	 */
	public static double [][][] solve_optimization_problem_precomputedAssignment(int num_robot,int num_tasks,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		int num_paths = max_num_paths;
		num_tasks = num_tasks + Dummy_Task;
		num_robot = num_robot + Dummy_Robot;
		//Define the optimization problem
		double [][][] Assignment_Matrix = new double[num_robot][num_tasks][num_paths];
		//Evaluate the number of feasible solutions for the optimization problem
		int pp = number_of_Feasible_Solution(num_robot,num_tasks);
		//Evaluate the set of feasible solutions for the optimization problem
		double [][][][] all_solutions = feasible_solution(num_robot,num_tasks);
		//Initialize the optimal assignment and the cost associated to it
		double [][][] Assignment_Matrix_ott = new double[num_robot][num_tasks][num_paths];
		double obj_value_ott = 100000000;
		//Start to slide all possible solutions 
		for (int k = 0; k < pp; k++) {
			Assignment_Matrix = all_solutions[k];
			double obj_value = 0;
			double cost_B = 0;
			double cost_F = 0;
			for (int i = 0; i < num_robot ; i++) {
				for(int j=0;j < num_tasks; j++) {
					for(int s = 0;s < num_paths;s++) {
						if(Assignment_Matrix[i][j][s] > 0) {
							cost_B = cost_B + evaluate_B_function(i,j,rsp,startsPose,goalsPose);
							cost_F = cost_F + evaluate_F_function(i,j,s,Assignment_Matrix,rsp,startsPose,goalsPose,tec,solver);						
					
						}
					}				
				}		
			}
			obj_value = alpha * cost_B + (1 - alpha)*cost_F;
			//Compare actual solution and optimal solution finds so far
			if (obj_value < obj_value_ott) {
				obj_value_ott = obj_value;
				Assignment_Matrix_ott = Assignment_Matrix;
			}
		}
		//Return the Optimal Assignment Matrix
	    return  Assignment_Matrix_ott;    
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
	
	public static double [][][] solve_optimization_problem(int num_robot,int num_tasks,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		int num_paths = max_num_paths;
		num_tasks = num_tasks + Dummy_Task;
		num_robot = num_robot + Dummy_Robot;
		//Initialize the optimal assignment and the cost associated to it
		double [][][] Assignment_Matrix_ott = new double[num_robot][num_tasks][num_paths];
		double obj_value_ott = 100000000;
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = solver.solve();
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate an optimal assignment that minimize only the B function
			resultStatus = solver.solve();
			//Evaluate the Assignment Matrix
			double [][][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver);
			double obj_value = 0;
			double cost_F = 0;
			double cost_B = solver.objective().value();
			//Evaluate the cost for this Assignment
			//Evaluate the cost for this Assignment
			for (int i = 0; i < num_robot ; i++) {
				for(int j=0;j < num_tasks; j++) {
					for(int s = 0;s < num_paths; s++) {
						if(Assignment_Matrix[i][j][s]>0) {
							cost_F = cost_F + evaluate_F_function(i,j,s,Assignment_Matrix,rsp,startsPose,goalsPose,tec,solver);		
						}
					}				
				}		
			}
			obj_value = alpha * cost_B + (1 - alpha)*cost_F;
			//Compare actual solution and optimal solution finds so far
			if (obj_value < obj_value_ott && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				obj_value_ott = obj_value;
				Assignment_Matrix_ott = Assignment_Matrix;
				//Add the constraint on cost for next solution
				solver = constraint_on_cost_solution(solver,Assignment_Matrix,obj_value);
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

	public static double [][][] solve_optimization_problem_exact(int num_robot,int num_tasks,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		int num_paths = max_num_paths;
		num_tasks = num_tasks + Dummy_Task;
		num_robot = num_robot + Dummy_Robot;
		Approximation_Flag = false;
		//Initialize the optimal assignment and the cost associated to it
		double [][][] Assignment_Matrix_ott = new double[num_robot][num_tasks][num_paths];
		double obj_value_ott = 100000000;
		double [] values_prev_solution = new double [num_robot*num_tasks*num_paths];
		//Solve the optimization problem
		MPSolver.ResultStatus resultStatus = solver.solve();
		while(resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
			//Evaluate a feasible assignment
			resultStatus = solver.solve();
			//Evaluate the Assignment Matrix
			double [][][] Assignment_Matrix = Assignment_Matrix(num_robot,num_tasks,solver);
			//Initialize cost of objective value
			double obj_value = 0;
			double cost_B = 0;
			double cost_F = 0;
			//Evaluate the cost for this Assignment
			for (int i = 0; i < num_robot ; i++) {
				for(int j=0;j < num_tasks; j++) {
					for(int s = 0;s < num_paths; s++) {
						if(Assignment_Matrix[i][j][s]>0) {
							cost_B = cost_B + evaluate_B_function(i,j,rsp,startsPose,goalsPose);
							cost_F = cost_F + evaluate_F_function(i,j,s,Assignment_Matrix,rsp,startsPose,goalsPose,tec,solver);			
						}
					}
									
				}		
			}
			obj_value = alpha * cost_B + (1 - alpha)*cost_F;
			//Compare actual solution and optimal solution finds so far
			if (obj_value < obj_value_ott && resultStatus != MPSolver.ResultStatus.INFEASIBLE) {
				obj_value_ott = obj_value;
				Assignment_Matrix_ott = Assignment_Matrix;
				//Set Hint on solution using the best solution
				for(int m = 0; m < solver.numVariables(); m++) {
					values_prev_solution [m] = solver.variables()[m].solutionValue();
					
				}
				solver.setHint(solver.variables(), values_prev_solution);
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
	public static TrajectoryEnvelopeCoordinatorSimulation Task_Assignment(double [][][] Assignment_Matrix,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,TrajectoryEnvelopeCoordinatorSimulation tec){
		int robot_IDs = Assignment_Matrix.length;
		int num_tasks = Assignment_Matrix[0].length;
		int num_paths = max_num_paths;
		for (int i = 0; i < robot_IDs; i++) {
			for (int j = 0; j < num_tasks; j++) {
				for(int s=0; s < num_paths;s++) {
					if(Assignment_Matrix[i][j][s]>0) {
						if(j < numTask && i < numRobot) {
							 rsp.setStart(startsPose[i]);
							 rsp.setGoals(goalsPose[j]);		
						 }else {
							 if(j > numTask){
								 rsp.setStart(startsPose[i]);
								 rsp.setGoals(startsPose[i]);
							 }else {
								 rsp.setStart(goalsPose[j]);
								 rsp.setGoals(goalsPose[j]);
							 }
							 	
						 }			
						if (!rsp.plan()) throw new Error ("No path between " + startsPose[i] + " and " + goalsPose[j]);
						PoseSteering[] ff = rsp.getPath();
						tec.addMissions(new Mission(i+1,ff));
					} else {
						System.out.println("Nope");
						}
				 }	
			 }
		 }
		return tec;
	}
	
	}

