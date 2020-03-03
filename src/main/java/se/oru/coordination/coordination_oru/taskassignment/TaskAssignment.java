package se.oru.coordination.coordination_oru.taskassignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
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
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
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
	protected static int numRobotAug;
	protected static int numTaskAug;
	protected static MPVariable [][] decision_variables;
	protected static double Objective_Function_Value;
	protected static double infinity = java.lang.Double.POSITIVE_INFINITY;
	protected static double Sum_Path_Length;
	protected static boolean Approximation_Flag;
	protected static int Dummy_Robot;
	protected static int Dummy_Task;
	protected static Task [] TasksMissions;
	
	

	private static Geometry makeFootprint(double x, double y, double theta,Polygon footprint) {
		AffineTransformation at = new AffineTransformation();
		at.rotate(theta);
		at.translate(x,y);	
		Geometry rect = at.transform(footprint);
		return rect;
	}
	
	
	
	private Geometry createEnvelope(PoseSteering[] pss,Polygon footprint) {
		Geometry onePoly = null;
		Geometry prevPoly = null;
		for (int i=0;i <pss.length;i++) {
			Geometry rect = makeFootprint(pss[i].getX(),pss[i].getY(),pss[i].getTheta(),footprint);			
			if (onePoly == null) {
				onePoly = rect;
				prevPoly = rect;
			}
			else {
				Geometry auxPoly = prevPoly.union(rect);
				onePoly = onePoly.union(auxPoly.convexHull());
				prevPoly = rect;
			}
		}
		Geometry Envelope = onePoly.getEnvelope();
		
		return Envelope;
		
	}
	
	
	
	
	/**
	 * Transform a 1D array of MPVariable into a 2D MATRIX  
	 * @param num_robot -> Number of robots
	 * @param num_tasks -> Number of tasks
	 * @param solver -> An optimization problem defined with {@link #optimization_problem}
	 * @return 2D Matrix of Decision_Variable
	 */
	private static MPVariable [][] tranform_array(MPSolver solver) {
		int num_robot = numRobot + Dummy_Robot;
		int num_tasks = numTask + Dummy_Task;
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
		int num_robot = numRobot + Dummy_Robot;
		int num_tasks = numTask + Dummy_Task;
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
	private static double evaluate_B_function(int robot_ID , int tasks_ID, ReedsSheppCarPlanner rsp, Task[] Tasks){
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double Path_length = 0;
		// N = M
		if(tasks_ID < numTask && robot_ID < numRobot) {
			
			rsp.setStart(Tasks[tasks_ID].getStartPose());
			rsp.setGoals(Tasks[tasks_ID].getGoalPose());
			
		}
		/*
		else { // N != M
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
		*/
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
	
	private static double evaluate_F_function(int robot_ID ,int tasks_ID,double [][] Assignment_Matrix,ReedsSheppCarPlanner rsp,Task [] Tasks,TrajectoryEnvelopeCoordinatorSimulation tec,MPSolver solver){
		//Evaluate the delay time on completion time for the actual couple of task and ID
		//Initialize the time delay 
		
		
		
		FleetMasterInterface flint = new FleetMasterInterface(0., 0., 0., 0.1, 500, 500, true);	
		double delay = 0;
		if(Assignment_Matrix[robot_ID][tasks_ID]>0) {
			//Evaluate the path for this couple of robot and task
			// N = M
			if(tasks_ID < numTask && robot_ID < numRobot) {
				rsp.setStart(Tasks[tasks_ID].getStartPose());
				rsp.setGoals(Tasks[tasks_ID].getGoalPose());
			}
			/*
			
			else { // N != M
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
			*/
			//rsp.setStart(startsPose[robot_ID]);
			//rsp.setGoals(goalsPose[tasks_ID]);
			rsp.plan();
			PoseSteering[] pss1 = rsp.getPath();
			TrajectoryEnvelope te1 = tec.getCurrentTrajectoryEnvelope(robot_ID+1);
			Trajectory iiprova = te1.getTrajectory();
			Trajectory ii = new Trajectory(pss1);
			
			
			CumulatedIndexedDelaysList uu = new CumulatedIndexedDelaysList();
			
			
			
			te1.setTrajectory(ii);
		
			
			
			
			//te1prova.setFootprint(tec.getFootprint(robot_ID+1));
		
			double Path_length = Missions.getPathLength(pss1);
			if(Approximation_Flag) {
				solver.objective().setCoefficient(solver.variables()[robot_ID*Assignment_Matrix[0].length+tasks_ID], Path_length);
			}
			for(int m=0; m < Assignment_Matrix.length; m++) {
				for(int n=0; n < Assignment_Matrix[0].length; n++) {
					if (Assignment_Matrix[m][n]>0 && m!=robot_ID && n!=tasks_ID) {
						rsp.setStart(Tasks[n].getStartPose());
						rsp.setGoals(Tasks[n].getGoalPose());
						rsp.plan();
						PoseSteering[] pss2 = rsp.getPath();
						
						TrajectoryEnvelope te2 = tec.getCurrentTrajectoryEnvelope(m+1);
						
						
						Trajectory jj = new Trajectory(pss2);
						
						Trajectory jjprova = te2.getTrajectory();
						te2.setTrajectory(jj);
						
						
						CumulatedIndexedDelaysList mm = new CumulatedIndexedDelaysList();
						
						
						
						te2.setTrajectory(jjprova);
						
					}
				}
			}
			
			te1.setTrajectory(iiprova);
		}
		delay = 10;
		
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
	private static double evaluate_B_function_approx(int robot_ID ,int tasks_ID, ReedsSheppCarPlanner rsp, Task[] Tasks){
		//Evaluate the path length for the actual couple of task and ID
		//Set the starting and the arrival point
		double x1 = 0;
		double y1 = 0;
		double x2 = 0;
		double y2 = 0;
		if(tasks_ID < numTask && robot_ID < numRobot) {
			// N = M
			x1 = Tasks[tasks_ID].getStartPose().getX();
			y1 = Tasks[tasks_ID].getStartPose().getY();
			x2 = Tasks[tasks_ID].getGoalPose().getX();
			y2 = Tasks[tasks_ID].getGoalPose().getY();
			
		}
		/*
		else {// N != M
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
		*/
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
	public static double [][][] feasible_solution(int num_robot,int num_tasks){
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
	private static MPSolver optimization_problem(int num_robot,int num_tasks) {
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
	public static MPSolver optimization_problem_complete(int num_robot,ReedsSheppCarPlanner rsp,Task [] Tasks,boolean Approximation) {
		int num_tasks = Tasks.length;
		TasksMissions = Tasks;
		
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
		MPVariable [][] Decision_Variable = tranform_array(solver); 
		double sum_path_length = 0;
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = solver.objective();
	
	    Approximation_Flag = Approximation;
	    if (Approximation == false) {
	    	 for (int i = 0; i < num_robot; i++) {
				 for (int j = 0; j < num_tasks; j++) {
					 objective.setCoefficient(Decision_Variable[i][j], evaluate_B_function(i,j,rsp,Tasks));
					 sum_path_length = sum_path_length + objective.getCoefficient(Decision_Variable[i][j]);
				 }			 
			 }
	    }else {
	    	 for (int i = 0; i < num_robot; i++) {
				 for (int j = 0; j < num_tasks; j++) {
					 objective.setCoefficient(Decision_Variable[i][j], evaluate_B_function_approx(i,j,rsp,Tasks));
					 sum_path_length = sum_path_length + objective.getCoefficient(Decision_Variable[i][j]);
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
	 * with n = number of robot and m = number of tasks
	 * Assignments are precomputed in this case  
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
	public static double [][] solve_optimization_problem_precomputedAssignment(ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		int num_tasks = numTask + Dummy_Task;
		int num_robot = numRobot + Dummy_Robot;
		Task [] Tasks = TasksMissions; 
		//Define the optimization problem
		double [][] Assignment_Matrix = new double[num_robot][num_tasks];
		//Evaluate the number of feasible solutions for the optimization problem
		int pp = number_of_Feasible_Solution(num_robot,num_tasks);
		//Evaluate the set of feasible solutions for the optimization problem
		double [][][] ii = feasible_solution(num_robot,num_tasks);
		//Initialize the optimal assignment and the cost associated to it
		double [][] Assignment_Matrix_ott = new double[num_robot][num_tasks];
		double obj_value_ott = 100000000;
		//Start to slide all possible solutions 
		for (int k = 0; k < pp; k++) {
			Assignment_Matrix = ii[k];
			double obj_value = 0;
			double cost_B = 0;
			double cost_F = 0;
			for (int i = 0; i < num_robot ; i++) {
				for(int j=0;j < num_tasks; j++) {
					if(Assignment_Matrix[i][j]>0) {
						cost_B = cost_B + evaluate_B_function(i,j,rsp,Tasks);
						cost_F = cost_F + evaluate_F_function(i,j,Assignment_Matrix,rsp,Tasks,tec,solver);						
					}				
				}		
			}
			obj_value = alpha * cost_B + (1- alpha)*cost_F;
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
	
	public static double [][] solve_optimization_problem(ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		//Initialize the optimal assignment and the cost associated to it
		int num_tasks = numTask + Dummy_Task;
		int num_robot = numRobot + Dummy_Robot;
		Task [] Tasks = TasksMissions; 
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
			for (int i = 0; i < num_robot ; i++) {
				for(int j=0;j < num_tasks; j++) {
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

	public static double [][] solve_optimization_problem_exact(ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,MPSolver solver,TrajectoryEnvelopeCoordinatorSimulation tec,double alpha){
		Approximation_Flag = false;
		int num_tasks = numTask + Dummy_Task;
		int num_robot = numRobot + Dummy_Robot;
		Task [] Tasks = TasksMissions; 
		//Initialize the optimal assignment and the cost associated to it
		double [][] Assignment_Matrix_ott = new double[num_robot][num_tasks];
		double obj_value_ott = 100000000;
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
						cost_B = cost_B + evaluate_B_function(i,j,rsp,Tasks);
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
	public static TrajectoryEnvelopeCoordinatorSimulation Task_Assignment(double [][] Assignment_Matrix,ReedsSheppCarPlanner rsp,Pose [] startsPose,Pose [] goalsPose,TrajectoryEnvelopeCoordinatorSimulation tec){
		int robot_IDs = Assignment_Matrix.length;
		int num_tasks = Assignment_Matrix[0].length;
		for (int i = 0; i < robot_IDs; i++) {
			 for (int j = 0; j < num_tasks; j++) {
				 if(Assignment_Matrix[i][j]>0) {
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
					//rsp.setStart(startsPose[i]);
					//rsp.setGoals(goalsPose[j]);			
					if (!rsp.plan()) throw new Error ("No path between " + startsPose[i] + " and " + goalsPose[j]);
					PoseSteering[] ff = rsp.getPath();
					tec.addMissions(new Mission(i+1,ff));
				 } else {
					 System.out.println("Nope");
					
				 }
			 }
		 }
		return tec;
	}
	
	}

