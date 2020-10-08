package se.oru.coordination.coordination_oru.taskassignment.test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class TestScalability {
	
	static {
	    System.loadLibrary("jniortools");
	  }
	
	protected int maxNumPaths;
	protected int numRobot;
	protected int numTask;
	protected double [][][] pathArray;
	protected double timeOut = Double.POSITIVE_INFINITY;
	protected double MaxPathLength = Integer.MAX_VALUE;
	
	public TestScalability(int numRobot,int numTask,int maxNumPaths) {
		this.numRobot = numRobot;
		this.numTask = numTask;
		this.maxNumPaths = maxNumPaths;
	}
	
	public void setPathArray(double [][][] matrix) {
		this.pathArray = matrix;
	}
	private MPVariable [][][] tranformArray(MPSolver optimizationProblem) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][][] decisionVariable = new MPVariable [numRobot][numTask][maxNumPaths];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobot; i++) {
			 for (int j = 0; j < numTask; j++) {
				 for (int s = 0; s < maxNumPaths; s++) {
					 decisionVariable[i][j][s] = array1D[i*numTask*maxNumPaths+j*maxNumPaths+s];
				 }
				 
			 }
	    }
		return decisionVariable;
	}
	
	private MPSolver constraintOnPreviousSolution(MPSolver optimizationProblem, double [][][] assignmentMatrix) {
		//Take decision Variable from Optimization Problem
		MPVariable [][][] DecisionVariable = tranformArray(optimizationProblem);
		//Initialize a Constraint
		//MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
		MPConstraint c2 = optimizationProblem.makeConstraint(0,numRobot-1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobot; i++) {
    		for (int j = 0; j < numTask; j++) {
    			for(int s = 0;s < maxNumPaths; s++) {
    					if (assignmentMatrix[i][j][s] >0) {
    						c2.setCoefficient(DecisionVariable[i][j][s],1);
    					}
    			}
    		}		
		 }
    	//Return the updated Optimization Problem
    	return optimizationProblem;
	}
	
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
	
		 for (int i=0; i< numRobot; i++ ) {
				for (int j=0; j< numTask; j++ ) {
					for(int s = 0; s < maxNumPaths; s++) {
							 if (i < numRobot) { //Considering only real Robot
								 double pss = pathArray[i][j][s];
								 if(pss==0) {
									 MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
									 c3.setCoefficient(decisionVariable[i][j][s],1); 
								 }
							 }
					}
				}
		 }
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}
	
	
	

	public MPSolver buildOptimizationProblemWithBNormalized() {
		
		//Take the number of tasks

		//Build the solver and an objective function
		MPSolver optimizationProblem = buildOptimizationProblem(numRobot,numTask);
		
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem); 
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
    	 for (int i = 0; i < numRobot; i++) {
			 for (int j = 0; j < numTask; j++) {
				 for(int s = 0; s < maxNumPaths; s++) {
					 double pathLength  =  pathArray[i][j][s];
					 if ( pathLength != MaxPathLength) {
						 //Set the coefficient of the objective function with the normalized path length
						 objective.setCoefficient(decisionVariable[i][j][s], pathLength); 
					 }else { // if the path does not exists or the robot type is different from the task type 
						//the path to reach the task not exists
						//the decision variable is set to 0 -> this allocation is not valid
						MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
						c3.setCoefficient(decisionVariable[i][j][s],1); 
					 }
				 }
			 }			 
		 }
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}
	
	
	
		
		public static void main(String[] args) throws InterruptedException {
			long initialTime = 	Calendar.getInstance().getTimeInMillis();
			int numRobot = 3;
			int numTask = 3;
			int numPath = 1;
			TestScalability test = new TestScalability(numRobot,numTask,numPath);
			double [][][]pathMatrix = new double[numRobot][numTask][numPath];
			for (int i = 0; i < numRobot; i++) {
				 for (int j = 0; j < numTask; j++) {
					 for(int s = 0; s < numPath; s++) {
						 pathMatrix[i][j][s] = 17.50;
					 }
				 }
			}
			test.setPathArray(pathMatrix);
			MPSolver optProblem = test.buildOptimizationProblemWithBNormalized();
			MPSolver.ResultStatus resultStatus = optProblem.solve();
			long timeOffsetInitial = Calendar.getInstance().getTimeInMillis();
			long timeOffset = 0;
			double timeOut = 15*1000;
			while(resultStatus != MPSolver.ResultStatus.INFEASIBLE && timeOffset < timeOut) {
				
				resultStatus = optProblem.solve();
				long timeOffsetFinal = Calendar.getInstance().getTimeInMillis();
				timeOffset = timeOffsetFinal - timeOffsetInitial;
				System.out.println("Time for solution " + timeOffset/1000 +" [s]");
			}
			
			long finalTime = 	Calendar.getInstance().getTimeInMillis();
			long timeRequire = finalTime - initialTime;
			System.out.println("Required Time for Optimization " + timeRequire +" [ms]");
			System.out.println("Number of robot: " + numRobot);
			System.out.println("Number of Task: " + numTask);
			System.out.println("Number of Path: " + numPath);
			System.out.println("Number of Path: " + (Calendar.getInstance().getTimeInMillis()<Double.POSITIVE_INFINITY));
			
		};

}
