package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import aima.core.agent.Model;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
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
public class TestCreateEnvelope {
	//load library used for optimization
	 static {
		    System.loadLibrary("jniortools");
		  }
	 
	 protected int maxNumPaths;
		protected int numRobot;
		protected int numRobotAug;
		protected int numTaskAug;
		protected ArrayList <Integer> IDsIdleRobots = new ArrayList <Integer>();
		protected int numTask;
		protected int dummyTask;
		protected int dummyRobot;
		protected double [][][] pathArray;
		protected double timeOut = Double.POSITIVE_INFINITY;
		protected double MaxPathLength = Integer.MAX_VALUE;
		protected ArrayList <Task> taskQueue = new ArrayList <Task>();
		
		
		
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
		
	 
		public boolean addTask(Task task) {
			if (task == null) {
				throw new Error("Cannot add the task");
			}
			boolean TaskisAdded = taskQueue.add(task);
			return TaskisAdded;
		}	
		
		public int getNumRobotAug() {
			return this.numRobotAug;
		}	
		
		
		
	 public void checkOnSharedTask() {
		 int sum=0;
		 
		 numRobotAug = numRobot;
		 System.out.println("numRobotAug" + numRobotAug);
		 for(Task taskID : taskQueue) {
			 sum += taskID.getRobotRequired();
			 
		 }
		 System.out.println("suuum" +sum +" diff " + (sum-numRobot));
		 if((sum-numRobot)>0) {
			 numRobotAug = numRobot + (sum-numRobotAug);
		 }
		 System.out.println("numRobotAug" + numRobotAug);
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
				 int RobotReq= 1;
				 if(j < taskQueue.size()) {
					 Task tt= taskQueue.get(j);
					 RobotReq = tt.getRobotRequired();
				 }
				
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
		
		
		

		public MPSolver buildOptimizationProblemWithBNormalized(AbstractTrajectoryEnvelopeCoordinator tec) {
			
			numTask = taskQueue.size();
			//Get free robots and their IDs
			numRobot = tec.getIdleRobots().size();
			IDsIdleRobots = tec.getIdleRobots();
			
			
			dummyRobotorTask(numRobot,numTask,tec);
			checkOnSharedTask();
			
			//Evaluate dummy robot and dummy task
			
			System.out.println("llll" +numRobotAug + "ppppp " + numTaskAug);
			//Take the number of tasks
			
			//Build the solver and an objective function
			MPSolver optimizationProblem = buildOptimizationProblem(numRobotAug,numTask);
			
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
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		
		
		//BrowserVisualization viz = new BrowserVisualization();
		//viz.setInitialTransform(20, 0, 0);
		//tec.setVisualization(viz);
		//tec.setUseInternalCriticalPoints(false);

		String yamlFile = "maps/map-empty.yaml";
		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)

		
		Random rand = new Random();
		double alpha = 1.0;
		double timeOut = 15*1000;

		int numPath = 1;
	
		TestCreateEnvelope test = new TestCreateEnvelope();
		int numPaths = 1;
		double delta = 0;
		for(int i = 1; i<= 3; i++) {
			
			Pose startPoseRobot = new Pose(4.0,(6.0 + delta),0.0);
			int robotType = rand.nextInt(2)+1;
			Robot robot = new Robot(i,1);
			tec.addRobot(robot,startPoseRobot);
			Pose startPoseGoal = new Pose(15.0,(6.0 + delta),0.0);
			Pose goalPoseRobot = new Pose(30.0 ,(6.0 + delta) ,0.0);
	
			int taskType = rand.nextInt(2)+1;
			Task task = new Task(i,startPoseGoal,goalPoseRobot,1);
			if(i==2) {
				task = new Task(i,10,startPoseGoal,goalPoseRobot,1);
				task.setRobotRequired(2);
			}else if(i==1) {
				task = new Task(i,5,startPoseGoal,goalPoseRobot,1);
			}else if(i==4) {
				task = new Task(i,10,startPoseGoal,goalPoseRobot,1);
			}
			test.addTask(task);

			delta += 3.0;
			
		}
		
		
		PrintStream fileStream = null;

		try {
			fileStream = new PrintStream(new File("ProvaTask.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		fileStream.println(tec.getRobotType(1)+"");
		

		for (int robotID : tec.getIdleRobots()) {
			Coordinate[] footprint = tec.getFootprint(robotID);
	
			
			//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setRadius(0.2);
			rsp.setFootprint(footprint);
			rsp.setTurningRadius(4.0);
			rsp.setDistanceBetweenPathPoints(0.5);
			rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/map-empty.yaml"));
			double res = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
			rsp.setMapResolution(res);
			rsp.setPlanningTimeInSecs(2);
			tec.setMotionPlanner(robotID, rsp);
		}
		tec.setFakeCoordinator(true);
		//Solve the problem to find some feasible solution
	
		
		double [][][]pathMatrix = new double[test.numRobotAug][test.numTaskAug][numPath];
		for (int i = 0; i < test.numRobotAug; i++) {
			 for (int j = 0; j < test.numTaskAug; j++) {
				 for(int s = 0; s < numPath; s++) {
					 pathMatrix[i][j][s] = 17.50;
				 }
			 }
		}
		test.setPathArray(pathMatrix);
		
		MPSolver optProblem = test.buildOptimizationProblemWithBNormalized(tec);
		MPSolver.ResultStatus resultStatus = optProblem.solve();
		long timeOffsetInitial = Calendar.getInstance().getTimeInMillis();
		System.out.println("Test: " + resultStatus);
		System.out.println("NumRobot: " + test.numRobot);
		System.out.println("NumTask: " + test.numTask);
		System.out.println("NumRobot A: " + test.numRobotAug);
		System.out.println("NumTask A: " + test.numTaskAug);
		MPVariable [][][] ll= test.tranformArray(optProblem);
		for (int i = 0; i < test.numRobotAug; i++) {
			 for (int j = 0; j < test.numTaskAug; j++) {
				 for(int s = 0; s < numPath; s++) {
					 System.out.println("Value >> " + ll[i][j][s].solutionValue() +" i>> " +(i+1) + " j>> " + (j+1) + " s>> " + (s+1) );
				 }
			 }
		}
	}
}
