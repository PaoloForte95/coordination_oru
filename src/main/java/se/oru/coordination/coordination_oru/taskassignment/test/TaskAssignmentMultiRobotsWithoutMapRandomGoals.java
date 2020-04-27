package se.oru.coordination.coordination_oru.taskassignment.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Comparator;
import java.util.Random;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import com.vividsolutions.jts.geom.Coordinate;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.taskassignment.TaskAssignment;
import se.oru.coordination.coordination_oru.taskassignment.Robot;
import se.oru.coordination.coordination_oru.taskassignment.Task;
import com.google.ortools.linearsolver.*;


@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TaskAssignmentMultiRobotsWithoutMapRandomGoals {
	//load library used for optimization
	 static {
		    System.loadLibrary("jniortools");
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
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 0, 0);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);

		
	
		
	   
		Pose startPoseRobot1 = new Pose(9.0,12.0,0.0);
		Pose startPoseRobot2 = new Pose(11.0,16.0,-Math.PI/4);
		Pose startPoseRobot3 = new Pose(10.0,20.0,0.0);
		Pose startPoseRobot4 = new Pose(16.0,30.0,-Math.PI/2);
		Pose startPoseRobot5 = new Pose(5.0,20.0,Math.PI/2);
		Pose startPoseRobot6 = new Pose(5.0,8.0,0.0);
		Pose startPoseRobot7 = new Pose(18.0,20.0,Math.PI/2);
		Pose startPoseRobot8 = new Pose(7.0,32.0,-Math.PI/2);
		
		
		Robot robot1 = new Robot(1, 1);
		Robot robot2 = new Robot(2, 1);
		Robot robot3 = new Robot(3, 1);
		Robot robot4 = new Robot(4, 1);
		Robot robot5 = new Robot(5, 1);
		Robot robot6 = new Robot(6, 1);
		Robot robot7 = new Robot(7, 1);
		Robot robot8 = new Robot(8, 1);
		
		
		
		tec.addRobot(robot1,startPoseRobot1);
		tec.addRobot(robot2,startPoseRobot2);
		tec.addRobot(robot3,startPoseRobot3);
		tec.addRobot(robot4, startPoseRobot4);
		tec.addRobot(robot5, startPoseRobot5);
		//tec.addRobot(robot6, startPoseRobot6);
		//tec.addRobot(robot7, startPoseRobot7);
		
		String yamlFile = "maps/map-empty.yaml";
	
		
		
	
		
		PrintStream fileStream = null;
		try {
			fileStream = new PrintStream(new File("RandomPose.txt"));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Random rand = new Random();
		//create Random Start and Goal Poses for Task
		
		ReedsSheppCarPlanner rsp2 = new ReedsSheppCarPlanner();
		rsp2.setRadius(0.2);
		rsp2.setTurningRadius(4.0);
		rsp2.setFootprint(tec.getDefaultFootprint());
		rsp2.setDistanceBetweenPathPoints(0.5);
		rsp2.setMapFilename("maps"+File.separator+Missions.getProperty("image", "maps/map-empty.yaml"));
		double res2 = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
		rsp2.setMapResolution(res2);
		rsp2.setPlanningTimeInSecs(2);
		
		
		TaskAssignment assignmentProblem = new TaskAssignment();
		
		double rangeMinStart = 15.0;
		double rangeMinStarty = 4.0;
		double rangeMinGoal = 20.0;
		double rangeMinGoaly = 6.0;
		double rangeMax = 50.0;
		double rangeMaxy = 40.0;
		Pose RandomStartPose = new Pose(0.0,0.0,0);
		Pose RandomGoalPose = new Pose(0.0,0.0,0);
		for(int j=1;j <= 5; j++) {
			boolean flag1 = false;
			while(! flag1) {
				double randomStartPosex = rangeMinStart + (rangeMax-rangeMinStart)*rand.nextDouble();
				double randomStartPosey = rangeMinStarty + (rangeMaxy-rangeMinStarty)*rand.nextDouble();
				RandomStartPose = new Pose(randomStartPosex,randomStartPosey,Math.PI/2);
				flag1 = rsp2.isFree(RandomStartPose);
				fileStream.println("Pose startPoseGoal"+j+" = new Pose" + RandomStartPose+";");
			}
			boolean flag2 = false;
			while(! flag2) {
				double randomGoalPosex = rangeMinGoal + (rangeMax-rangeMinGoal)*rand.nextDouble();
				double randomGoalPosey =  rangeMinGoaly + (rangeMaxy-rangeMinGoaly)*rand.nextDouble();
				RandomGoalPose = new Pose(randomGoalPosex,randomGoalPosey,Math.PI/2);
				flag2 = rsp2.isFree(RandomGoalPose);
				fileStream.println("Pose goalPoseGoal"+j+" = new Pose" + RandomGoalPose+";");
			}
			Task taskRandom = new Task(j,RandomStartPose,RandomGoalPose,1);
			assignmentProblem.addTask(taskRandom);
			
			
		}
		
		
		
		int numPaths = 1;
		
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
		
	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		double alpha = 1.0;
		
		assignmentProblem.setmaxNumPaths(numPaths);
		assignmentProblem.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		assignmentProblem.instantiateFleetMaster(0.1, false);
		assignmentProblem.setFleetVisualization(viz);
		assignmentProblem.setCoordinator(tec);
		assignmentProblem.setLinearWeight(alpha);
		assignmentProblem.setCostFunctionsWeight(1.0, 0.0, 0.0);
		MPSolver solver = assignmentProblem.buildOptimizationProblemWithBNormalized(tec);
		double [][][] assignmentMatrix = assignmentProblem.solveOptimizationProblem(solver,tec);
		
		for (int i = 0; i < assignmentMatrix.length; i++) {
			for (int j = 0; j < assignmentMatrix[0].length; j++) {
				for(int s = 0; s < numPaths; s++) {
					System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ assignmentMatrix[i][j][s]);
					if(assignmentMatrix[i][j][s] == 1) {
						System.out.println("Robot " +(i+1) +" is assigned to Task "+ (j+1)+" throw Path " + (s+1));
					}
				}
			} 
		}
		assignmentProblem.TaskAllocation(assignmentMatrix,tec);	
		//Missions.saveScenario("RandomGoals");
	}
}
