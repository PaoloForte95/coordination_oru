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
import se.oru.coordination.coordination_oru.ForwardModel;
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
public class Robot {
	 protected int robotID;
	 protected int robotType;
	 protected Coordinate[] footprint;
	 protected ForwardModel fm = null;
	 protected Pose startingPosition;
	
	 
	 /**
		 * The default footprint used for robots if none is specified.
		 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
		 */
	 private static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			 new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5)
		};
		
	 
	 
	 
	 private static ConstantAccelerationForwardModel DEFAULT_FORWARD_MODEL = new ConstantAccelerationForwardModel(1, 1, 1000.0, 1000, 30);
	 /**
		 * Create a new {@link Robot} 
		 * @param robotID -> The ID of the Robot
		 * @param robotType -> The type of the Robot
		 * @param startingPosition -> The Starting Position of the Robot.
		 * @param footprint -> The footprint of the robot. 
		 * @param fm -> The forward model of the robot.
		 */
	 public Robot(int robotID, int robotType, Pose startingPosition,Coordinate[] footprint, ForwardModel fm) {
		 this.robotID = robotID;
		 this.robotType = robotType;
		 this.startingPosition = startingPosition;
		 this.footprint = footprint;
		 this.fm = fm;
		
	 }
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param startingPosition -> The Starting Position of the Robot.
	  */
	 public Robot(int robotID,Pose startingPosition) {
		 this(robotID,1,startingPosition,DEFAULT_FOOTPRINT,DEFAULT_FORWARD_MODEL);
	 }
	 
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param RobotType -> The type of the Robot
	  * @param startingPosition -> The Starting Position of the Robot.
	  */
	 public Robot(int robotID,int robotType,Pose startingPosition) {
		 this(robotID,robotType,startingPosition,DEFAULT_FOOTPRINT,DEFAULT_FORWARD_MODEL);
	 }
	 

	 public int getRobotID() {
		 return this.robotID;
	 }

	
	 public int getRobotType() {
		 return this.robotType;
	 }

	 public void setRobotType(int robotType) {
		 this.robotType= robotType;;
	 }
	 
	 public Pose getStartingPosition() {
		 return this.startingPosition;
	 }

	 public  Coordinate[] getFootprint() {
		 return this.footprint;
	 }

	 public void setFootprint(Coordinate[] footprint) {
		 this.footprint= footprint;;
	 }
	 
	 public  ForwardModel getForwardModel() {
		 return this.fm;
	 }
	 
	 public void setForwardModel(ForwardModel fm) {
		 this.fm= fm;
	 }
	 
}








