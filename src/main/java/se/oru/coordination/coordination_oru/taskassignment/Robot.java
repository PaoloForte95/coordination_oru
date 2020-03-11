package se.oru.coordination.coordination_oru.taskassignment;


import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;
import javax.swing.SwingUtilities;
import org.apache.commons.collections.comparators.ComparatorChain;
import org.metacsp.framework.Constraint;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;
import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.ForwardModel;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.StringUtils;
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
			 new Coordinate(-1.7, 0.7),	//back left
			 new Coordinate(-1.7, -0.7),	//back right
			 new Coordinate(2.7, -0.7),	//front right
			 new Coordinate(2.7, 0.7)	//front left
		};
		
	 /**
		 * The default forward model used for robots if none is specified.
		 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
		 */
	 
	 private static ForwardModel DEFAULT_FORWARD_MODEL = new ConstantAccelerationForwardModel(2, 2,1000, 1000,30);
	 /**
		 * Create a new {@link Robot} 
		 * @param RobotID -> The ID of the Robot
		 * @param RobotType -> The type of the Robot
		 * @param StartingPosition The Starting Position of the Robot.
		 * @param footprint The footprint of the robot. 
		 * @param fm The forward model of the robot.
		 */
	 public Robot(int robotID, int robotType, Pose startingPosition,Coordinate[] footprint, ForwardModel fm) {
		 this.robotID = robotID;
		 this.robotType = robotType;
		 this.startingPosition = startingPosition;
		 this.footprint = footprint;
		 this.fm = fm;
		
	 }
	 
	 public Robot(int robotID,Pose startingPosition) {
		 this(robotID,0,startingPosition,DEFAULT_FOOTPRINT,DEFAULT_FORWARD_MODEL);
	 }

	 public int getrobotID() {
		 return this.robotID;
	 }

	
	 public int getrobotType() {
		 return this.robotType;
	 }

	 public void setrobotType(int robotType) {
		 this.robotType= robotType;;
	 }
	 
	 public Pose getstartingPosition() {
		 return this.startingPosition;
	 }

	 public  Coordinate[] getfootprint() {
		 return this.footprint;
	 }

	 public void setfootprint(Coordinate[] footprint) {
		 this.footprint= footprint;;
	 }
	 
	 public  ForwardModel getForwardModel() {
		 return this.fm;
	 }
	 
	 public void setForwardModel(ForwardModel fm) {
		 this.fm= fm;
	 }
	 
}








