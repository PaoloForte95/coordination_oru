package se.oru.coordination.coordination_oru.taskassignment;

import com.vividsolutions.jts.geom.Coordinate;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.ForwardModel;

public class Robot {
	
	 protected int robotID;
	 protected int robotType;
	 protected Coordinate[] footprint;
	 protected ForwardModel fm = null;
	
	 
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
		 * @param footprint -> The footprint of the robot. 
		 * @param fm -> The forward model of the robot.
		 */
	 public Robot(int robotID, int robotType,Coordinate[] footprint, ForwardModel fm) {
		 this.robotID = robotID;
		 this.robotType = robotType;
		 this.footprint = footprint;
		 this.fm = fm;
		
	 }
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param startingPosition -> The Starting Position of the Robot.
	  */
	 public Robot(int robotID) {
		 this(robotID,1,DEFAULT_FOOTPRINT,DEFAULT_FORWARD_MODEL);
	 }
	 
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param RobotType -> The type of the Robot
	  */
	 public Robot(int robotID,int robotType) {
		 this(robotID,robotType,DEFAULT_FOOTPRINT,DEFAULT_FORWARD_MODEL);
	 }
	 
	 
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param RobotType -> The type of the Robot
	  * @param footprint -> The footprint of the robot
	  */
	 public Robot(int robotID,int robotType,Coordinate[] footprint) {
		 this(robotID,robotType,footprint,DEFAULT_FORWARD_MODEL);
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








