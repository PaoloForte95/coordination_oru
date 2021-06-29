package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.RobotReport;

public interface FleetVisualization {
	
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String ... extraStatusInfo);
	
	@Deprecated
	public void displayRobotState(Polygon fp, RobotReport rr, String ... extraStatusInfo);
	
	public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor);
	
	public void addEnvelope(TrajectoryEnvelope te);
	
	public void removeEnvelope(TrajectoryEnvelope te);
	
	public void updateVisualization();
	
	public void setMap(String yamlFile);
	
	public int periodicEnvelopeRefreshInMillis();

	public void displayMaterial(Pose poseMaterial,int materialID,double materialAmount,String... extraStatusInfo);

	public void displayWaypoint(Pose pose, String name);

	public void loadMaterial(int robotID, int materialID) ;

	public void updateMaterialAmount(int getmaterialID, double amount, String toLocation);

}
