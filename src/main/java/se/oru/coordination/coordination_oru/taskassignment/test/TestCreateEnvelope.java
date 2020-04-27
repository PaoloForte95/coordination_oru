package se.oru.coordination.coordination_oru.taskassignment.test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

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

public class TestCreateEnvelope {

	
		public static double probability(double f1, double f2, double temp) {
		    if (f2 < f1) return 1;
		    return 1-Math.exp((f1 - f2) / temp);
		}
		public static void main(String[] args) throws InterruptedException {
			Coordinate footprint1 = new Coordinate(-1.0,0.5);
			Coordinate footprint2 = new Coordinate(1.0,0.5);
			Coordinate footprint3 = new Coordinate(1.0,-0.5);
			Coordinate footprint4 = new Coordinate(-1.0,-0.5);
			ArrayList <Integer> IDsRandomRobots2 = new ArrayList <Integer>();
			IDsRandomRobots2.add(1);
			IDsRandomRobots2.add(2);
			IDsRandomRobots2.add(7);
			IDsRandomRobots2.add(3);
			System.out.println(true == true);
			int cont=1;
			int prova2prec = 0;
			int prova2 = 0;
			while(IDsRandomRobots2.size() > 0) {
				prova2prec = prova2;
				int ind2 = (int) Math.floor(Math.random()*IDsRandomRobots2.size());
				prova2 = IDsRandomRobots2.get(ind2);
				IDsRandomRobots2.remove(ind2);
				double  ll = probability(prova2prec,prova2,cont);
				System.out.println(ll);
				System.out.println(Math.random());
				System.out.println(Math.exp((prova2prec - prova2) / cont));
				System.out.println(prova2prec+"--"+prova2);
				cont +=1;
			}
			int numIteration =1;
			for(int fat= 1 ; fat <= 5; fat++) {
				numIteration *= fat;
			}
			System.out.println(numIteration);
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setRadius(0.2);
			rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
			rsp.setTurningRadius(3.0);
			rsp.setDistanceBetweenPathPoints(0.1);

			Pose poseFrom1 = new Pose(0.0,0.0,0.0);
			Pose poseTo1 = new Pose(10.0,10.0,Math.PI);

			Pose poseFrom2 = new Pose(0.0,10.0,0.0);
			Pose poseTo2 = new Pose(10.0,0.0,Math.PI);

			rsp.setStart(poseFrom1);
			rsp.setGoals(poseTo1);
			if (!rsp.plan()) throw new Error("Cannot plan for Robot1");
			SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(rsp.getPath(), footprint1, footprint2, footprint3, footprint4);

			rsp.setStart(poseFrom2);
			rsp.setGoals(poseTo2);
			if (!rsp.plan()) throw new Error("Cannot plan for Robot2");
			SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(rsp.getPath(), footprint1, footprint2, footprint3, footprint4);

			CriticalSection[] css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, 2.0);
			System.out.println("Found " + css.length + " critical sections");
			System.out.println(Arrays.toString(css));
			

	}

}
