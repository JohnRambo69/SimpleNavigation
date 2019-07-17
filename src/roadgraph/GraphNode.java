package roadgraph;
/**
 * @author LucasS
 */

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class GraphNode implements Comparable<GraphNode>{
	
	private GeographicPoint location;
	private List<GraphNode> neighboursList;
	private double distanceTo;
	private double distanceToGoal;

	public GraphNode(GeographicPoint location) {
		this.location = location;
		this.neighboursList = new ArrayList<GraphNode>();
		this.distanceTo = Double.POSITIVE_INFINITY;
		this.distanceToGoal = 0.0;
	}
	
	public void addEdge(GraphNode edge) {
		neighboursList.add(edge);
	}
	
	public GeographicPoint getGeoPoint() {
		return location;
	}
	
	public List<GraphNode> getEdges(){
		return neighboursList;
	}
	
	public double getDistance() {
		return distanceTo;
	}
	
	public void setDistance(double distanceTo) {
		this.distanceTo = distanceTo;
	}
	
	public double getDistanceToGoal() {
		return distanceToGoal;
	}
	
	public void setDistanceToGoal(double dist) {
		this.distanceToGoal = dist;
	}
	
	public int compareTo(GraphNode other) {
		if(this.distanceTo > other.distanceTo) {
			return 1;
		}
		else if(this.distanceTo < other.distanceTo){
			return -1;
		}
		else {
			return 0;
		}
	}
	
	public String toString() {
		return  location.toString();
	}
}
