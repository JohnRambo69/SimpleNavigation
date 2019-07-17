package roadgraph;
/**
 * @author LucasS
 */

import geography.GeographicPoint;

public class EdgeNode {
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double length;
	
	public EdgeNode(GeographicPoint from, GeographicPoint to, String roadName,
	String roadType, double length) {
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public GeographicPoint getFrom() {
		return from;
	}
	
	public GeographicPoint getTo() {
		return to;
	}
	
	public String getRoadName() {
		return roadName;
	}
	
	public String getRoadType() {
		return roadType;
	}
	
	public double length() {
		return length;
	}
	
	public String toString() {
		return "Road Name: " + roadName + ", Length: " + length + ", Road type: " + roadType;
	}

}
