/**
 * @author UCSD MOOC development team and LucasS
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;


import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private List<GraphNode> nodeList;
	private List<EdgeNode> edgeList;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
	nodeList = new ArrayList<GraphNode>();
	edgeList = new ArrayList<EdgeNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return nodeList.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> result = new HashSet<GeographicPoint>();
			for(GraphNode g : nodeList) {
				GeographicPoint p = g.getGeoPoint();
				result.add(p);
			}
		return result;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edgeList.size();
	}
	
	public List<EdgeNode> getAllEdges(){
		return edgeList;
	}
	
	public GraphNode getVertex(GeographicPoint point) {
		for(GraphNode gn : nodeList) {
			if(gn.getGeoPoint().equals(point)) {
				return gn;
			}
		}
		return null;
	}
	
	private double roadLength(GraphNode start, GraphNode end) {
		for(EdgeNode e : edgeList) {
			if(e.getFrom().equals(start.getGeoPoint()) && e.getTo().equals(end.getGeoPoint()))
				return e.length();
		}
		return 0.0;
	}

	private void clearGraphNodes() {
		if(!nodeList.isEmpty()) {
			for(GraphNode gn : nodeList) {
			gn.setDistance(Double.POSITIVE_INFINITY);
			}
		}
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(checkVertexExist(location) || location == null) {
			return false;
		}
		GraphNode gn = new GraphNode(location);
		nodeList.add(gn);
		// TODO: Implement this method in WEEK 3
		return true;
	}
	
	private boolean checkVertexExist(GeographicPoint location) {
		for(GraphNode g : nodeList) {
			if(g.getGeoPoint().equals(location)) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if(!checkVertexExist(from) || !checkVertexExist(to)
				|| length < 0
				||  from == null || to == null || roadName == null || roadType == null) {
			throw new IllegalArgumentException(String.format("Wrong inputs!"));
		}
		// create and add edge
		EdgeNode eg = new EdgeNode(from,to,roadName,roadType,length);
		edgeList.add(eg);
		// add edge to vertex
			for(GraphNode gn : nodeList) {
				if(gn.getGeoPoint().equals(from)) {
					GraphNode g = this.getVertex(to);
					gn.addEdge(g);
				}
			}
		//TODO: Implement this method in WEEK 3
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		LinkedList<GeographicPoint> result = new LinkedList<GeographicPoint>();		
		HashMap<GraphNode, GraphNode> parentMap = new HashMap<GraphNode, GraphNode>();
		boolean found = this.bfsSearch(start, goal, nodeSearched, parentMap);
		
			if(!found) {
				System.out.println("No path found.");
				return null;
			}
			// reconstruct the path
			//parentMap.forEach((key,value) -> System.out.println(key + " = " + value));
			
			GeographicPoint currPoint = goal;
			while (!currPoint.equals(start)) {
				result.addFirst(currPoint);
				GraphNode tempPoint = parentMap.get(this.getVertex(currPoint));
				currPoint =  tempPoint.getGeoPoint();
			}
			result.addFirst(start);
			
			//result = this.getPath(parentMap, start, goal);
			return result;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

	}
	private LinkedList<GeographicPoint> getPath(HashMap<GraphNode, GraphNode> parentMap,
													GeographicPoint start,
													GeographicPoint goal){
		LinkedList<GeographicPoint> result = new LinkedList<GeographicPoint>();
		GeographicPoint currPoint = goal;
		while (!currPoint.equals(start)) {

			result.addFirst(currPoint);
			GraphNode tempPoint = parentMap.get(this.getVertex(currPoint));
			currPoint =  tempPoint.getGeoPoint();
		}
		result.addFirst(start);
		return result;
	}
	
	private boolean bfsSearch(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,
			 					    HashMap<GraphNode, GraphNode> parentMap) {
		Queue<GraphNode> queue = new LinkedList<GraphNode>();
		Set<GraphNode> visited = new HashSet<GraphNode>();	
		GraphNode current = this.getVertex(start);
		queue.add(current);
			while(!queue.isEmpty()) {
				GraphNode curr = queue.remove();
				nodeSearched.accept(curr.getGeoPoint());
					if(curr.getGeoPoint().equals(goal)) {
						System.out.println(parentMap.size() + " visited by BFS.");
						return true;
						
					}
					List<GraphNode> currEdges = curr.getEdges();
					ListIterator<GraphNode> it = currEdges.listIterator(currEdges.size());
					while (it.hasPrevious()) {
						GraphNode next = it.previous();
						if (!visited.contains(next)) {
							visited.add(next);
							parentMap.put(next, curr);
							queue.add(next);

						}
					}
					
			}
		return false;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	private double distanceTo(GraphNode from, GraphNode to) {
		double lat1 = from.getGeoPoint().getX();
		double lon1 = from.getGeoPoint().getY();
		double lat2 = to.getGeoPoint().getX();
		double lon2 = to.getGeoPoint().getY();

	    	int R = 6373; // radius of the earth in kilometres
	    	double lat1rad = Math.toRadians(lat1);
	    	double lat2rad = Math.toRadians(lat2);
	    	double deltaLat = Math.toRadians(lat2-lat1);
	    	double deltaLon = Math.toRadians(lon2-lon1);

	    	double a = Math.sin(deltaLat/2) * Math.sin(deltaLat/2) +
	    	        Math.cos(lat1rad) * Math.cos(lat2rad) *
	    	        Math.sin(deltaLon/2) * Math.sin(deltaLon/2);
	    	double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

	    	double d = R * c;
	    	return d;
	}

	
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// initialization
		clearGraphNodes();
		LinkedList<GeographicPoint> result = new LinkedList<GeographicPoint>();
		HashMap<GraphNode, GraphNode> parentMap = new HashMap<GraphNode,GraphNode>();
		
		
		boolean found = false;
		// TODO: Implement this method in WEEK 4
		PriorityQueue<GraphNode> queue = new PriorityQueue<GraphNode>();
		
		//this is for helper
		Set<GraphNode> visited = new HashSet<GraphNode>();
		GraphNode current = this.getVertex(start);
		double distance = 0.0;
		current.setDistance(0.0);
		queue.add(current);
		
			while(!queue.isEmpty()) {
				GraphNode curr = queue.poll();

				nodeSearched.accept(curr.getGeoPoint());
					if(!visited.contains(curr)) {
						
						visited.add(curr);
						
					}
					if(curr.getGeoPoint().equals(goal)) {
						System.out.println(visited.size() +" visited by Dijkstra");
						found = true;
						break;
						
					}
					
					List<GraphNode> currEdges = curr.getEdges();
					ListIterator<GraphNode> it = currEdges.listIterator(currEdges.size());
					while (it.hasPrevious()) {
						GraphNode next = it.previous();
						if (!visited.contains(next)) {
							
							
							distance = curr.getDistance() + this.roadLength(curr, next);
								
							if(distance < next.getDistance()) {
								next.setDistance(distance);

							//visited.add(next);
							parentMap.put(next, curr);
							queue.add(next);
							}

						}
					}
			}
			
			if(!found) {
				System.out.println("No matches found.. Djkra");
				return result;
			}
			// find the path
			
			GeographicPoint currPoint = goal;
			while (!currPoint.equals(start)) {
				result.addFirst(currPoint);
				GraphNode tempPoint = parentMap.get(this.getVertex(currPoint));
				currPoint =  tempPoint.getGeoPoint();
				
			}
			result.addFirst(start);
			//result = this.getPath(parentMap, start, goal);
			return result;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		//initialize
		clearGraphNodes();
		HashMap<GraphNode,GraphNode> parentMap = new HashMap<GraphNode,GraphNode>();
		boolean found = false;
		
		HashSet<GraphNode> visited = new HashSet<GraphNode>();
		RoadComparor rd = new RoadComparor();
		PriorityQueue<GraphNode> queue = new PriorityQueue<GraphNode>(rd);
		GraphNode current = this.getVertex(start);
		current.setDistance(0.0);
		queue.add(current);
		
			while(!queue.isEmpty()) {

				nodeSearched.accept(current.getGeoPoint());
				
				current = queue.poll();
				if(!visited.contains(current)) {
					visited.add(current);
				}
					if(current.getGeoPoint().equals(goal)) {
						//System.out.println("Visited:  " + visited.size() + " by aStar.");
						//return this.getPath(parentMap, start, goal);
						found = true;
						break;
					
				}
			List<GraphNode> currNeigh = current.getEdges();	
				for(GraphNode next : currNeigh) {
					System.out.println(next);
					if(!visited.contains(next)) {
						
						double dist = this.roadLength(current, next) + current.getDistance();

						if( dist < next.getDistance()) {
							//System.out.println(next.getDistance());
							next.setDistance(dist);
							double heurestic = this.distanceTo(next, this.getVertex(goal));
							next.setDistanceToGoal(heurestic + dist);
							
							queue.add(next);
							parentMap.put(next, current);
						}

					}
				}
			}

			System.out.println(visited.size() + " visited by aStar.");
			System.out.println(parentMap);
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if(found) {
		return this.getPath(parentMap, start, goal);
		}
		else {
			List<GeographicPoint>  res = new LinkedList<GeographicPoint>();
			return res;
		}
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		System.out.println(simpleTestMap.getVertices());

		List<EdgeNode> temp = simpleTestMap.getAllEdges();
				System.out.println(temp);
		GeographicPoint testStart = new GeographicPoint(1.0,1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0,-1.0);
		List<GeographicPoint> out = simpleTestMap.bfs(testStart, testEnd);
			for(GeographicPoint p : out) {
				System.out.println(p);
			}
		*/
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println(testroute);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
