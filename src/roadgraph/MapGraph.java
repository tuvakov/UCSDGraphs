/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import com.sun.javafx.collections.MappingChange.Map;

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
	
	// A hash set that hold all the vertices
	Set<GeographicPoint> vertices;
	HashSet<MapEdge> edges;
	HashMap<GeographicPoint, HashSet<MapEdge>> adjList;

	/** 
	 * Create a new empty MapGraph 
	*/
	
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		
		// Instantiating member variables
		vertices = new HashSet<GeographicPoint>();
		edges = new HashSet<MapEdge>();
		adjList = new HashMap<GeographicPoint, HashSet<MapEdge>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
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
		// TODO: Implement this method in WEEK 3
		
		// Checking if the location is null or already in the graph
		if(location == null || vertices.contains(location))
			return false;
		
		if(vertices.add(location)){
			adjList.put(location, new HashSet<MapEdge>());
			return true;
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

		//TODO: Implement this method in WEEK 3
		
		// Supplying preconditions
		if (!vertices.contains(from) || !vertices.contains(to) || to == null || from == null || length < 0)
			throw new IllegalArgumentException();
		
		// Adding edge
		MapEdge mapEdge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(mapEdge);
		adjList.get(from).add(mapEdge);
		
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
	
	// Set path
	private List<GeographicPoint> setPath(HashMap<GeographicPoint, GeographicPoint> parentMap, 
			GeographicPoint start, GeographicPoint goal){
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		GeographicPoint current = goal;
		
		while(current != start){
			path.addFirst(current);
			current = parentMap.get(current);
		}
		
		path.addFirst(start);
		
		return path;
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
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
				
	 	Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
        HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        GeographicPoint current;
        HashSet<MapEdge> neighbors;
        
        queue.add(start);
        visited.add(start);
        
        while(!queue.isEmpty()){
        	
        	current = queue.poll();
        	
        	if(current.equals(goal)){
        		return setPath(parentMap, start, goal);
        	}
        	
        	neighbors = adjList.get(current);
        	
        	for(MapEdge me: neighbors){
        		if(!visited.contains(me.getEndPoint())){
        	      	visited.add(me.getEndPoint());
        			queue.add(me.getEndPoint());
                	nodeSearched.accept(current);
        			parentMap.put(me.getEndPoint(), current);
        		
        		}
        	}
        }

        return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
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
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		// nodeSearched.accept(next.getLocation());
		
		
		// Initialization of required variables
		Comparator<Distance> comparator = new DistanceComparator();
		PriorityQueue<Distance> queue = new PriorityQueue<Distance>(10, comparator);
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, Double> distances = new HashMap<GeographicPoint, Double>();
		
		double distance = 0;
		int count = 0;
		GeographicPoint current;
		
		distances.put(start, 0.0);
		queue.add(new Distance(start, 0));
		
		while(!queue.isEmpty()){
			
			distance = queue.peek().getDis();
			current = queue.poll().getNode();
			count++;
			//System.out.println(current);
			if(!visited.contains(current)){
				visited.add(current);
				if(current.equals(goal)){
					System.out.println("Dijkstra: " + count);
					System.out.println("Dijkstra total distance: " + distances.get(current));
					return setPath(parentMap, start, goal);
				}
	
				for(MapEdge n:adjList.get(current)){
					if(!visited.contains(n.getEndPoint())){
						
						if(distances.containsKey(n.getEndPoint()) && distances.get(n.getEndPoint()) > (distance + n.getLength())){
							distances.put(n.getEndPoint(), (distance + n.getLength()));
							parentMap.put(n.getEndPoint(), current);
							nodeSearched.accept(current);
							queue.add(new Distance(n.getEndPoint(), (distance + n.getLength())));
						}else if(!distances.containsKey(n.getEndPoint())){
							distances.put(n.getEndPoint(), (distance + n.getLength()));
							parentMap.put(n.getEndPoint(), current);
							nodeSearched.accept(current);
							queue.add(new Distance(n.getEndPoint(), (distance + n.getLength())));
						}
					}	
				}
			}
		}
		
		
		
		System.out.println("HR");
		return null;
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
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		// Initialization of required variables
		Comparator<Distance> comparator = new DistanceComparator();
		PriorityQueue<Distance> queue = new PriorityQueue<Distance>(10, comparator);
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, Double> distances = new HashMap<GeographicPoint, Double>();
		
		double distance = 0;
		GeographicPoint current;
		
		distances.put(start, 0.0);
		queue.add(new Distance(start, 0));
				
		while(!queue.isEmpty()){
					
		distance = queue.peek().getDis();
		current = queue.poll().getNode();
		if(!visited.contains(current)){
			visited.add(current);
			if(current.equals(goal)){
				System.out.println("A*: " + visited.size());
				System.out.println("A* route total distance: " + distances.get(current));
				return setPath(parentMap, start, goal);
			}
			for(MapEdge n:adjList.get(current)){
				if(!visited.contains(n.getEndPoint())){
					//System.out.println(n.getEndPoint().distance(goal));
					double d = n.getEndPoint().distance(goal) + distances.get(current) + n.getLength();
					
					/*
					 * Bu aradaki if durma nedeni
					 * sadece yeni uzaklik daha onceden var olan uzakliktan kisa ise update yapilmasi icin
					 */
					
					if(distances.containsKey(n.getEndPoint()) && distances.get(n.getEndPoint()) > d){
								
						distances.put(n.getEndPoint(), distances.get(current) + n.getLength());
						parentMap.put(n.getEndPoint(), current);
						nodeSearched.accept(current);
						queue.add(new Distance(n.getEndPoint(), d));
					}else if(!distances.containsKey(n.getEndPoint())){
						distances.put(n.getEndPoint(), distances.get(current) + n.getLength());
						parentMap.put(n.getEndPoint(), current);
						nodeSearched.accept(current);
						queue.add(new Distance(n.getEndPoint(), d));
						}
					}	
				}
			}
		//System.out.println(queue);
		}		
		
		return null;
	}

	
	public void printGraph(){
		
		System.out.println("Vertex size: " + this.getNumVertices());
		System.out.println("Edge size: "+ this.getNumEdges());

		for(GeographicPoint m: vertices){
			System.out.println(m.toString() + " : " + adjList.get(m));
		}
	}
	
	
	public static void main(String[] args)
	{
		
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/san_diego.map", firstMap);
		System.out.println("DONE.");
		
		//firstMap.printGraph();
		
		for(MapEdge e: firstMap.edges){
			
			System.out.println(e.getRoadType());
		}
	
		/*
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		
		List<GeographicPoint> bfs = firstMap.bfs(start, end);
		List<GeographicPoint> dijkstra = firstMap.dijkstra(start, end); 
		System.out.println("-----");
		List<GeographicPoint> astar = firstMap.aStarSearch(start, end);

		
		System.out.println("");
		System.out.println("BFS: " + bfs);
		System.out.println("Dijkstra: " + dijkstra);
		System.out.println("A*: " + astar);
		
		//System.out.println(firstMap.adjList.get(new GeographicPoint(4.0, 0.0)));
	
		System.out.println(start.distance(new GeographicPoint(6.5, 0)));
		
		// You can use this method for testing.  
		
		*/
		
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
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);
		
		//System.out.println(testroute);
		
		
		//List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		*/
		
		/*
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		//System.out.println("---");
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		//System.out.println("Dijkstra: " + testroute);
		//System.out.println("A*:" + testroute2);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
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
