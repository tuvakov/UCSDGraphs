package examples;
import java.util.*;
public class Graph {

	private int mNumEdges;
	private int mNumVertices;
	private Map<Integer, HashSet<Integer>> AdjList;
	
	
	public Graph(){
		this.mNumEdges = 0;
		this.mNumVertices = 0;
		AdjList = new HashMap<Integer, HashSet<Integer>>();
	}
	
	
	public void addVertex(Integer vertex){
		if(!AdjList.containsKey(vertex)){
			AdjList.put(vertex, new HashSet<Integer>());
			mNumVertices++;
		}
	}
	
	public void addEdge(Integer v, Integer w){
		(AdjList.get(v)).add(w);
		mNumEdges++;
	}
	
	
	public List<Integer> getNeighboors(Integer vertex){
		
		List<Integer> list = new ArrayList<Integer>(AdjList.get(vertex));
		
		for(Integer i: AdjList.get(vertex)){
			list.addAll(AdjList.get(i));
		}
	
 		return list;
	}
	
	public static void main(String args[]){
		
		Graph graph = new Graph();
		
		graph.addVertex(0);
		graph.addVertex(1);
		graph.addVertex(2);
		graph.addVertex(3);
	
		
		graph.addEdge(0, 1);
		graph.addEdge(0, 2);
		graph.addEdge(1, 3);
		graph.addEdge(2, 1);
		graph.addEdge(2, 3);

		
		
		System.out.println(graph.getNeighboors(0));
		
	}
	
	
	public class Node<E>{
		
		private E data;
		
		public Node(E data){
			this.data = data;
		}
	}
}


