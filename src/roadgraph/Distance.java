package roadgraph;

import geography.GeographicPoint;

public class Distance {

	private GeographicPoint node;
	private double dis;
	
	public Distance(GeographicPoint node, double dis){
		this.node = node;
		this.dis = dis;
	}
	
	
	public GeographicPoint getNode(){ return this.node; }
	
	public double getDis(){ return this.dis; }
	
	@Override
	public boolean equals(Object o){
		
		boolean equal = false;
		
		if(o instanceof Distance){ 	
			equal = ((Distance) o).getDis() == this.getDis() && ((Distance) o).getNode().equals(this.getNode());
		}
		
		return equal;
	}
	
	@Override
	public String toString(){
		return node.toString() + " " + dis;
	}
	
}
