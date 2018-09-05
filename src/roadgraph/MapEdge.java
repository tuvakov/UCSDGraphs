package roadgraph;
import geography.GeographicPoint;
/*
 * This class keeps tracks of edge between GeographicPoints
 */
public class MapEdge implements Comparable<MapEdge>{

	// Defining member variables
	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	private String roadName;
	private String roadType;
	private double length;
	
	// Initializing all member variables in constructor;
	public MapEdge(GeographicPoint start, GeographicPoint end,
			String roadName, String roadType, double length){
		
		// Checking if start or end point were passed properly
		if(start == null || end == null){
			throw new NullPointerException();
		}
		
		this.startPoint = start;
		this.endPoint = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		
	}
	
	
	// Getter methods for the member variables
	public GeographicPoint getStartPoint(){	return this.startPoint; }
	
	public GeographicPoint getEndPoint(){return this.endPoint; }
	
	public double getLength() {return length;}
	
	public String getRoadType(){return this.roadType;}
	
	@Override
	public String toString(){
		return endPoint.toString();
	}


	@Override
	public int compareTo(MapEdge arg0) {
		// TODO Auto-generated method stub
		return Double.valueOf(this.getLength()).compareTo(arg0.getLength());
	}
}
