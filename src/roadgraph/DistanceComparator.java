package roadgraph;

import java.util.Comparator;

public class DistanceComparator implements Comparator<Distance>{

	@Override
	public int compare(Distance x, Distance y) {
		// TODO Auto-generated method stub
		return Double.valueOf(x.getDis()).compareTo(y.getDis());
	}
	
	
}