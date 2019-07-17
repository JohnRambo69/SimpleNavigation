package roadgraph;

import java.util.Comparator;

public class RoadComparor implements Comparator<GraphNode> {

		public int compare(GraphNode x,GraphNode y) {
			if(x.getDistanceToGoal() > y.getDistanceToGoal()) {
				return 1;
			}
			else if(x.getDistanceToGoal() < y.getDistanceToGoal()){
				return -1;
			}
			else {
				return 0;
			}
		}
}

