package org.example;

import com.google.ortools.Loader;
import com.google.ortools.sat.*;

import java.util.LinkedList;

public class CVRP2 {
	static DataModel data = new DataModel();

    public static void main(String[] args) {
		Loader.loadNativeLibraries();

		CpModel model = new CpModel();




	}


	static class DataModel {
		public final long[][] distanceMatrix = {
				{0, 548, 776, 696, 582, 274, 502, 194, 308},
				{548, 0, 684, 308, 194, 502, 730, 354, 696},
				{776, 684, 0, 992, 878, 502, 274, 810, 468},
				{696, 308, 992, 0, 114, 650, 878, 502, 844},
				{582, 194, 878, 114, 0, 536, 764, 388, 730},
				{274, 502, 502, 650, 536, 0, 228, 308, 194},
				{502, 730, 274, 878, 764, 228, 0, 536, 194},
				{194, 354, 810, 502, 388, 308, 536, 0, 342},
				{308, 696, 468, 844, 730, 194, 194, 342, 0},

		};
		public final long[] demands = {0, 1, 1, 2, 4, 2, 4, 6, 6};
		public final long vehicleCapacity = 15;
		public final int vehicleNumber = 5;
		public final int depot = 0;
		public final int customerNumber = distanceMatrix.length - 1;
	}


}
