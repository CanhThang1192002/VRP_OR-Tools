//package org.example;
//
//import com.google.ortools.Loader;
//import com.google.ortools.sat.CpModel;
//import com.google.ortools.sat.CpSolver;
//import com.google.ortools.sat.IntVar;
//
//public class VRPSolver {
//    public static void main(String[] args) {
//        // Load the OR-Tools library
//        Loader.loadNativeLibraries();
//
//        // Define the VRP data
//        int numLocations = 6;  // Number of locations (including depot)
//        int numVehicles = 2;   // Number of vehicles
//        int depot = 0;         // Index of the depot
//        int[] demands = {0, 10, 5, 8, 3, 6};  // Demand at each location
//        int[][] distances = {
//            {0, 10, 20, 15, 30, 25},
//            {10, 0, 25, 30, 20, 15},
//            {20, 25, 0, 10, 40, 30},
//            {15, 30, 10, 0, 35, 20},
//            {30, 20, 40, 35, 0, 10},
//            {25, 15, 30, 20, 10, 0}
//        };  // Distance matrix
//
//        // Create a CP model
//        CpModel model = new CpModel();
//
//        // Define variables
//        int numNodes = numLocations;
//        int numVehiclesVar = numVehicles;
//        int vehicleCapacity = 15;  // Vehicle capacity
//
//        // Vehicle routes
//        IntVar[][] x = new IntVar[numNodes][numNodes];
//        for (int i = 0; i < numNodes; i++) {
//            for (int j = 0; j < numNodes; j++) {
//                x[i][j] = model.newBoolVar("x_" + i + "_" + j);
//            }
//        }
//
//        // Define objective variable (total distance)
//        IntVar totalDistance = model.newIntVar(0, Integer.MAX_VALUE, "totalDistance");
//
//        // Calculate total distance and add it as an objective
//        for (int i = 0; i < numNodes; i++) {
//            for (int j = 0; j < numNodes; j++) {
//                IntVar prodVar = model.newIntVar(0, Integer.MAX_VALUE, "prod_" + i + "_" + j);
//                model.addMultiplication(x[i][j], distances[i][j], prodVar);
//                model.addEquality(totalDistance, model.newIntVar(0, Integer.MAX_VALUE, "totalDist"), prodVar);
//            }
//        }
//
//        // Constraints (similar to previous code)
//
//        // Solve the model
//        CpSolver solver = new CpSolver();
//        CpSolverStatus status = solver.solve(model);
//
//        // Print the solution (similar to previous code)
//    }
//}
//
//
