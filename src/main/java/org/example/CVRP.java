package org.example;

import com.google.ortools.Loader;
import com.google.ortools.sat.*;

import java.util.ArrayList;
import java.util.List;


public class CVRP {

    public static DataModel data = new DataModel();
    public static List<Edge> allEdge = new ArrayList<Edge>();

    private static void createAllEdge() {
        for (int i = 0; i < data.customerNumber; i++) {
            for (int j = 0; j < data.customerNumber; j++) {
                if(i!=j)
                    allEdge.add(new Edge(i, j, data.distanceMatrix[i][j]));
            }
        }
    }

    private static List<Edge> delta(int i) {
        List<Edge> delta = new ArrayList<Edge>();
        for (Edge e : allEdge) {
            if (e.from == i || e.to == i) {
                delta.add(e);
            }
        }
        return delta;
    }

    private static List<Edge> deltaPlus(int i) {
        List<Edge> deltaPlus = new ArrayList<Edge>();
        for (Edge e : allEdge) {
            if (e.from == i && e.to != data.depot) {
                deltaPlus.add(e);
            }
        }
        return deltaPlus;
    }

    private static List<Edge> deltaMinus(int i) {
        List<Edge> deltaMinus = new ArrayList<Edge>();
        for (Edge e : allEdge) {
            if (e.to == i && e.from != data.depot) {
                deltaMinus.add(e);
            }
        }
        return deltaMinus;
    }

    private static long R(long[] demand, long capacity) {
        long R = 0;
        for (int i = 0; i < demand.length; i++) {
            R += demand[i];
        }
        return R / capacity;
    }

    public static void main(String[] args) {
        Loader.loadNativeLibraries();
        createAllEdge();
        CpModel model = new CpModel();
        IntVar[][] c = new IntVar[data.customerNumber][data.customerNumber];
        for (int i = 0; i < data.customerNumber; i++) {
            for (int j = 0; j < data.customerNumber; j++) {
                // c[i][j] là chi phí đi từ i đến j = distanceMatrix[i][j]
                c[i][j] = model.newIntVar(0, 1000000, "c[" + i + "," + j + "]");
                model.addEquality(c[i][j], data.distanceMatrix[i][j]);
            }
        }

        // x[i][j] number of times edge e is traversed in the
        //solution
        IntVar[][] x = new IntVar[data.customerNumber][data.customerNumber];
        for (int i = 0; i < data.customerNumber; i++) {
            for (int j = 0; j < data.customerNumber; j++) {
                x[i][j] = model.newIntVar(0, 1000000, "x[" + i + "," + j + "]");
            }
        }


        for (int i = 0; i < data.customerNumber; i++) {
            model.addEquality(x[i][i], 0);
        }

        //sum(e∈δ(i) xe = 2)
        for (int i = 1; i < data.customerNumber; i++) {
            List<Edge> delta = delta(i);
            IntVar[] tmp = new IntVar[delta.size()];
            for (int j = 0; j < delta.size(); j++) {
                tmp[j] = x[delta.get(j).from][delta.get(j).to];
            }
            model.addEquality(LinearExpr.sum(tmp), 2);
        }


        //sum(e∈δ(0) xe = 2*m)
        IntVar[] tmp0 = new IntVar[data.customerNumber];
        System.arraycopy(x[0], 0, tmp0, 0, data.customerNumber);
        model.addEquality(LinearExpr.sum(tmp0), 2 * data.vehicleNumber);

        for (int i = 1; i < data.customerNumber; i++) {
            List<Edge> delta = delta(i);
            IntVar[] tmp = new IntVar[delta.size()];
            for (int j = 0; j < delta.size(); j++) {
                tmp[j] = x[delta.get(j).from][delta.get(j).to];
            }
            model.addGreaterOrEqual(LinearExpr.sum(tmp), 2 * R(data.demands, data.vehicleCapacity));
        }

        // mọi cạnh i->j chỉ được đi qua 1 lần nếu i,j khác 0 và n+1
        for (int i = 1; i < data.customerNumber; i++) {
            for (int j = 1; j < data.customerNumber; j++) {
                if (i != j) {
                    model.addLessThan(x[i][j], 2);
                }
            }
        }

        // mục tiêu là tối thiểu tổng chi phí c_ij*x_ij
        IntVar obj = model.newIntVar(0, 1000000, "obj");
        IntVar[] tmp = new IntVar[data.customerNumber * data.customerNumber];
        for (int i = 0; i < data.customerNumber; i++) {
            for (int j = 0; j < data.customerNumber; j++) {
                tmp[i * data.customerNumber + j] = model.newIntVar(0, 1000000, "tmp[" + i + "," + j + "]");
                model.addMultiplicationEquality(tmp[i * data.customerNumber + j], c[i][j], x[i][j]);
            }
        }
        model.addEquality(LinearExpr.sum(tmp), obj);
        model.minimize(obj);

        CpSolver solver = new CpSolver();
        CpSolverStatus status = solver.solve(model);
        if (status == CpSolverStatus.OPTIMAL) {
            System.out.println("obj = " + solver.objectiveValue());
            for (int i = 0; i < data.customerNumber; i++) {
                for (int j = 0; j < data.customerNumber; j++) {
                    if (solver.value(x[i][j]) != 0) {
                        System.out.println("x[" + i + "," + j + "] = " + solver.value(x[i][j]));
                    }
                }
            }
        } else {
            System.out.println("No solution");
        }
        System.out.println("Statistics");
	    System.out.printf("  conflicts: %d%n", solver.numConflicts());
	    System.out.printf("  branches : %d%n", solver.numBranches());
	    System.out.printf("  wall time: %f s%n", solver.wallTime());
    }

    static class DataModel {
        public final long[][] distanceMatrix = {
	            {0, 548, 776, 696, 582, 274, 0},
	            {548, 0, 684, 308, 194, 502, 548},
	            {776, 684, 0, 992, 878, 502, 776},
	            {696, 308, 992, 0, 114, 650, 696},
	            {582, 194, 878, 114, 0, 536, 582},
	            {274, 502, 502, 650, 536, 0, 274},
	            {0, 548, 776, 696, 582, 274, 0},};

        public final int vehicleNumber = 2;
        public final int depot = 0;
        public final long[] demands = {0, 1, 1, 2, 4, 2, 4, 6, 6};
        public final long vehicleCapacity = 15;
        public final int customerNumber = distanceMatrix.length - 1;

    }
}
