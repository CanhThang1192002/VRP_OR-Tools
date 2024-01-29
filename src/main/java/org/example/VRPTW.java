package org.example;

import com.google.ortools.Loader;
import com.google.ortools.sat.*;

import java.util.ArrayList;
import java.util.List;


public class VRPTW {
    final static DataModel data = new DataModel();
    private static final List<Edge> allEdge = new ArrayList<Edge>();

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
            if (e.from == i) {
                deltaPlus.add(e);
            }
        }
        return deltaPlus;
    }

    private static List<Edge> deltaMinus(int i) {
        List<Edge> deltaMinus = new ArrayList<Edge>();
        for (Edge e : allEdge) {
            if (e.to == i) {
                deltaMinus.add(e);
            }
        }
        return deltaMinus;
    }

    public static void main(String[] args) {
        Loader.loadNativeLibraries();
        int graphSize = data.timeMatrix.length;// |V|=n+2 với 0 và n+1 là 2 depot
        //Feasible vehicle routes then correspond
        //to paths starting at vertex 0 and ending at vertex n + 1.
        for (int i = 0; i < data.customerNumber; i++) {
            for (int j = 0; j < data.customerNumber; j++) {
                if (i != j) {
                    allEdge.add(new Edge(i, j, data.timeMatrix[i][j]));
                }
            }
        }

        CpModel model = new CpModel();
        IntVar[][][] x = new IntVar[graphSize][graphSize][data.vehicleNumber];

        for (int i = 0; i < graphSize; i++) {
            for (int j = 0; j < graphSize; j++) {
                for (int k = 0; k < data.vehicleNumber; k++) {
                    x[i][j][k] = model.newIntVar(0, 1, "x[" + i + "," + j + "," + k + "]");
                }
            }
        }
        List<IntVar> Xijk = new ArrayList<>();
        List<Long> Cij = new ArrayList<>();
        for (int i = 0; i < data.timeMatrix.length; i++) {
            for (int j = 0; j < data.timeMatrix.length; j++) {
                if (i == 0 && j == graphSize - 1 || i == graphSize - 1 && j == 0) continue;
                for (int k = 0; k < data.vehicleNumber; k++) {
                    Xijk.add(x[i][j][k]);
                    Cij.add(data.timeMatrix[i][j]);
                }
            }
        }
        IntVar[] XijkArray = new IntVar[Xijk.size()];
        for (int i = 0; i < Xijk.size(); i++) {
            XijkArray[i] = Xijk.get(i);
        }
        long[] CijArray = new long[Cij.size()];
        for (int i = 0; i < Cij.size(); i++) {
            CijArray[i] = Cij.get(i);
        }
        model.maximize(LinearExpr.weightedSum(XijkArray, CijArray));


        IntVar[][] w = new IntVar[data.vehicleNumber][graphSize];
        // w_i_k là thời gian bắt đầu phục vụ khách hàng i bởi xe k
        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int i = 0; i < graphSize; i++) {
                    w[k][i] = model.newIntVar(data.earliestArrivalTime[i],data.latestArrivalTime[i], "w[" + k + "," + i + "]");
            }
        }

        //Constrains
//        ràng buộc 1: sum(sum(x_i_j_k)) = 1 với mọi i thuộc N, k thuộc K, j thuộc delta+(i)
        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int i = 0; i < graphSize; i++) {
                IntVar[] tmp = new IntVar[graphSize];
                for (int j = 0; j < graphSize; j++) {
                    tmp[j] = x[i][j][k];
                }
                model.addEquality(LinearExpr.sum(tmp), 1);
            }
        }
        // ràng buộc 2: sum(j thuộc delta+(0), k thuộc K, x_0_j_k) = 1
        for (int k = 0; k < data.vehicleNumber; k++) {
            IntVar[] tmp = new IntVar[graphSize];
            for (int j = 0; j < graphSize; j++) {
                tmp[j] = x[0][j][k];
            }
            model.addEquality(LinearExpr.sum(tmp), 1);
        }

        // ràng buộc 3: sum(j thuộc delta-(n+1), k thuộc K, x_j_n+1_k) = 1
        for (int k = 0; k < data.vehicleNumber; k++) {
            IntVar[] tmp = new IntVar[graphSize];
            for (int j = 0; j < graphSize; j++) {
                tmp[j] = x[j][graphSize - 1][k];
            }
            model.addEquality(LinearExpr.sum(tmp), 1);
        }


        // ràng buộc 4: sum(i thuộc delta - (j), k thuộc K, x_i_j_k) - sum(i thuộc delta + (j), k thuộc K, x_j_i_k) = 0 với mọi j thuộc N
        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int j = 1; j < graphSize - 1; j++) {
                List<IntVar> Xij = new ArrayList<>();
                List<IntVar> Xji = new ArrayList<>();
                for (int i = 0; i < graphSize; i++) {
                    if (i == j) continue;
                    if (i == 0 && j == graphSize - 1) continue;
                    Xij.add(x[i][j][k]);
                    Xji.add(x[j][i][k]);
                }
                IntVar[] XijArray = new IntVar[Xij.size()];
                IntVar[] XjiArray = new IntVar[Xji.size()];
                for (int i = 0; i < Xij.size(); i++) {
                    XijArray[i] = Xij.get(i);
                    XjiArray[i] = Xji.get(i);
                }
                model.addEquality(LinearExpr.sum(XijArray), LinearExpr.sum(XjiArray));
            }
        }


        for (int k = 0; k < data.vehicleNumber; k++) {
            List<IntVar> XiList = new ArrayList<>();
            List<Long> demandList = new ArrayList<>();
            for (int i = 1; i < graphSize; i++) {
                for (int j = 0; j < graphSize; j++) {
                    if (i == j) continue;
                    if (i == graphSize - 1 && j == 0) continue;
                    XiList.add(x[i][j][k]);
                    demandList.add((long) data.demands[i]);
                }
            }
            IntVar[] Xi = new IntVar[XiList.size()];
            long[] demand = new long[demandList.size()];
            for (int i = 0; i < XiList.size(); i++) {
                Xi[i] = XiList.get(i);
                demand[i] = demandList.get(i);
            }
            model.addLessOrEqual(LinearExpr.weightedSum(Xi, demand), data.vehicleCapacity);
        }

        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int i = 0; i < graphSize; i++) {
                for (int j = 0; j < graphSize; j++) {
                    if (i == 0 && j == graphSize - 1 || i == graphSize - 1 && j == 0) continue;
                    long Mij = Math.max(0, data.latestArrivalTime[i] + data.timeMatrix[i][j] - data.earliestArrivalTime[i]);
                    IntVar[] tmp = new IntVar[3];
                    tmp[0] = w[k][i];
                    tmp[1] = w[k][j];
                    tmp[2] = x[i][j][k];

                    long tmp2[] = new long[3];
	    			tmp2[0] = 1;
	    			tmp2[1] = -1;
	    			tmp2[2] = Mij;

                    model.addLessOrEqual(LinearExpr.weightedSum(tmp, tmp2), -data.servingTime[i] - data.timeMatrix[i][j] + Mij);

                }
            }
        }

        //Solve

        CpSolver solver = new CpSolver();
        CpSolverStatus status = solver.solve(model);

        if (status == CpSolverStatus.OPTIMAL || status == CpSolverStatus.FEASIBLE) {
            System.out.printf("Objective function: %.2f%n", solver.objectiveValue());
            for (int k = 0; k < data.vehicleNumber; k++) {
                List<Integer> route = new ArrayList<>();
                int post = 0;
                System.out.println("Route " + k + " :");
                route.add(post);
                for (int j = 0; j < graphSize; j++) {
                    if (solver.value(x[post][j][k]) == 1) {
                        post = j;
                        route.add(post);
                        break;
                    }
                }
                while (post != graphSize - 1) {
                    for (int j = 0; j < graphSize; j++) {
                        if (solver.value(x[post][j][k]) == 1) {
                            post = j;
                            route.add(post);
                        }
                    }
                }
                long demand = 0;
                System.out.println("<ID: A, D>");
                for (int i = 0; i < route.size(); i++) {
                    System.out.print("(" + route.get(i) + ": " + solver.value(w[k][route.get(i)]) + ", " + data.demands[route.get(i)] + ") --> ");
                    demand += data.demands[route.get(i)];
                }
                System.out.println();
                System.out.println("Demand = " + demand);

            }
        } else {
            System.out.println("No solution");

        }
        System.out.println("Statistics");
        System.out.printf("  conflicts: %d%n", solver.numConflicts());
        System.out.printf("  branches : %d%n", solver.numBranches());
        System.out.printf("  wall time: %.2f s%n", solver.wallTime());


    }


    static class DataModel {
        public final long[][] timeMatrix = {
                {0, 548, 776, 696, 582, 274, 0},
                {548, 0, 684, 308, 194, 502, 548},
                {776, 684, 0, 992, 878, 502, 776},
                {696, 308, 992, 0, 114, 650, 696},
                {582, 194, 878, 114, 0, 536, 582},
                {274, 502, 502, 650, 536, 0, 274},
                {0, 548, 776, 696, 582, 274, 0},
        };
        public final long[] demands = {0, 1, 1, 2, 4, 2, 0};
        public final long[] earliestArrivalTime = {0, 187, 276, 123, 516, 511, 6};
        public final long[] latestArrivalTime = {1000, 1000, 2000, 800, 1244, 1123, 5115};
        public final long[] servingTime = {0, 500, 500, 100, 580, 1220, 0};

        public final long vehicleCapacity = 100;
        public final int vehicleNumber = 3;
        public final int customerNumber = timeMatrix.length - 1;

        public static boolean isNotTravelable(int i, int j, int graphSize) {
            if (i == j) return true;
            return (i == 0 && j == graphSize - 1 || i == graphSize-1 && j == 0);
        }

    }

}