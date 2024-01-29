package org.example;


import com.google.ortools.Loader;
import com.google.ortools.sat.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;


public class Test {
    public static void main(String[] args) {
        Loader.loadNativeLibraries();
        final DataModel data = new DataModel();
        int graphSize = data.timeMatrix.length;
        CpModel model = new CpModel();
        IntVar[][][] X = new IntVar[graphSize][][];
        for (int i = 0; i < graphSize; i++) {
            X[i] = new IntVar[graphSize][];
            for (int j = 0; j < graphSize; j++) {
                X[i][j] = new IntVar[data.vehicleNumber];
                for (int k = 0; k < data.vehicleNumber; k++) {
                    X[i][j][k] = model.newIntVar(0, 1, ""); // Binary variable for the state of the arc.
                    // X[i][j][k] = 1 if vehicle k travel from i to j
                }
            }
        }
        //minimize sum(Xijk * Cij) with Cij is timeMatrix and Xijk is binary variable
        List<IntVar> ListXijk = new ArrayList<>();
        List<Long> ListCij = new ArrayList<Long>();
        for (int i = 0; i < graphSize; i++) {
            for (int j = 0; j < graphSize; j++) {
                if (DataModel.isNotTravelable(i, j, graphSize)) continue;
                for (int k = 0; k < data.vehicleNumber; k++) {
                    ListXijk.add(X[i][j][k]);
                    ListCij.add(data.timeMatrix[i][j]);
                }
            }
        }
        IntVar[] Xijk = new IntVar[ListXijk.size()];
        for (int i = 0; i < ListXijk.size(); i++) {
            Xijk[i] = ListXijk.get(i);
        }
        long[] Cij = new long[ListCij.size()];
        for (int i = 0; i < ListXijk.size(); i++) {
            Cij[i] = ListCij.get(i);
        }

        model.minimize(LinearExpr.weightedSum(Xijk, Cij));

        // sum(Xijk) = 1 with k = 1..data.vehicleNumber, j is delta+(i)
        for (int i = 1; i < graphSize - 1; i++) {
            List<IntVar> XjkList = new ArrayList<>();
            for (int j = 0; j < graphSize; j++) {
                if (DataModel.isNotTravelable(i, j, graphSize)) continue;
                XjkList.addAll(Arrays.asList(X[i][j]).subList(0, data.vehicleNumber));
            }
            IntVar[] Xjk = new IntVar[XjkList.size()];
            for (int index = 0; index < XjkList.size(); index++) {
                Xjk[index] = XjkList.get(index);
            }

            model.addEquality(LinearExpr.sum(Xjk), 1);
        }

        //sum(X0jk) = 1 with k = 1..data.vehicleNumber, j is delta+(0)
        for (int k = 0; k < data.vehicleNumber; k++) {
            List<IntVar> XjList = new ArrayList<>();
            for (int j = 1; j < graphSize; j++) {
                if (DataModel.isNotTravelable(0, j, graphSize)) continue;
                XjList.add(X[0][j][k]);
            }
            IntVar[] Xj = new IntVar[XjList.size()];
            for (int index = 0; index < XjList.size(); index++) {
                Xj[index] = XjList.get(index);
            }
            model.addEquality(LinearExpr.sum(Xj), 1);
        }

        //sum(Xij) = sum(Xji) with k = 1..data.vehicleNumber, i->j is an arc
        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int j = 1; j < graphSize - 1; j++) {
                List<IntVar> XijList = new ArrayList<>();
                List<IntVar> XjiList = new ArrayList<IntVar>();
                for (int i = 0; i < graphSize; i++) {
                    if (DataModel.isNotTravelable(i, j, graphSize)) continue;
                    XijList.add(X[i][j][k]);
                    XjiList.add(X[j][i][k]);
                }
                IntVar[] Xij = new IntVar[XijList.size()];
                IntVar[] Xji = new IntVar[XjiList.size()];
                for (int index = 0; index < XijList.size(); index++) {
                    Xij[index] = XijList.get(index);
                    Xji[index] = XjiList.get(index);
                }
                model.addEquality(LinearExpr.sum(Xij), LinearExpr.sum(Xji));
            }
        }

        //sum(xi_n+1_k)=1 with k = 1..data.vehicleNumber and i = delta-(n+1)
        for (int k = 0; k < data.vehicleNumber; k++) {
            List<IntVar> XiList = new ArrayList<>();
            for (int i = 1; i < graphSize - 1; i++) {
                XiList.add(X[i][graphSize - 1][k]);
            }
            IntVar[] Xi = new IntVar[XiList.size()];
            for (int index = 0; index < XiList.size(); index++) {
                Xi[index] = XiList.get(index);
            }
            model.addEquality(LinearExpr.sum(Xi), 1);
        }

        //wik is arrival time at node i with vehicle k

        // earlistArrivalTime[i] <= wik <= latestArrivalTime[i] with i = 1..data.customerNumber, k = 1..data.vehicleNumber
        IntVar[][] W = new IntVar[data.vehicleNumber][graphSize];
        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int i = 0; i < graphSize; i++) {
                W[k][i] = model.newIntVar(data.earliestArrivalTime[i], data.latestArrivalTime[i], "");
            }
        }

        //sum(qi * Xijk) <= data.vehicleCapacity with k = 1..data.vehicleNumber, i = 1..data.customerNumber, j = delta+(i)
        for (int k = 0; k < data.vehicleNumber; k++) {
            List<IntVar> listxijk = new ArrayList<>();
            List<Long> listDemand = new ArrayList<>();
            for (int i = 1; i < graphSize - 1; i++) {
                for (int j = 0; j < graphSize; j++) {
                    if (DataModel.isNotTravelable(i, j, graphSize)) continue;
                    listxijk.add(X[i][j][k]);
                    listDemand.add(data.demands[i]);
                }
            }
            IntVar[] lXijk = new IntVar[listxijk.size()];
            long[] lQ = new long[listDemand.size()];
            for (int index = 0; index < listxijk.size(); index++) {
                lXijk[index] = listxijk.get(index);
                lQ[index] = listDemand.get(index);
            }
            model.addLessOrEqual(LinearExpr.weightedSum(lXijk, lQ), data.vehicleCapacity);
        }

        //wik + servingTime[i] + timeMatrix[i][j] <= wjk + Mij * (1 - Xijk) with k = 1..data.vehicleNumber, i = 1..data.customerNumber, j = delta+(i)
        for (int k = 0; k < data.vehicleNumber; k++) {
            for (int i = 0; i < graphSize; i++) {
                for (int j = 0; j < graphSize; j++) {
                    if (DataModel.isNotTravelable(i, j, graphSize)) continue;
                    long Mij = Math.max(0, data.latestArrivalTime[i] + data.servingTime[i] + data.timeMatrix[i][j] - data.earliestArrivalTime[j]);
                    IntVar[] tmp = new IntVar[3];
                    tmp[0] = W[k][i];
                    tmp[1] = W[k][j];
                    tmp[2] = X[i][j][k];

                    long[] tmp2 = new long[3];
                    tmp2[0] = 1;
                    tmp2[1] = -1;
                    tmp2[2] = Mij;

                    model.addLessOrEqual(LinearExpr.weightedSum(tmp, tmp2), -data.servingTime[i] - data.timeMatrix[i][j] + Mij);
                }
            }
        }

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
                    if (solver.value(X[post][j][k]) == 1) {
                        post = j;
                        route.add(post);
                        break;
                    }
                }
                while (post != graphSize - 1) {
                    for (int j = 0; j < graphSize; j++) {
                        if (solver.value(X[post][j][k]) == 1) {
                            post = j;
                            route.add(post);
                        }
                    }
                }
                long demand = 0;
                System.out.println("<ID: A, D>");
                for (int i = 0; i < route.size(); i++) {
                    System.out.print("(" + route.get(i) + ": " + solver.value(W[k][route.get(i)]) + ", " + data.demands[route.get(i)] + ") --> ");
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

        public final long vehicleCapacity = 10;
        public final int vehicleNumber = 5;

        public static boolean isNotTravelable(int i, int j, int graphSize) {
            if (i == j) return true;
            return (i == 0 && j == graphSize - 1 || i == graphSize-1 && j == 0);
        }
    }
}
