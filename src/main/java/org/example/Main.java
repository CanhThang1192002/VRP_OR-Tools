// Copyright 2010-2022 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Minimal example to call the GLOP solver.
// [START program]
package org.example;
// [START import]

import com.google.ortools.Loader;
import com.google.ortools.constraintsolver.RoutingIndexManager;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.sat.*;

import java.io.File;
import java.util.Scanner;

public class Main {
  private final static String filename = "src/main/java/org/example/data.txt";
  private int numberOfVehicle;
  private int numberOfCustomer;
  private int[] demand;
  private int[][] timeMatrix;
  private int[][] timeWindow;
  private int capacity;
  private int depot;

  public static void main(String[] args) {
    Loader.loadNativeLibraries();
    CpModel model = new CpModel();
    Main main = new Main();
    main.readData(filename);
    RoutingIndexManager routingIndexManager =
            new RoutingIndexManager(main.numberOfCustomer, main.numberOfVehicle, main.depot);
    RoutingModel routingModel = new RoutingModel(routingIndexManager);
    // biến nhị phân x_i_j_k = 1 nếu xe k đi từ i đến j
    // với i,j thuộc từ 0 đến N+1, k thuộc từ 0 đến K-1
    int numberOfVertex = main.numberOfCustomer + 2;
    int[] depot = {0, numberOfVertex - 1};
    //    The VRPTW can be defined on a directed graph G = (V, A) where |V | = n + 2, and the
//depot is represented by the two vertices 0 and n+1. Feasible vehicle routes then correspond
//to paths starting at vertex 0 and ending at vertex n + 1. The set of vehicles is denoted by
//K, with |K| = m. Let si denote the service time at i (with s0 = sn+1 = 0) and let tij be the
//travel time from i to j. In addition to the time window [ai, bi] associated with each vertex
//i ∈ N = V \ {0, n + 1}, time windows [a0, b0] and [an+1, bn+1] can also be associated with
//the depot vertex. If no particular restrictions are imposed on vehicle availability, one may
//simply set a0 = mini∈N{ai − t0i}, b0 = maxi∈N{bi − t0i}, an+1 = mini∈N{ai + si + ti,n+1}
//and bn+1 = maxi∈N{bi+si+ti,n+1}. As in the CVRP, let qi denote the demand of customer
//i, and let Q be the vehicle capacity.
//While several models are available for the VRPTW, this problem is often formulated as
//a multicommodity network flow model with time window and capacity constraints. This
//model involves two types of variables: binary variables xk ij, (i, j) ∈ A, k ∈ K, equal to 1
//if and only if arc (i, j) is used by vehicle k, and continuous variables wik, i ∈ N, k ∈ K,
//indicating the time at which vehicle k starts servicing vertex i. Let δ+(i) = {j : (i, j) ∈ A}
//and δ−(j) = {i : (i, j) ∈ A}. The problem can then be stated as follows (see, e.g.,
//Desrochers et al. (1988)) :
    IntVar[][][] x = new IntVar[numberOfVertex][numberOfVertex][main.numberOfVehicle];
    for (int i = 0; i < numberOfVertex; i++) {
      for (int j = 0; j < numberOfVertex; j++) {
        for (int k = 0; k < main.numberOfVehicle; k++) {
          x[i][j][k] = model.newIntVar(0, 1, "x[" + i + "," + j + "," + k + "]");
        }
      }
    }
    // biến t_i_j là thời gian đi từ i đến j
    IntVar[][] t = new IntVar[numberOfVertex][numberOfVertex];
    for (int i = 0; i < numberOfVertex; i++) {
      for (int j = 0; j < numberOfVertex; j++) {
        t[i][j] = model.newIntVar(0, 1000000, "t[" + i + "," + j + "]");
      }
    }
    //biến s_i là thời gian phục vụ khách hàng i(s_0 = s_n+1 = 0)
    IntVar[] s = new IntVar[numberOfVertex];
    for (int i = 0; i < numberOfVertex; i++) {
      s[i] = model.newIntVar(0, 1000000, "s[" + i + "]");
    }
    // biến [a_i,b_i] là thời gian khách hàng i có thể được phục vụ với i thuộc 1 đến N+1
    IntVar[][] a = new IntVar[numberOfVertex][2];
    for (int i = 0; i < numberOfVertex; i++) {
      for (int j = 0; j < 2; j++) {
        a[i][j] = model.newIntVar(0, 1000000, "a[" + i + "," + j + "]");
      }
    }
    // biến q_i là nhu cầu của khách hàng i
    IntVar[] q = new IntVar[numberOfVertex];
    for (int i = 0; i < numberOfVertex; i++) {
      q[i] = model.newIntVar(0, 1000000, "q[" + i + "]");
    }
    // biến Q là dung lượng của xe
    IntVar Q = model.newIntVar(0, 1000000, "Q");
    // delta+(i) = {j : (i, j) ∈ A}
    // delta-(j) = {i : (i, j) ∈ A}
    IntVar[][] deltaPlus = new IntVar[numberOfVertex][numberOfVertex];
    IntVar[][] deltaMinus = new IntVar[numberOfVertex][numberOfVertex];
    for (int i = 0; i < numberOfVertex; i++) {
      for (int j = 0; j < numberOfVertex; j++) {
        deltaPlus[i][j] = model.newIntVar(0, 1, "deltaPlus[" + i + "," + j + "]");
        deltaMinus[i][j] = model.newIntVar(0, 1, "deltaMinus[" + i + "," + j + "]");
      }
    }

    // ràng buộc 1: sum(sum(x_i_j_k)) = 1 với mọi i thuộc N, k thuộc K, j thuộc delta+(i)
    for (int i = 0; i < numberOfVertex; i++) {
      for (int k = 0; k < main.numberOfVehicle; k++) {
        IntVar[] tmp = new IntVar[numberOfVertex];
        for (int j = 0; j < numberOfVertex; j++) {
          tmp[j] = x[i][j][k];
        }
        LinearExpr linearExpr = LinearExpr.sum(tmp);
        model.addEquality(linearExpr, 1);
      }
    }
    // ràng buộc 2: với mỗi j thuộc delta+(0), với mỗi xe k thuộc K, sum(x_0_j_k) = 1
    for (int j = 0; j < numberOfVertex; j++) {
      IntVar[] tmp = new IntVar[main.numberOfVehicle];
      System.arraycopy(x[0][j], 0, tmp, 0, main.numberOfVehicle);
      LinearExpr linearExpr = LinearExpr.sum(tmp);
      model.addEquality(linearExpr, 1);
    }

    IntVar obj = model.newIntVar(0, 1000000, "obj");
    model.addLessOrEqual(obj, 1000000);
    model.minimize(obj);

    CpSolver solver = new CpSolver();
    CpSolverStatus status = solver.solve(model);
    if (status == CpSolverStatus.OPTIMAL) {
      System.out.println("obj = " + solver.objectiveValue());
      for (int i = 0; i < main.numberOfCustomer; i++) {
        for (int j = 0; j < main.numberOfCustomer; j++) {
          for (int k = 0; k < main.numberOfVehicle; k++) {
            if (solver.value(x[i][j][k]) == 1) {
              System.out.println("x[" + i + "," + j + "," + k + "] = " + solver.value(x[i][j][k]));
            }
          }
        }
      }
    } else {
      System.out.println("No solution");
    }


  }

  private void readData(String filename) {
    try {
      Scanner scanner = new Scanner(new File(filename));
      numberOfCustomer = scanner.nextInt();
      numberOfVehicle = scanner.nextInt();
      capacity = scanner.nextInt();
      timeWindow = new int[numberOfCustomer][2];
      demand = new int[numberOfCustomer];
      timeMatrix = new int[numberOfCustomer][numberOfCustomer];
      for (int i = 0; i < numberOfCustomer; i++) {
        for (int j = 0; j < numberOfCustomer; j++) {
          timeMatrix[i][j] = scanner.nextInt();
        }
      }
      for (int i = 0; i < numberOfCustomer; i++) {
        demand[i] = scanner.nextInt();
        timeWindow[i][0] = scanner.nextInt();
        timeWindow[i][1] = scanner.nextInt();
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }


}
