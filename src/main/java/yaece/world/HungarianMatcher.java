// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import java.util.Arrays;

/**
 * Exact O(n^3) Kuhn-Munkres (Hungarian) linear assignment solver for zero-swap multi-target data
 * association between existing tracks and incoming sensor observations.
 */
public final class HungarianMatcher {
  private static final double INF = 1e9;

  private HungarianMatcher() {}

  /**
   * Solves minimum-cost 1-to-1 assignment on a rectangular cost matrix {@code cost[rows][cols]},
   * rejecting any pair whose cost strictly exceeds {@code maxAllowedCost}.
   *
   * @param cost non-empty rectangular matrix of non-negative association costs
   * @param maxAllowedCost maximum gating cost; assignments above this become {@code -1}
   * @return array of length {@code cost.length} where {@code result[r]} is the assigned column
   *     index or {@code -1} if unmatched
   */
  public static int[] solve(double[][] cost, double maxAllowedCost) {
    int rows = cost.length;
    if (rows == 0) {
      return new int[0];
    }
    int cols = cost[0].length;
    int[] result = new int[rows];
    Arrays.fill(result, -1);
    if (cols == 0) {
      return result;
    }

    int dim = Math.max(rows, cols);
    double dummyCost = maxAllowedCost + 1.0;

    // 1-indexed classic Jonker-Volgenant / Hungarian potentials
    double[] u = new double[dim + 1];
    double[] v = new double[dim + 1];
    int[] p = new int[dim + 1];
    int[] way = new int[dim + 1];

    for (int i = 1; i <= dim; i++) {
      p[0] = i;
      int j0 = 0;
      double[] minv = new double[dim + 1];
      Arrays.fill(minv, INF);
      boolean[] used = new boolean[dim + 1];
      do {
        used[j0] = true;
        int i0 = p[j0];
        double delta = INF;
        int j1 = 0;
        for (int j = 1; j <= dim; j++) {
          if (!used[j]) {
            double c =
                (i0 <= rows && j <= cols) ? Math.min(cost[i0 - 1][j - 1], dummyCost) : dummyCost;
            double cur = c - u[i0] - v[j];
            if (cur < minv[j]) {
              minv[j] = cur;
              way[j] = j0;
            }
            if (minv[j] < delta) {
              delta = minv[j];
              j1 = j;
            }
          }
        }
        for (int j = 0; j <= dim; j++) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);

      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    for (int j = 1; j <= cols; j++) {
      int r = p[j] - 1;
      if (r >= 0 && r < rows && cost[r][j - 1] <= maxAllowedCost) {
        result[r] = j - 1;
      }
    }
    return result;
  }
}
