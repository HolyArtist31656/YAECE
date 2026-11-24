package yaece.world;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;

class HungarianMatcherTest {

  @Test
  void solvesOptimalRectangularAssignmentWithGating() {
    double[][] cost = {
      {0.2, 3.0, 5.0},
      {2.5, 0.3, 4.0},
      {9.0, 8.0, 4.5}
    };
    int[] assign = HungarianMatcher.solve(cost, 1.0);
    assertArrayEquals(new int[] {0, 1, -1}, assign);
  }

  @Test
  void swapsSuboptimalGreedyChoiceForGlobalMinimum() {
    // Greedy would pick (0->0: 0.4) leaving (1->1: 10.0); optimal is (0->1: 0.5, 1->0: 0.45)
    double[][] cost = {
      {0.40, 0.50},
      {0.45, 10.0}
    };
    int[] assign = HungarianMatcher.solve(cost, 1.0);
    assertArrayEquals(new int[] {1, 0}, assign);
  }
}
