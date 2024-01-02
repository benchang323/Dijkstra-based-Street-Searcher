package hw8.spp;

import hw8.graph.Edge;
import hw8.graph.Graph;
import hw8.graph.Vertex;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

public class DijkstraStreetSearcher extends StreetSearcher {

  /**
   * Creates a StreetSearcher object.
   *
   * @param graph an implementation of Graph ADT.
   */
  public DijkstraStreetSearcher(Graph<String, String> graph) {
    super(graph);
  }

  @Override
  public void findShortestPath(String startName, String endName) {
    // Input Validation
    inputValidation(startName, endName);

    // Initialize vertices (end points)
    Vertex<String> start = vertices.get(startName);
    Vertex<String> end = vertices.get(endName);

    // Initialize fields
    Map<Vertex<String>, Double> distanceMap = initializeDistanceMap(start);
    PriorityQueue<Vertex<String>> priorityQueue = initializePriorityQueue(distanceMap);
    Set<Vertex<String>> visited = new HashSet<>();

    // Dijkstra's Algorithm
    // repeat N times
    while (!priorityQueue.isEmpty()) {
      // Vertex with smallest distance
      Vertex<String> current = priorityQueue.poll();
      visited.add(current);

      // No path found
      if (current == end) {
        break;
      }

      updateDistances(current, distanceMap, visited, priorityQueue);
    }

    // Update ditance
    double totalDist = distanceMap.get(end);

    // Print Path
    List<Edge<String>> path = getPath(end, start);
    print(totalDist, path);
  }

  private void print(double totalDist, List<Edge<String>> path) {
    if (VERBOSE) {
      printPath(path, totalDist);
    }
  }

  private void inputValidation(String startName, String endName) {
    if (!vertices.containsKey(startName)) {
      System.out.println("Invalid Endpoint: " + startName);
    }
    if (!vertices.containsKey(endName)) {
      System.out.println("Invalid Endpoint: " + endName);
    }
  }

  private Map<Vertex<String>, Double> initializeDistanceMap(Vertex<String> start) {
    Map<Vertex<String>, Double> distanceMap = new HashMap<>();

    // for each vertex v
    for (Vertex<String> vertex : graph.vertices()) {
      if (vertex == start) {
        // distance[s] = 0, s is the source
        distanceMap.put(vertex, 0.0);
      } else {
        // distance[v] = infinity
        distanceMap.put(vertex, MAX_DISTANCE);
        // previous[v] = null and explored[v] = false are not needed
        // since they are implicitly defined by the data structures
      }
    }

    return distanceMap;
  }

  private PriorityQueue<Vertex<String>> initializePriorityQueue(Map<Vertex<String>, Double> distanceMap) {
    // Source: https://www.geeksforgeeks.org/comparator-comparingdouble-method-in-java-with-examples/#
    // Comparator.comparingDouble() method returns a lexicographic-order comparator with a function
    // to allow us to compare distance
    PriorityQueue<Vertex<String>> priorityQueue = new PriorityQueue<>(Comparator.comparingDouble(distanceMap::get));

    // for each vertex v
    for (Vertex<String> vertex : graph.vertices()) {
      // insert into priority queue so minimum edge is at top
      priorityQueue.add(vertex);
    }

    return priorityQueue;
  }

  private void updateDistances(Vertex<String> current, Map<Vertex<String>, Double> distanceMap,
                               Set<Vertex<String>> visited, PriorityQueue<Vertex<String>> priorityQueue) {
    // for every u: unexplored neighbor(v)
    for (Edge<String> edge : graph.outgoing(current)) {
      // get neighbor
      Vertex<String> neighbor = graph.to(edge);

      // skip if visited
      if (visited.contains(neighbor)) {
        continue;
      }

      // d = distance[v] + weight[v, u]
      double newDist = distanceMap.get(current) + (double) graph.label(edge);

      // if d < distance[u]
      if (newDist < distanceMap.get(neighbor)) {
        // distance[u] = d
        // previous[u] = v
        priorityQueue.remove(neighbor);
        distanceMap.put(neighbor, newDist);
        graph.label(neighbor, edge);
        priorityQueue.add(neighbor);
      }
    }
  }
}
