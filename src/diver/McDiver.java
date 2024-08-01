package diver;

import datastructures.PQueue;
import datastructures.SlowPQueue;
import game.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import graph.ShortestPaths;


/** This is the place for your implementation of the {@code SewerDiver}.
 */
public class McDiver implements SewerDiver {

    private Set<Long> discovered = new HashSet<>();
    private SlowPQueue<Long> priorityQueue = new SlowPQueue<>();

    /** See {@code SewerDriver} for specification. */
    @Override
    public void seek(SeekState state) {
        // TODO : Look for the ring and return.
        // DO NOT WRITE ALL THE CODE HERE. DO NOT MAKE THIS METHOD RECURSIVE.
        // Instead, write your method (it may be recursive) elsewhere, with a
        // good specification, and call it from this one.
        //
        // Working this way provides you with flexibility. For example, write
        // one basic method, which always works. Then, make a method that is a
        // copy of the first one and try to optimize in that second one.
        // If you don't succeed, you can always use the first one.
        //
        // Use this same process on the second method, scram.
        dfs1(state);
    }

    private void dfs(SeekState state) {
        if (state.distanceToRing() == 0) {
            return;
        }
        long standingOn = state.currentLocation();
        Collection<NodeStatus> neighbors = state.neighbors();
        discovered.add(standingOn);
        for (NodeStatus neighbor : neighbors) {
            if (!discovered.contains(neighbor.getId())) {
                discovered.add(neighbor.getId());
                state.moveTo(neighbor.getId());
                dfs(state);
                if (state.distanceToRing() == 0) {
                    return;
                }
                state.moveTo(standingOn);
            }
        }
    }

    /** Utilizes an optimized depth first search algorithm to find the shortest path to the ring
     * from the entrance by prioritizing neighbors that have a shorter distance to the ring.
     * Function recursively calls on the algorithm to move through the neighbors and breaks after
     * the call to avoid extra steps being taken after reaching the ring in order to clear the call
     * stack. Verifies that distance to ring is greater than zero before stopping the function.
     */
    // Depth first search algorithm for seek() method
    private void dfs1(SeekState state) {

        // Checks if already reached the ring and exits method
        if (state.distanceToRing() == 0) {
            return;
        }

        // Creates a new list of the neighbors
        // Sorts the neighbors by their manhattan distance to the ring
        long standingOn = state.currentLocation();
        List<NodeStatus> neighbors = new ArrayList<>(state.neighbors());
        Collections.sort(neighbors, Comparator.comparingInt(NodeStatus::getDistanceToRing));

        // Adds current value to the discovered set
        discovered.add(standingOn);

        // Iterates through each neighbor and checks if it has been discovered
        // If neighbor has not been discovered, adds it to the set, and moves to the neighbor
        for (NodeStatus neighbor : neighbors) {
            if (!discovered.contains(neighbor.getId())) {
                discovered.add(neighbor.getId());
                state.moveTo(neighbor.getId());
                // Recursively goes through each neighbor's neighbors
                dfs1(state);
                // Returns method after recursion to avoid extra steps after reaching ring
                if (state.distanceToRing() == 0) {
                    return;
                }
                state.moveTo(standingOn);
            }
        }
    }

    private void dfsQueue(SeekState state) { // implement using priority queue
        if (state.distanceToRing() == 0) {
            return;
        }
        // calculate manhattan distance for each neighbor and rank them in priority queue
        // traverse to the neighbor with the least distance
        long standingOn = state.currentLocation();
        // Collection<NodeStatus> neighbors = state.neighbors();
        // SlowPQueue<Long> priorityQueue = new SlowPQueue<>();
        priorityQueue.add(standingOn, state.distanceToRing());

        while (!priorityQueue.isEmpty()) {
            long current = priorityQueue.extractMin();
            if (!discovered.contains(current)) {
                discovered.add(current);
                Collection<NodeStatus> neighbors = state.neighbors();
                for (NodeStatus neighbor : neighbors) {
                    if (!discovered.contains(neighbor.getId())) {
                        priorityQueue.add(neighbor.getId(), neighbor.getDistanceToRing());
                    }
                }
                long nextNode = priorityQueue.extractMin();
                state.moveTo(nextNode);
                dfsQueue(state);
                if (state.distanceToRing() == 0) {
                    return;
                }
            }
        }
    }


    /** See {@code SewerDriver} for specification. */
    @Override
    public void scram(ScramState state) {
        // TODO: Get out of the sewer system before the steps are used up.
        // DO NOT WRITE ALL THE CODE HERE. Instead, write your method elsewhere,
        // with a good specification, and call it from this one.
        seekShortestPath(state);
    }

    public void shortestPath(ScramState state) {
        Maze maze = new Maze((Set<Node>) state.allNodes());
        ShortestPaths<Node, Edge> shortestPaths = new ShortestPaths<>(maze);

        // Compute the shortest paths from the current node
        shortestPaths.singleSourceDistances(state.currentNode());

        // Get the shortest path to the exit
        List<Edge> shortestPath = shortestPaths.bestPath(state.exit());

        // Follow the shortest path, collecting coins along the way
        for (Edge edge : shortestPath) {
            state.moveTo(maze.dest(edge));
        }
    }

    // Path finding algorithm used in scram() method
    public void seekShortestPath(ScramState state) {
        // Finds shortest paths from current node and creates a priority queue of all nodes
        Maze maze = new Maze((Set<Node>) state.allNodes());
        ShortestPaths<Node, Edge> shortestPaths = new ShortestPaths<>(maze);
        shortestPaths.singleSourceDistances(state.currentNode());
        SlowPQueue<Node> heapOfNodes = new SlowPQueue<>();
        for (Node n : state.allNodes()) {
            heapOfNodes.add(n, (double) n.getTile().coins() * -1);
        }
        while (!heapOfNodes.isEmpty()) {
            List<Edge> pathToExit = shortestPaths.bestPath(state.exit());
            if (lengthOfPath(pathToExit) >= state.stepsToGo()) {
                break;
            }
            if (state.currentNode().equals(state.exit())) {
                break;
            }
            // Extract highestPriority node with the least distance to ring
            Node highestPriority = heapOfNodes.extractMin();
            System.out.println(highestPriority.getTile().coins());
            // Get the shortest path to the exit
            List<Edge> shortestPath = shortestPaths.bestPath(highestPriority);
            for (Edge edge : shortestPath) {
                state.moveTo(maze.dest(edge));
                shortestPaths.singleSourceDistances(state.currentNode());
                shortestPath = shortestPaths.bestPath(highestPriority);
                // Sum all the steps in the path to the exit
                if (lengthOfPath(shortestPath) >= state.stepsToGo()) {
                    break;
                }
            }
            // If path cannot be reached in steps remaining, goes to the shortest path to exit
            shortestPath = shortestPaths.bestPath(state.exit());
            for (Edge edge : shortestPath) {
                state.moveTo(maze.dest(edge));
            }
        }
    }

    public void seekShortestPath2(ScramState state) {
        Maze maze = new Maze((Set<Node>) state.allNodes());
        ShortestPaths<Node, Edge> shortestPaths = new ShortestPaths<>(maze);
        shortestPaths.singleSourceDistances(state.currentNode());
        SlowPQueue<Node> heapOfNodes = new SlowPQueue<>();
        for (Node n : state.allNodes()) {
            heapOfNodes.add(n, (double) n.getTile().coins() * -1);
        }
        while (!heapOfNodes.isEmpty()) {
            Node highestPriority = heapOfNodes.extractMin(); // extract highestPriority node with highest value
            List<Edge> shortestPath = shortestPaths.bestPath(highestPriority); // Get the shortest path to the exit
            for (Edge edge : shortestPath) {
                state.moveTo(maze.dest(edge));
                shortestPaths.singleSourceDistances(state.currentNode());
                shortestPath = shortestPaths.bestPath(highestPriority);
                if (lengthOfPath(shortestPath) < state.stepsToGo()) { // sum all the steps in the path to the exit
                    break;
                }
            }
            shortestPath = shortestPaths.bestPath(state.exit());
            for (Edge edge : shortestPath) {
                state.moveTo(maze.dest(edge));
            }
        }
    }

    public void seekShortestPath3(ScramState state) {
        Maze maze = new Maze((Set<Node>) state.allNodes());
        ShortestPaths<Node, Edge> shortestPaths = new ShortestPaths<>(maze);
        shortestPaths.singleSourceDistances(state.currentNode());
        SlowPQueue<Node> heapOfNodes = new SlowPQueue<>();
        for (Node n : state.allNodes()) {
            heapOfNodes.add(n, (double) n.getTile().coins() * -1);
        }
        if (heapOfNodes.isEmpty()) {
            return;
        }
        Node highestPriority = heapOfNodes.extractMin();
        List<Edge> shortestPath = shortestPaths.bestPath(highestPriority);
        // Check if McDiver can reach the exit with fewer steps than currently available
        if (lengthOfPath(shortestPath) < state.stepsToGo() && lengthOfPath(shortestPath) <
                lengthOfPath(shortestPaths.bestPath(state.exit()))) {
            for (Edge edge : shortestPath) {
                state.moveTo(maze.dest(edge));
                shortestPaths.singleSourceDistances(state.currentNode());
                seekShortestPath3(state);
            }
        } else {
            // McDiver cannot reach the exit with fewer steps, move to the exit directly
            shortestPath = shortestPaths.bestPath(state.exit());
            for (Edge edge : shortestPath) {
                state.moveTo(maze.dest(edge));
            }
        }
    }

    public void seekShortestPath4(ScramState state) {
        Maze maze = new Maze((Set<Node>) state.allNodes());
        ShortestPaths<Node, Edge> shortestPaths = new ShortestPaths<>(maze);
        shortestPaths.singleSourceDistances(state.currentNode());
        SlowPQueue<Node> heapOfNodes = new SlowPQueue<>();
        for (Node n : state.allNodes()) {
            heapOfNodes.add(n, (double) n.getTile().coins() * -1);
        }
        while (!heapOfNodes.isEmpty() && state.stepsToGo() > 0) {
            Node highestPriority = heapOfNodes.extractMin();
            List<Edge> shortestPath = shortestPaths.bestPath(highestPriority);

            if (lengthOfPath(shortestPath) <= state.stepsToGo()) {
                for (Edge edge : shortestPath) {
                    state.moveTo(maze.dest(edge));
                    shortestPaths.singleSourceDistances(state.currentNode());
                }
            } else {
                // McDiver cannot reach the exit with remaining steps, move to the exit directly
                shortestPath = shortestPaths.bestPath(state.exit());
                for (Edge edge : shortestPath) {
                    state.moveTo(maze.dest(edge));
                }
            }
        }
    }

    // Helper method that calculates length of path by incrementing for each edge in shortestPath
    // Returns double value representing path length
    public double lengthOfPath(List<Edge> shortestPath) {
        double lengthOfPath = 0.0;
        for (Edge edge : shortestPath) {
            lengthOfPath += edge.length();
        }
        return lengthOfPath;
    }



}
