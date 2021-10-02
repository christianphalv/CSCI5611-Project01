//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The function connectNeighbors() will always be called before planPath()
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of the positions in the nodePos array will ever be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file (PRM.pde) for compatabilty reasons.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it uses BFS which will not provide the shortest path.
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment. This file is
// intended to illustrate the basic set-up for the assignmtent, don't assume 
// this example funcationality is correct and end up copying it's mistakes!).


// IMPORTS
import java.util.PriorityQueue;


//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node


//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestNode(startPos, nodePos, numNodes);
  int goalID = closestNode(goalPos, nodePos, numNodes);
  
  path = runAStar(nodePos, numNodes, startID, goalID);
  //path = fixPath(path, startPos, goalPos, centers, radii, numObstacles, nodePos);
  
  return path;
}

/*
// Fixes the closest node
ArrayList<Integer> fixPath(ArrayList<Integer> path, Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos) {
  
  for (int i = path.size() - 1; i >=0; i--) {
    hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], nodePos[path.get(i)].minus(startPos), nodePos[path.get(i)].distanceTo(startPos));
    if (!circleListCheck.hit){
      path.subList(0, i-1).clear();
      break;
    }
  }
  return path;
}
*/

// A*
ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, int startID, int goalID) {
  PriorityQueue<Node> fringe = new PriorityQueue();  //New empty fringe
  Node[] nodes = new Node[maxNumNodes];
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) {
    parent[i] = -1; //No parent yet
    nodes[i] = new Node(i, Float.MAX_VALUE, nodePos[i].distanceTo(nodePos[goalID]));
  }
  
  
  fringe.add(nodes[startID]);
  nodes[startID].setG(0);
  
  while (fringe.size() > 0) {
    Node currentNode = fringe.poll();
    
    if (currentNode.getID() == goalID){
      break;
    }

    for (int i = 0; i < neighbors[currentNode.getID()].size(); i++) {
      Node neighborNode = nodes[neighbors[currentNode.getID()].get(i)];
      float calculatedDistance = currentNode.getG() + nodePos[currentNode.getID()].distanceTo(nodePos[neighborNode.getID()]);
      if (neighborNode.getG() > calculatedDistance) {
        parent[neighborNode.getID()] = currentNode.getID();
        neighborNode.setG(calculatedDistance);
        if (!fringe.contains(neighborNode)) {
          fringe.add(neighborNode);
        }
      }
      
    }
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }

  return path;
}

// Class Node
public class Node implements Comparable<Node> {
  private int ID;
  private float g;
  private float h;
  private float f;
  
  public Node(int ID, float g, float h) {
    this.ID = ID;
    this.g = g;
    this.h = h;
    this.f = g + h;
  }
  
  public void setG(float g) {
    this.g = g;
    this.f = this.g + this.h;
  }
  
  public int getID() {
    return this.ID;
  }
  public float getG() {
    return this.g;
  }
  
  public float getH() {
    return this.h;
  }
  
  public float getF() {
    return this.f;
  }
  
  @Override
  public int compareTo(Node other) {
    return Float.compare(this.f, other.f);
  }
  
  @Override
  public boolean equals(Object other) {
    if (other == this) {
      return true;
    } else if (other instanceof Integer) {
      Integer otherInteger = (Integer) other;
      return Integer.compare(this.ID, otherInteger) == 0;
    } else if (!(other instanceof Node)) {
      return false;
    } else {
      Node otherNode = (Node) other;
      return Integer.compare(this.ID, otherNode.getID()) == 0;
    }
  }
}
