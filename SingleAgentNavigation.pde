int windowWidth = 1024;
int windowHeight = 768;
int strokeWidth = 2;

int maxNumObstacles = 1000;
int numObstacles = 75;
Obstacle[] obstacles = new Obstacle[maxNumObstacles];
int maxNumNodes = 1000;
int numNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];
Character character;
Character character2;
ArrayList<Integer> curPath;
ArrayList<Integer> curPath2;
Vec2 startPos = new Vec2(100,100);
Vec2 startPos2 = new Vec2(100,700);
Vec2 goalPos = new Vec2(900,500);
Vec2 goalPos2 = new Vec2(800,200);
int currNode;
int currNode2;
boolean debug = false;
float characterRadius = 20;
PImage characterSprite;
PImage treeSprite;
PImage houseSprite;

void initObstacles(int n) {
  if (n > maxNumObstacles) {
    println("Too many obstacles");
    exit();
  }
  
  for (int i = 0; i < n; i++) {
    obstacles[i] = new Obstacle();
    while (pointInCircle(obstacles[i].c, obstacles[i].r, startPos, character.r) || pointInCircle(obstacles[i].c, obstacles[i].r, startPos2, character2.r) || pointInCircle(obstacles[i].c, obstacles[i].r, goalPos, character.r) || pointInCircle(obstacles[i].c, obstacles[i].r, goalPos2, character2.r)) {
      obstacles[i] = new Obstacle();
    }
  }
}

Vec2[] getObstacleCenters(Obstacle[] obstacles) {
  Vec2[] centers = new Vec2[numObstacles];
  for (int i = 0; i < numObstacles; i++) {
    centers[i] = obstacles[i].c;
  }
  return centers;
}

float[] getObstacleRadii(Obstacle[] obstacles) {
  float[] radii = new float[numObstacles];
  for (int i = 0; i < numObstacles; i++) {
    radii[i] = obstacles[i].r + characterRadius;
  }
  return radii;
}

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  nodePos[0] = startPos;
  nodePos[1] = startPos2;
  for (int i = 2; i < numNodes - 2; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
  nodePos[numNodes - 1] = goalPos;
  nodePos[numNodes - 2] = goalPos2;
}





void setup() {
  size(1024, 768);
  
  character = new Character(startPos.x, startPos.y);
  character2 = new Character(startPos2.x, startPos2.y);
  initObstacles(numObstacles);
  characterSprite = loadImage("sprites/car.png");
  treeSprite = loadImage("sprites/tree.png");
  houseSprite = loadImage("sprites/house.png");

  generateRandomNodes(numNodes, getObstacleCenters(obstacles), getObstacleRadii(obstacles));

  
  connectNeighbors(getObstacleCenters(obstacles), getObstacleRadii(obstacles), numObstacles, nodePos, numNodes);
  
  curPath = planPath(startPos, goalPos, getObstacleCenters(obstacles), getObstacleRadii(obstacles), numObstacles, nodePos, numNodes);
  curPath2 = planPath(startPos2, goalPos2, getObstacleCenters(obstacles), getObstacleRadii(obstacles), numObstacles, nodePos, numNodes);
  while (curPath.get(0) < 0) {
    initObstacles(numObstacles);

    generateRandomNodes(numNodes, getObstacleCenters(obstacles), getObstacleRadii(obstacles));

  
    connectNeighbors(getObstacleCenters(obstacles), getObstacleRadii(obstacles), numObstacles, nodePos, numNodes);
  
    curPath = planPath(startPos, goalPos, getObstacleCenters(obstacles), getObstacleRadii(obstacles), numObstacles, nodePos, numNodes);
    curPath2 = planPath(startPos2, goalPos2, getObstacleCenters(obstacles), getObstacleRadii(obstacles), numObstacles, nodePos, numNodes);
  }
  currNode = 0;
  currNode2 = 0;
}


void draw() {
  strokeWeight(1);
  background(color(255, 255, 255)); //Grey background
  stroke(0, 0, 0);
  fill(255, 0, 0);
  
  
  // Draw obstacles
  if (debug) {
    for (int i = 0; i < numObstacles; i++) {
      obstacles[i].debugRender();
    }
    //Draw Planned Path
    stroke(20,255,40);
    strokeWeight(5);
    if (curPath.size() == 0){
      line(startPos.x,startPos.y,goalPos.x,goalPos.y);
      return;
    }
    line(startPos.x,startPos.y,nodePos[curPath.get(0)].x,nodePos[curPath.get(0)].y);
    for (int i = 0; i < curPath.size()-1; i++){
      int curNode = curPath.get(i);
      int nextNode = curPath.get(i+1);
      line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
    }
    line(goalPos.x,goalPos.y,nodePos[curPath.get(curPath.size()-1)].x,nodePos[curPath.get(curPath.size()-1)].y);
  }

  for (int i = 0; i < numObstacles; i++) {
    obstacles[i].render();
  }

  if (currNode < curPath.size() && nodePos[curPath.get(currNode)].distanceTo(character.p) < 5) {
    currNode++;
  }
  
  if (currNode2 < curPath2.size() && nodePos[curPath2.get(currNode2)].distanceTo(character2.p) < 5) {
    currNode2++;
  }
  
  if (currNode < curPath.size()) {
    Vec2 dir = nodePos[curPath.get(currNode)].minus(character.p).normalized();
    if (dot(character.d, dir) != 1) {
      character.d = interpolate(character.d, dir, 0.05);
    } 
    
    character.p = character.p.plus(character.d);
  }
  
  if (currNode2 < curPath2.size()) {
    Vec2 dir = nodePos[curPath2.get(currNode2)].minus(character2.p).normalized();
    if (dot(character2.d, dir) != 1) {
      character2.d = interpolate(character2.d, dir, 0.05);
    } 
    
    character2.p = character2.p.plus(character2.d);
  }
  
  character.render();
  character2.render();
  
  // Draw start and goal
  pushMatrix();
  translate(-20, -20);
  image(houseSprite, startPos.x, startPos.y, 40, 40);
  image(houseSprite, startPos2.x, startPos2.y, 40, 40);
  image(houseSprite, goalPos.x, goalPos.y, 40, 40);
  image(houseSprite, goalPos2.x, goalPos2.y, 40, 40);
  popMatrix();
  
}

void keyPressed() {

  if (key == 'd') {
    debug = !debug;
  }

}


public class Obstacle {
  private Vec2 c;
  private float r;
  
  public Obstacle() {
    this.r = random(10, 50);
    this.c = new Vec2(random(this.r, windowWidth - this.r), random(this.r, windowHeight - this.r));
  }
  
  public Obstacle(float x, float y, float r) {
    this.c = new Vec2(x, y);
    this.r = r;
  }
  
  public void render() {
    pushMatrix();
    translate(-r, -r);
    image(treeSprite, c.x, c.y, 2 * r, 2 * r);
    popMatrix();
  }
  
  public void debugRender() {
    fill(255, 0, 255);
    circle(this.c.x, this.c.y, this.r + characterRadius);
  }
}

public class Character {
  private Vec2 p;
  private Vec2 d;
  private float r = characterRadius;
  
  public Character(float x, float y) {
    this.p = new Vec2(x, y);
    this.d = new Vec2(0, 1);
  }

  public void render() {
    pushMatrix();
    translate(p.x, p.y);
    rotate(calculateAngle() + PI);
    translate(-p.x, -p.y);
    translate(-r / 2, -r);
    image(characterSprite, p.x, p.y, r, 2 * r);
    popMatrix();
  }
  
  public float calculateAngle() {
    Vec2 up = new Vec2(0, 1).normalized();
    float theta = atan2(d.y, d.x) - atan2(up.y, up.x);
    if (cross(d.normalized(), up) < 0) {
      theta = TWO_PI - theta;
    }
    return theta;
  }
  
  public float getR() {
    return this.r;
  }
}
