/**
 * Tangent Bug Navigation Simulation
 * --------------------------------------
 * This program implements a Tangent Bug algorithm for autonomous navigation.
 */

ArrayList<ArrayList<PVector>> allObstacles;
ArrayList<PVector> currentTrace;
PVector robotPos, targetPos;
Bot myBot;
int state = 0; // 0: Drawing, 1: Place Robot, 2: Place Target, 3: Simulation

void setup() 
{
  size(800, 600);
  allObstacles = new ArrayList<ArrayList<PVector>>();
  textAlign(CENTER);
}

void draw() 
{
  background(240);
  drawEnvironment();
  
  // State machine for the setup process
  if (state == 0) 
  {
    fill(0); text("STEP 1: Draw CLOSED obstacles and press ENTER", width/2, 30);
  } 
  else if (state == 1) 
  {
    fill(0, 0, 255); text("STEP 2: Click to place the ROBOT", width/2, 30);
    if(robotPos != null) drawGhostPoint(robotPos, color(0, 0, 255));
  } 
  else if (state == 2) 
  {
    fill(0, 150, 0); text("STEP 3: Click to place the TARGET", width/2, 30);
    if(robotPos != null) drawGhostPoint(robotPos, color(0, 0, 255));
    if(targetPos != null) drawGhostPoint(targetPos, color(0, 255, 0));
  } 
  else if (state == 3) 
  {
    myBot.update(targetPos, allObstacles);
    myBot.display(targetPos, allObstacles);
    drawGhostPoint(targetPos, color(0, 255, 0));
  }
}

void drawGhostPoint(PVector p, color c) 
{
  fill(c, 180); noStroke();
  ellipse(p.x, p.y, 15, 15);
}

// --- BOT CLASS (TANGENT BUG LOGIC) ---
class Bot 
{
  PVector pos;
  float sensorRange = 150; // Detection horizon
  float bodyRadius = 18;   // Physical size for collision
  ArrayList<PVector> trail = new ArrayList<PVector>();

  Bot(float x, float y) { pos = new PVector(x, y); 
}

  void update(PVector target, ArrayList<ArrayList<PVector>> obstacles) 
  {
    // Save position to trail every 4 frames
    if (frameCount % 4 == 0) trail.add(pos.copy());
    
    float dTarget = PVector.dist(pos, target);
    if (dTarget < 5) return; // Destination reached

    // 1. DIRECT MOTION: If the path to target is clear within sensor range, go straight
    if (isPathClear(pos, target, obstacles)) 
    {
      moveTowards(target, 2.2);
    } 
    else 
    {
      // 2. TANGENT LOGIC: Find the best visible edge (discontinuity) to bypass obstacle
      PVector escapePoint = findTangentPoint(target, obstacles);
      if (escapePoint != null) {
        moveTowards(escapePoint, 2.0);
      } 
      else 
      {
        // 3. FALLBACK: If stuck, move slowly towards target and let physics handle sliding
        moveTowards(target, 0.5); 
      }
    }
    // Safety physics layer to prevent wall penetration
    applyPhysics(obstacles);
  }

  /**
   * Scans 360 degrees to find "discontinuities" (edges).
   * It selects the edge that minimizes [dist to edge + dist from edge to target].
   */
  PVector findTangentPoint(PVector target, ArrayList<ArrayList<PVector>> obstacles) 
  {
    int rays = 120; 
    PVector bestPoint = null;
    float minTotalDist = Float.MAX_VALUE;
    float lastDist = -1;
    for (int i = 0; i <= rays; i++) 
    {
      float angle = i * TWO_PI / rays;
      PVector rayDir = PVector.fromAngle(angle).setMag(sensorRange);
      PVector rayEnd = PVector.add(pos, rayDir);
      
      PVector hit = getIntersection(pos, rayEnd, obstacles);
      float currentDist = (hit == null) ? sensorRange : PVector.dist(pos, hit);

      // Detect DISCONTINUITY: A sharp jump in distance suggests an obstacle corner/edge
      if (lastDist != -1 && abs(currentDist - lastDist) > 10) 
      {
        // Select the furthest point of the discontinuity as the potential "way out"
        PVector candidate = (currentDist > lastDist) ? rayEnd : hit;
        
        // Apply a safety offset so the bot doesn't scrape the corner
        PVector offsetDir = PVector.sub(candidate, pos).normalize();
        PVector testPoint = PVector.add(candidate, offsetDir.mult(5)); 
        
        // Heuristic: Cost = G(pos to edge) + H(edge to target)
        float score = PVector.dist(pos, testPoint) + PVector.dist(testPoint, target);
        
        if (score < minTotalDist && isPathClear(pos, testPoint, obstacles)) 
        {
          minTotalDist = score;
          bestPoint = testPoint;
        }
      }
      lastDist = currentDist;
    }
    return bestPoint;
  }

  void moveTowards(PVector goal, float speed) 
  {
    PVector dir = PVector.sub(goal, pos);
    if (dir.mag() > 0.1) 
    {
      dir.setMag(speed);
      pos.add(dir);
    }
  }

  /** Checks if a straight line to a point is free of obstacles */
  boolean isPathClear(PVector a, PVector b, ArrayList<ArrayList<PVector>> obstacles) 
  {
    float d = min(PVector.dist(a, b), sensorRange);
    PVector end = PVector.add(a, PVector.sub(b, a).setMag(d));
    return getIntersection(a, end, obstacles) == null;
  }

  /** Returns the closest intersection point of a ray with all obstacles */
  PVector getIntersection(PVector a, PVector b, ArrayList<ArrayList<PVector>> obstacles) 
  {
    PVector closest = null;
    float minDist = Float.MAX_VALUE;
    for (ArrayList<PVector> obs : obstacles) 
    {
      for (int i = 0; i < obs.size(); i++) 
      {
        PVector hit = lineIntersect(a, b, obs.get(i), obs.get((i+1)%obs.size()));
        if (hit != null) 
        {
          float d = PVector.dist(a, hit);
          if (d < minDist) { minDist = d; closest = hit; 
        }
        }
      }
    }
    return closest;
  }

  /** Standard Line-Line intersection formula */
  PVector lineIntersect(PVector a, PVector b, PVector c, PVector d) 
  {
    float den = (d.y-c.y)*(b.x-a.x) - (d.x-c.x)*(b.y-a.y);
    if (den == 0) return null;
    float ua = ((d.x-c.x)*(a.y-c.y)-(d.y-c.y)*(a.x-c.x))/den;
    float ub = ((b.x-a.x)*(a.y-c.y)-(b.y-a.y)*(a.x-c.x))/den;
    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) return new PVector(a.x + ua*(b.x-a.x), a.y + ua*(b.y-a.y));
    return null;
  }

  /** Hard collision solver using segment projection */
  void applyPhysics(ArrayList<ArrayList<PVector>> obstacles) 
  {
    for (ArrayList<PVector> obs : obstacles) 
    {
      for (int i = 0; i < obs.size(); i++) 
      {
        PVector cp = closestPointOnSegment(pos, obs.get(i), obs.get((i+1)%obs.size()));
        float d = PVector.dist(pos, cp);
        if (d < bodyRadius) 
        {
          PVector push = PVector.sub(pos, cp).setMag(bodyRadius - d + 1.5);
          pos.add(push);
        }
      }
    }
  }

  PVector closestPointOnSegment(PVector p, PVector a, PVector b) 
  {
    PVector ap = PVector.sub(p, a);
    PVector ab = PVector.sub(b, a);
    float t = constrain(ap.dot(ab) / ab.magSq(), 0, 1);
    return PVector.add(a, PVector.mult(ab, t));
  }

  void display(PVector target, ArrayList<ArrayList<PVector>> obstacles) 
  {
    // Draw trail
    stroke(0, 150, 255, 50); noFill();
    beginShape(); for (PVector p : trail) vertex(p.x, p.y); endShape();
    
    // Draw sensor horizon
    noFill(); stroke(0, 100, 250, 20); ellipse(pos.x, pos.y, sensorRange*2, sensorRange*2);
    
    // Draw sensor rays hitting obstacles
    stroke(150, 150, 150, 40);
    for (int i = 0; i < 60; i++) 
    {
      PVector ray = PVector.fromAngle(i * TWO_PI / 60).setMag(sensorRange);
      PVector hit = getIntersection(pos, PVector.add(pos, ray), obstacles);
      if (hit != null) line(pos.x, pos.y, hit.x, hit.y);
    }
    
    // Visualize "Thinking" (Optimal Tangent Path)
    PVector thought = findTangentPoint(target, obstacles);
    if (thought != null) 
    {
      stroke(255, 0, 0, 150); strokeWeight(2); line(pos.x, pos.y, thought.x, thought.y);
      fill(255, 0, 0); noStroke(); ellipse(thought.x, thought.y, 6, 6);
    }
    
    // Draw Robot body
    strokeWeight(1);
    fill(0, 100, 255); noStroke(); ellipse(pos.x, pos.y, bodyRadius*2, bodyRadius*2);
    fill(255); ellipse(pos.x, pos.y, 6, 6);
  }
}

// --- WORLD DRAWING FUNCTIONS ---
void drawEnvironment() 
{
  stroke(50); strokeWeight(3); fill(180);
  for (ArrayList<PVector> obs : allObstacles) 
  {
    beginShape(); for (PVector p : obs) vertex(p.x, p.y); endShape(CLOSE);
  }
  if (currentTrace != null) 
  {
    stroke(200, 0, 0); noFill();
    beginShape(); for (PVector p : currentTrace) vertex(p.x, p.y); endShape();
  }
}

void mousePressed() 
{
  if (state == 0) currentTrace = new ArrayList<PVector>();
  else if (state == 1) robotPos = new PVector(mouseX, mouseY);
  else if (state == 2) targetPos = new PVector(mouseX, mouseY);
}

void mouseDragged() 
{ 
  if (state == 0 && currentTrace != null) currentTrace.add(new PVector(mouseX, mouseY)); 
}

void mouseReleased() 
{
  if (state == 0 && currentTrace != null && currentTrace.size() > 2) 
  {
    currentTrace.add(currentTrace.get(0).copy());
    allObstacles.add(currentTrace); currentTrace = null;
  }
}

void keyPressed() 
{
  if (keyCode == ENTER && state < 3) 
  { 
    state++; 
    if (state == 3 && robotPos != null) myBot = new Bot(robotPos.x, robotPos.y); 
  }
  if (key == 'r' || key == 'R') 
  { 
    state = 0; allObstacles.clear(); robotPos = null; targetPos = null; 
  }
}
