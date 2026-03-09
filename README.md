# TangentBug-Navigation-Processing

An autonomous robot navigation simulation using the **Tangent Bug** algorithm, implemented in Processing (Java).



## 📌 Overview
This project demonstrates a reactive navigation strategy where a robot reaches a target by scanning its environment in real-time. Unlike global planners (like A* or Dijkstra), the **Tangent Bug** algorithm doesn't require a pre-existing map. It makes decisions based on local sensor data, identifying obstacle edges ("discontinuities") to find the optimal path.

## 📺 Demo
### 1. Simple Obstacle

[SimpleObstacle.webm](https://github.com/user-attachments/assets/f22e44fb-e678-4964-9c00-0c8b82f71012)

### 2. Multiple Obstacles

[MultipleObstacles.webm](https://github.com/user-attachments/assets/014e8f77-d855-4baa-a01e-b9b43b8392a3)

### 3. Concave Obstacle

[ConcaveObstacle.webm](https://github.com/user-attachments/assets/f5ad7875-6981-4693-9f93-553e27bc2c1d)

### 4. Convex Obstacle

[ConvexObstacle.webm](https://github.com/user-attachments/assets/6041ba86-4407-4398-aa68-08ad8ba35d8b)


## 🚀 Features
* **Discontinuity Detection**: The robot scans 360° and identifies sharp jumps in distance to find obstacle corners.
* **Optimal Heuristic**: Chooses the tangent point that minimizes the total estimated distance: $d(robot, edge) + d(edge, target)$.
* **Real-time Visualization**:
    * **Gray Rays**: Show actual sensor hits on obstacles.
    * **Red Line**: Displays the robot's "thought process" (the chosen tangent).
    * **Blue Trail**: Tracks the historical path of the robot.
* **Reactive Physics**: A collision layer prevents the robot from phasing through walls while allowing it to "slide" along them.


## 🧠 The Algorithm
The core logic follows the Tangent Bug principles:
1.  **Motion-to-Goal**: If the target is visible within the `sensorRange`, move directly towards it.
2.  **Boundary-Following (Virtual)**: If the direct path is blocked, the robot identifies all visible "discontinuities" (where an obstacle starts or ends from its perspective).
3.  **Tangent Selection**: It calculates a "cost" for each detected edge using the following heuristic:
    $$f(x) = d(robot, x) + d(x, target)$$
    The robot then targets the edge $x$ that offers the shortest path to the goal.

## 🛠️ Installation & Usage
1.  Download and install [Processing 4](https://processing.org/).
2.  Clone this repository or copy the code into a new sketch.
3.  Run the sketch and follow these steps:
    * **Step 1 (Draw)**: Draw closed polygons with the mouse. Press `ENTER`.
    * **Step 2 (Robot)**: Click to place the **Blue Robot**. Press `ENTER`.
    * **Step 3 (Target)**: Click to place the **Green Target**. Press `ENTER`.
    * **Step 4 (Simulate)**: Watch the Tangent Bug navigate! Press `R` to reset.

## 📝 License
This project is open-source and available under the MIT License. Feel free to use it for educational purposes!
