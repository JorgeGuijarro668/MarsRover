# 🚀 Mars Rover Terrain Navigation System
### Pathfinding Optimization on Martian Elevation Maps

## 🛰️ Overview
This project implements an algorithmic navigation system for a Mars rover using a **real Martian elevation map** and multiple search algorithms from the **SimpleAI** library. The system evaluates and compares A*, BFS, DFS, Greedy Search, and Iterative Deepening, using a **custom cost function**, **elevation-aware movement constraints**, and a **Euclidean heuristic** to compute optimal routes across complex terrain.

The final optimized A* implementation successfully computed a feasible route across **3,576.9 meters** of terrain, outperforming other approaches in both runtime and path efficiency.

---

## 🗺️ Terrain Map Processing
The preprocessing pipeline loads and converts NASA-style `.img` elevation data into a usable height matrix:

- Parsed metadata from image headers (resolution, scale, min/max elevation)
- Cleaned invalid pixel values and normalized the elevation range
- Applied **local mean downscaling** to reduce resolution while preserving terrain shape
- Generated a final subsampled map (`mars_map.npy`) optimized for pathfinding
- Integrated new meter-per-pixel scale factors after subsampling

---

## 🤖 Navigation Algorithm Design
A custom `MarsSearchProblem` class (SimpleAI-compatible) defines the rover’s constraints and traversal rules.

### ✔️ State Representation
- Each state is a `(row, col)` coordinate on the terrain matrix  
- Supports **8-direction movement** (cardinal + diagonal)

### ✔️ Movement Constraints
- Movement allowed only if **elevation difference ≤ 0.25 m**
- Prevents sharp ascents/descents that a rover could not safely traverse

### ✔️ Cost Function
```
cost = elevation_change + 1
```

### ✔️ Heuristic (A*)
```
h = sqrt((row - goal_row)**2 + (col - goal_col)**2)
```

---

## ⚙️ Algorithms Evaluated
- **A\*** (final chosen algorithm)
- **Greedy Best-First Search**
- **Breadth-First Search (BFS)**
- **Depth-First Search (DFS)**
- **Iterative Deepening DFS**

---

## 📊 Visualization
- 3D terrain rendering using Plotly  
- Rover path overlay with elevation-aware visualization  
- Custom lighting, colorscale, and elevation exaggeration  

---

## 📁 Repository Structure
```
mars-rover-navigation/
├── mars_rover_final.py
├── height_map_preprocessing.py
├── mars_map.npy
└── README.md
```

---

## 🧪 Key Technical Contributions
- Implemented a Mars rover navigation system using **A\***, BFS, DFS, Greedy, and Iterative Deepening  
- Developed a **terrain-aware search problem** with elevation constraints and 8-direction movement  
- Built a full preprocessing pipeline for NASA-style `.img` terrain data  
- Engineered a Euclidean-distance heuristic and elevation-weighted cost model for optimal routing  
- Achieved optimal route planning across **3.5 km of Martian terrain**
- Generated Plotly 3D visualizations for debugging, validation, and analysis  

---

## 🧭 Technologies Used
Python · NumPy · SimpleAI · Plotly · Matplotlib · Scikit-Image
