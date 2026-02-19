# 🤖 Warehouse Robot Simulator

A Python-based simulation demonstrating pathfinding algorithms and route optimization strategies in a grid-based warehouse environment.

## 🚀 Overview
This project simulates an autonomous robot navigating a warehouse to collect packages. It visualizes the trade-offs between different navigation strategies, tracking metrics like **Energy Consumption** and **Time Efficiency**.

It was built to demonstrate:
* **Pathfinding:** A* (A-Star) Algorithm for obstacle avoidance.
* **Route Optimization:** A greedy "Nearest Neighbor" approach to solve the Traveling Salesman Problem (TSP).

## 📄 Project Report
For a detailed explanation of the algorithms (A* & TSP), mathematical proofs, and performance analysis, please refer to the full project report:
**[Read the Full Report (PDF)](./wbs_project_report.pdf)**

## 🛠️ Features
* **Interactive Grid:** Left-click to place orders, Right-click to place walls/shelves.
* **Two Operation Modes:**
    1.  **Single Return:** Robot returns home after every package (Low efficiency).
    2.  **Batch (TSP):** Robot calculates an optimized route to collect all packages in one trip (High efficiency).
* **Real-time Metrics:** Live tracking of distance traveled and time elapsed.
* **Dynamic Visuals:** built with `pygame`.

## 🎮 Controls
| Key / Action | Function |
|:---|:---|
| **Left Click** | Place an Order (Target) |
| **Right Click** | Toggle Obstacle (Wall) |
| **R Key** | Reset Robot & Metrics |
| **C Key** | Clear Entire Grid |
| **UI Buttons** | Select Strategy & Run Mission |

## 🧠 Algorithms Used
* **A* Search:** Used for point-to-point navigation to find the shortest path around obstacles using Manhattan Distance heuristic.
* **Nearest Neighbor:** Used in 'Batch Mode' to sort the target list by proximity, minimizing total travel distance.
