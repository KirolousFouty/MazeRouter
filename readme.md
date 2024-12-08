# Maze Router Project

## Team
- Andrew Ishak
- Freddy Amgad
- Kirolous Fouty
- Michael Reda

#### Tentative Guiding Workplan
- **Member 1**: Implement routing algorithm and BFS logic.
- **Member 2**: Input parsing and output generation.
- **Member 3**: Develop visualization and refine visual output.
- **Member 4**: Testing, debugging, and documentation.

#### Workflow history (Intermediate Milestone)
- **Kirolous**: keleton and draft code, but not working properly
- **Michael & Freddy**: fixed the code and the penalties on M0
- **Kirolous**: multi grid and heuristic order support
- **Freddy**: fixed bend calculation, and improved visualization
- **Andrew**: test cases, inputs, and screenshots

#### Workflow history (Final Milestone)
- **Freddy**: multi grid cost, visulization, and correct the logic
- **Kirolous**: fixed parsing, and multi grid visualization
- **Michael**: adjusted_logic_of_cost_across_two_layers, and Statements_Printing
- **Andrew**: test cases, inputs, slides, and screenshots


## Overview
The Maze Router project is designed to optimize routing paths for IC design using a grid-based layout. It supports two-layer routing (M0 and M1) with distinct directional preferences and incorporates penalties for bends and vias. The router reads input files defining grid dimensions, obstacles, and nets and generates output files with optimized routing paths. A visualization script is included to display the results graphically.

---

## Implementation
The project is implemented in Python and follows a modular structure:
- **MazeRouter Class**: Handles grid setup, obstacle placement, routing logic, and result storage.
- **Routing Algorithm**: Uses Breadth-First Search (BFS) for pathfinding, considering penalties for bends and vias.
- **Visualization**: Plots routed paths and obstacles on a 2D grid for easy examination.

---

## Compilation and Execution

### Prerequisites
- Python 3.8 or higher
- Matplotlib library (`pip install matplotlib`)
- NumPy library (`pip install numpy`)

### Steps to Run
1. Place your input file (e.g., `input.txt`) in the project directory.
2. Run the script:
   ```bash
   python maze_router.py input.txt output.txt
3. The output file (output.txt) will be generated in the project directory.


### Challenges
- Balancing Complexity and Efficiency: Ensuring the algorithm handled multi-layer routing while maintaining acceptable performance.
- Handling Multi-Pin Nets: Developing logic to route multiple pins in one net.
- Penalty Calculation: Accurately applying penalties for bends and vias while maintaining shortest paths.
- Visualization: Representing complex routing scenarios clearly.


### Files
- mazerouter.py: Main script containing all functionalities.
- input.txt: Input file.
- output.txt: Generated output file with routed paths.
- README.md: Documentation and report.

### Results
The router has been tested with various scenarios, including simple 2-pin nets, multi-pin nets, varying penalties, and overlapping nets. Visualization confirms correct routing.
