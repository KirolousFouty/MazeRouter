import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from heapq import heappush, heappop
import heapq


class MazeRouter:
    def __init__(self, rows, cols, obstacles, routes):
        # Initialize grids as numpy arrays (which support item assignment)
        self.grid_M0 = np.zeros((rows, cols), dtype=int)
        self.grid_M1 = np.zeros((rows, cols), dtype=int)
        self.rows = rows
        self.cols = cols
        self.obstacles = obstacles
        self.routes = routes
        self.via_penalty = 10  # Adjust via penalty if necessary

    def add_obstacle(self, layer, x, y):
        # Mark the obstacle on the respective grid (layer 0 or 1)
        if layer == 0:
            self.grid_M0[x, y] = -1
        elif layer == 1:
            self.grid_M1[x, y] = -1
        self.obstacles.add((layer, x, y))

    def parse_input(self, filename):
        with open(filename, 'r') as file:
            lines = file.readlines()

        # First line: Grid size and penalties
        grid_params = lines[0].strip().split(', ')
        self.rows, self.cols = int(grid_params[0]), int(grid_params[1])
        self.bend_penalty = int(grid_params[2])
        self.via_penalty = int(grid_params[3])

        # Parse obstacles
        for line in lines[1:]:
            if line.startswith('OBS'):
                layer, x, y = map(int, line.strip('OBS ()\n').split(','))
                self.add_obstacle(layer, x, y)

            # Parse nets
            elif line.startswith('net'):
                tokens = line.strip().split()
                net_name = tokens[0]
                pins = [tuple(map(int, pin.strip('()').split(','))) for pin in tokens[1:]]
                self.routes.append((net_name, pins))

    def is_valid_move(self, layer, x, y):
        """Check if a move is within bounds and not blocked by an obstacle."""
        if layer == 0:
            return 0 <= x < self.rows and 0 <= y < self.cols and self.grid_M0[x, y] != -1
        elif layer == 1:
            return 0 <= x < self.rows and 0 <= y < self.cols and self.grid_M1[x, y] != -1
        return False

    def route_net(self, net_name, pins):
        """
        Routes a net by connecting all its pins sequentially.
        """
        full_path = []
        for i in range(len(pins) - 1):  # Iterate over each pair of pins
            start = pins[i]
            end = pins[i + 1]
            path = self.route_two_pins(start, end)
            if path is None:  # If no path is found, return failure
                print(f"Failed to route {net_name} between {start} and {end}")
                return None
            full_path.extend(path)  # Add this segment to the full path
        return full_path

    def route_two_pins(self, start, end):
        """
        Routes between two specific pins using A* search.
        """
        open_list = []
        heappush(open_list, (0, start, []))  # (cost, current_position, path)
        g_costs = {start: 0}  # To keep track of the best cost to a node
        visited = set()

        while open_list:
            _, (layer, x, y), path = heappop(open_list)

            if (layer, x, y) in visited:
                continue
            visited.add((layer, x, y))

            path = path + [(layer, x, y)]

            if (layer, x, y) == end:  # Reached the target
                return path

            # Explore neighbors (horizontal, vertical, and via if necessary)
            for d_layer, dx, dy, penalty in [
                (0, 0, 1, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, -1, 0, 0),  # Same layer
                (1, 0, 0, self.via_penalty)  # Switch layer with penalty
            ]:
                nl, nx, ny = layer + d_layer, x + dx, y + dy

                # Check if the move is valid
                if self.is_valid_move(nl, nx, ny) and (nl, nx, ny) not in visited:
                    cost = g_costs[(layer, x, y)] + penalty
                    if (nl, nx, ny) not in g_costs or cost < g_costs[(nl, nx, ny)]:
                        g_costs[(nl, nx, ny)] = cost
                        heappush(open_list, (cost, (nl, nx, ny), path))

        return None

    def route_all(self):
        """
        Routes all nets sequentially.
        """
        output_routes = []
        for net_name, pins in self.routes:
            print(f"Routing {net_name}...")
            route = self.route_net(net_name, pins)
            if route:
                output_routes.append((net_name, route))
            else:
                print(f"Failed to route {net_name}.")
        return output_routes

    def save_output(self, output_filename, output_routes):
        with open(output_filename, 'w') as file:
            for net_name, route in output_routes:
                route_str = ' '.join([f'({l}, {x}, {y})' for l, x, y in route])
                file.write(f'{net_name} {route_str}\n')

    def visualize(self, output_routes):
        fig, ax = plt.subplots(1, 2, figsize=(20, 10))  # Two subplots for M0 and M1
        layers = [self.grid_M0, self.grid_M1]
        titles = ["Layer M0", "Layer M1"]

        for i, layer in enumerate(layers):
            ax[i].set_xlim(0, self.cols)
            ax[i].set_ylim(0, self.rows)
            ax[i].set_xticks(range(self.cols))
            ax[i].set_yticks(range(self.rows))
            ax[i].grid(which='both', color='gray', linestyle='--', linewidth=0.5)
            ax[i].set_title(titles[i])
            ax[i].invert_yaxis()

            # Draw obstacles
            for (l, x, y) in self.obstacles:
                if l == i:
                    ax[i].add_patch(plt.Rectangle((y, x), 1, 1, color='black'))

            # Draw routes for the specific layer
            for net_name, route in output_routes:
                route_layer = [(ly, x, y) for (ly, x, y) in route if ly == i]
                if route_layer:
                    xs, ys = zip(*[(y, x) for _, x, y in route_layer])
                    ax[i].plot(xs, ys, marker='o', label=net_name)

            ax[i].legend()
        plt.show()


# Example usage:
router = MazeRouter(100, 200, set(), [])
router.parse_input("input.txt")
output_routes = router.route_all()
router.save_output("output.txt", output_routes)
router.visualize(output_routes)
