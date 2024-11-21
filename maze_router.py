import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class MazeRouter:
    def __init__(self, grid_size, bend_penalty, via_penalty):
        self.rows, self.cols = grid_size
        self.bend_penalty = bend_penalty
        self.via_penalty = via_penalty
        self.grid_M0 = np.zeros((self.rows, self.cols))  # Layer M0 (horizontal preference)
        self.grid_M1 = np.zeros((self.rows, self.cols))  # Layer M1 (vertical preference)
        self.obstacles = set()
        self.routes = []

    def add_obstacle(self, layer, x, y):
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
                layer, x, y = map(int, line.strip('OBS ()\n').split(', '))
                self.add_obstacle(layer, x, y)

            # Parse nets
            elif line.startswith('net'):
                tokens = line.strip().split()
                net_name = tokens[0]
                pins = [tuple(map(int, pin.strip('()').split(', '))) for pin in tokens[1:]]
                self.routes.append((net_name, pins))

    def route_net(self, net_name, pins):
        visited = set()
        queue = deque([(pins[0][0], pins[0][1], pins[0][2], [], 0)])  # (layer, x, y, path, cost)
        target = pins[1:]

        while queue:
            layer, x, y, path, cost = queue.popleft()

            if (layer, x, y) in visited:
                continue
            visited.add((layer, x, y))

            path = path + [(layer, x, y)]

            # Check if all targets are reached
            if all((tl, tx, ty) in path for tl, tx, ty in target):
                return path

            # Explore neighbors
            for d_layer, dx, dy, penalty in [
                (0, 0, 1, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, -1, 0, 0),  # Same layer
                (1, 0, 0, self.via_penalty)  # Via
            ]:
                nl, nx, ny = layer + d_layer, x + dx, y + dy

                if 0 <= nx < self.rows and 0 <= ny < self.cols and (nl, nx, ny) not in visited:
                    if nl == 0 and self.grid_M0[nx, ny] != -1:
                        queue.append((nl, nx, ny, path, cost + penalty))
                    elif nl == 1 and self.grid_M1[nx, ny] != -1:
                        queue.append((nl, nx, ny, path, cost + penalty))

        return None

    def route_all(self):
        output_routes = []
        for net_name, pins in self.routes:
            route = self.route_net(net_name, pins)
            if route:
                output_routes.append((net_name, route))
        return output_routes

    def save_output(self, output_filename, output_routes):
        with open(output_filename, 'w') as file:
            for net_name, route in output_routes:
                route_str = ' '.join([f'({l}, {x}, {y})' for l, x, y in route])
                file.write(f'{net_name} {route_str}\n')

    def visualize(self, output_routes):
        fig, ax = plt.subplots(figsize=(10, 10))

        # Draw grid
        ax.set_xlim(0, self.cols)
        ax.set_ylim(0, self.rows)
        ax.set_xticks(range(self.cols))
        ax.set_yticks(range(self.rows))
        ax.grid(which='both', color='gray', linestyle='--', linewidth=0.5)

        # Draw obstacles
        for (layer, x, y) in self.obstacles:
            ax.add_patch(plt.Rectangle((y, x), 1, 1, color='black'))

        # Draw routes
        for net_name, route in output_routes:
            xs, ys = zip(*[(y, x) for _, x, y in route])
            ax.plot(xs, ys, marker='o', label=net_name)

        ax.legend()
        plt.gca().invert_yaxis()
        plt.show()

# Example usage:
# router = MazeRouter((100, 200), 20, 5)
# router.parse_input("input.txt")
# output_routes = router.route_all()
# router.save_output("output.txt", output_routes)
# router.visualize(output_routes)
