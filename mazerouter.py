import numpy as np
import matplotlib.pyplot as plt
import heapq  # Priority queue for optimal routing


class MazeRouter:
    def __init__(self, grid_size, bend_penalty, via_penalty, move_cost=0):
        self.rows, self.cols = grid_size
        self.bend_penalty = bend_penalty
        self.via_penalty = via_penalty
        self.move_cost = move_cost  # Cost for moving one cell
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
        grid_params = lines[0].strip().split(',')
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

    def route_net(self, net_name, pins):
        visited = set()
        # Priority queue for nodes to explore: (cost, layer, x, y, path, prev_dir)
        queue = []
        heapq.heappush(queue, (0, pins[0][0], pins[0][1], pins[0][2], [], None))
        target = pins[1:]

        while queue:
            cost, layer, x, y, path, prev_dir = heapq.heappop(queue)

            # Debugging point: Current node and cost
            print(f"Exploring node: Layer={layer}, X={x}, Y={y}, Cost={cost}")

            if (layer, x, y) in visited:
                continue
            visited.add((layer, x, y))

            path = path + [(layer, x, y)]

            # Check if all targets are reached
            if all((tl, tx, ty) in path for tl, tx, ty in target):
                # Mark the path as obstacles in the grid
                for (l, px, py) in path:
                    if l == 0:
                        self.grid_M0[px, py] = -1
                    elif l == 1:
                        self.grid_M1[px, py] = -1
                print(f"Completed route for {net_name}. Total cost: {cost}")
                return path

            # Explore neighbors
            for d_layer, dx, dy, penalty in [
                (0, 0, 1, 1), (0, 0, -1, 1), (0, 1, 0, 2), (0, -1, 0, 2),  # Same layer
                (1, 0, 0, self.via_penalty)  # Via
            ]:
                nl, nx, ny = layer + d_layer, x + dx, y + dy

                if 0 <= nx < self.rows and 0 <= ny < self.cols and (nl, nx, ny) not in visited:
                    if nl == 0 and self.grid_M0[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                        move_cost = self.move_cost + penalty + bend_cost
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                        # Debugging point: Neighbor being added to the queue
                        print(f"Adding neighbor: Layer={nl}, X={nx}, Y={ny}, MoveCost={move_cost}, TotalCost={cost + move_cost}")
                    elif nl == 1 and self.grid_M1[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                        move_cost = self.move_cost + penalty + bend_cost
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                        # Debugging point: Neighbor being added to the queue
                        print(f"Adding neighbor: Layer={nl}, X={nx}, Y={ny}, MoveCost={move_cost}, TotalCost={cost + move_cost}")

        print(f"Failed to route {net_name}.")
        return None

    def route_all(self):
        output_routes = []
        for net_name, pins in self.routes:
            print(f"Routing net: {net_name}")
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
router = MazeRouter((100, 200), bend_penalty=5, via_penalty=20, move_cost=0)
router.parse_input("input.txt")
output_routes = router.route_all()
router.save_output("output.txt", output_routes)
router.visualize(output_routes)
