import numpy as np
import matplotlib.pyplot as plt
import heapq  # Priority queue for optimal routing

class MazeRouter:
    def __init__(self, grid_size, bend_penalty, via_penalty, move_cost=0):
        self.rows, self.cols = grid_size
        self.bend_penalty = bend_penalty
        self.via_penalty = via_penalty
        self.move_cost = move_cost
        self.grid_M0 = np.zeros((self.rows, self.cols))  # Layer M0
        self.grid_M1 = np.zeros((self.rows, self.cols))  # Layer M1
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

        # Parse grid size and penalties
        grid_params = lines[0].strip().split(',')
        self.rows, self.cols = int(grid_params[0]), int(grid_params[1])
        self.bend_penalty = int(grid_params[2])
        self.via_penalty = int(grid_params[3])

        # Parse obstacles and nets
        for line in lines[1:]:
            line = line.strip()
            if not line:  # Skip empty lines
                continue

            if line.startswith('OBS'):
                try:
                    # Parse obstacle coordinates
                    parts = line.replace('OBS', '').replace('(', '').replace(')', '').replace(" ", "").split(',')
                    if len(parts) != 3:
                        raise ValueError(f"Invalid obstacle format: {line}")
                    layer, x, y = map(int, parts)
                    self.add_obstacle(layer, x, y)
                except ValueError:
                    print(f"Malformed obstacle line: {line}")
                    continue

            elif line.startswith('net'):
                tokens = line.split()
                net_name = tokens[0]
                pins = []

                try:
                    # Group coordinates into 3D tuples
                    pin_strings = tokens[1:]  # Collect pin strings
                    # print(pin_strings)

                    # Initialize an empty list for tuples
                    result = []

                    # Temporary storage for one tuple
                    current_tuple = []

                    # Iterate through the list
                    for item in pin_strings:
                        # Remove parentheses and commas, then split to extract numbers
                        cleaned_item = item.strip('(),')
                        if cleaned_item.isdigit():  # Check if the cleaned item is a number
                            current_tuple.append(int(cleaned_item))
                            
                        # If a tuple reaches 3 elements, append it to the result and reset
                        if len(current_tuple) == 3:
                            result.append(tuple(current_tuple))
                            current_tuple = []

                    pins.append(result[0])
                    pins.append(result[1])

                except ValueError as e:
                    print(f"Skipping malformed pin: {result} ({e})")

                if pins:
                    self.routes.append((net_name, pins))
                else:
                    print(f"No valid pins found for net: {net_name}")



    def route_net(self, net_name, pins):
        visited = set()
        queue = []
        
        # Initialize the queue with the starting pin
        heapq.heappush(queue, (0, pins[0][0], pins[0][1], pins[0][2], [], None))
        targets = set(pins[1:])

        while queue:
            cost, layer, x, y, path, prev_dir = heapq.heappop(queue)
           #print(f"THis is the value of X: {x}")
        #print(f"THis is the value of y: {y}")
           # print(f"End of movment")


            
            # Skip visited nodes
            if (layer, x, y) in visited:
                continue
            
            visited.add((layer, x, y))
            path = path + [(layer, x, y)]

            # Check if all target pins have been reached
            if targets.issubset(path):
                for l, px, py in path:
                    if l == 0:
                        self.grid_M0[px, py] = -1
                    elif l == 1:
                        self.grid_M1[px, py] = -1
                # Debugging statement for the net name and the final path cost
                print(f"Net '{net_name}' routed successfully with final path cost: {cost}")
                return path

            # Explore neighbors
            for d_layer, dx, dy, penalty in [
                (0, 0, 1, 1), (0, 0, -1, 1), (0, 1, 0, 2), (0, -1, 0, 2),
                (1, 0, 0, self.via_penalty)
            ]:
                nl, nx, ny = layer + d_layer, x + dx, y + dy

                if 0 <= nx < self.rows and 0 <= ny < self.cols and (nl, nx, ny) not in visited:
                    if nl == 0 and self.grid_M0[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                        move_cost = self.move_cost + penalty + bend_cost
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                        #print(f"Evaluating: layer={nl}, x={nx}, y={ny}, cost={move_cost}, penalty={penalty}")
                    elif nl == 1 and self.grid_M1[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                        move_cost = self.move_cost + penalty + bend_cost
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                        #print(f"Evaluating: layer={nl}, x={nx}, y={ny}, cost={move_cost}, penalty={penalty}")

        # If no valid path is found, print debugging info
        print(f"Net '{net_name}' could not be routed.")
        return None

    def heuristic_order(self):
        def net_priority(net):
            _, pins = net
            distances = [
                abs(p1[1] - p2[1]) + abs(p1[2] - p2[2]) for i, p1 in enumerate(pins) for p2 in pins[i + 1:]
            ]
            return sum(distances)

        self.routes.sort(key=net_priority)

    def route_all(self):
        self.heuristic_order()
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

        # Draw obstacles as points
        for (layer, x, y) in self.obstacles:
            ax.plot(x , y , marker='o', color='black', markersize=6)

        # Draw routes
        for net_name, route in output_routes:
            xs, ys = zip(*[(x, y) for _, x, y in route])
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