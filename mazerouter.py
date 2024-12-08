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
                    result = []
                    current_tuple = []

                    for item in pin_strings:
                        cleaned_item = item.strip('(),')
                        if cleaned_item.isdigit():
                            current_tuple.append(int(cleaned_item))
                        if len(current_tuple) == 3:
                            result.append(tuple(current_tuple))
                            current_tuple = []

                    if len(result) >= 2:
                        pins.extend(result)

                except ValueError as e:
                    print(f"Skipping malformed pin: {result} ({e})")

                if pins:
                    self.routes.append((net_name, pins))
                else:
                    print(f"No valid pins found for net: {net_name}")



    def route_net(self, net_name, pins):
        print(f"\nRouting net '{net_name}' with pins: {pins}")
        visited = set()
        queue = []
        
        # Initialize the queue with the starting pin and consider via at the start
        start_pin = pins[0]
        heapq.heappush(queue, (0, start_pin[0], start_pin[1], start_pin[2], [(start_pin[0], start_pin[1], start_pin[2])], None))
        heapq.heappush(queue, (self.via_penalty, 1 - start_pin[0], start_pin[1], start_pin[2], [(start_pin[0], start_pin[1], start_pin[2])], None))
        targets = set(pins[1:])

        while queue:
            cost, layer, x, y, path, prev_dir = heapq.heappop(queue)
          #  print(f"\nProcessing node: layer={layer}, x={x}, y={y}, cost={cost}")
           # print(f"Path so far: {path}")
            #print(f"Targets remaining: {targets}")
           #print(f"THis is the value of X: {x}")
        #print(f"THis is the value of y: {y}")
           # print(f"End of movment")


            
            # Skip visited nodes
            if (layer, x, y) in visited:
                #print(f"Node (layer={layer}, x={x}, y={y}) already visited, skipping.")
                continue
            
            visited.add((layer, x, y))
            path = path + [(layer, x, y)]

            # Check if all target pins have been reached
            if targets.issubset(path):
                #print(f"All targets reached for net '{net_name}'. Final path: {path}")
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
                                # Adjust movement penalties based on the layer
                if layer == 0:  # Layer 0: Cheaper vertical movement
                    if (dx == 0 and abs(dy) == 1):  # Vertical
                        penalty = 1  # Reduced cost for vertical
                    elif (abs(dx) == 1 and dy == 0):  # Horizontal
                        penalty = 3  # Increased cost for horizontal
                elif layer == 1:  # Layer 1: Cheaper horizontal movement
                    if (abs(dx) == 1 and dy == 0):  # Horizontal
                        penalty = 1  # Reduced cost for horizontal
                    elif (dx == 0 and abs(dy) == 1):  # Vertical
                        penalty = 3  # Increased cost for vertical


                if 0 <= nx < self.rows and 0 <= ny < self.cols and (nl, nx, ny) not in visited:
                    if nl == 0 and self.grid_M0[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        if d_layer == 0:  # Same layer
                            bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                            move_cost = self.move_cost + penalty + bend_cost
                        else:  # Different layer
                            move_cost = self.move_cost + self.via_penalty
                            bend_cost = 0  # No bend cost when changing layers
                        #print(f"Exploring neighbor: layer={nl}, x={nx}, y={ny}")
                       # print(f"Penalty={penalty}, Move_cost={move_cost}, Total_cost={cost + move_cost}")
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                    elif nl == 1 and self.grid_M1[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        if d_layer == 0:  # Same layer
                            bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                            move_cost = self.move_cost + penalty + bend_cost
                        else:  # Different layer
                            move_cost = self.move_cost + self.via_penalty
                            bend_cost = 0  # No bend cost when changing layers
                       ## print(f"Exploring neighbor: layer={nl}, x={nx}, y={ny}")
                       # print(f"Penalty={penalty}, Bend_cost={bend_cost}, Move_cost={move_cost}, Total_cost={cost + move_cost}")
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                   # else:
                      #  print(f"Skipping invalid layer: {nl}")

                       # print(f"Bend_cost '{bend_cost}' routed successfully with final path cost: {cost}")
                       # print(f"vis cosdt '{self.via_penalty}' routed successfully with final path cost: {cost}")
                     #   print(f"cost till now '{penalty}'")

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
                file.write(f'{net_name} {route_str} \n')

    def visualize(self, output_routes):
        fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(20, 10))

        # Draw grid for M0
        ax0.set_xlim(0, self.cols)
        ax0.set_ylim(0, self.rows)
        ax0.set_xticks(range(self.cols))
        ax0.set_yticks(range(self.rows))
        ax0.grid(which='both', color='gray', linestyle='--', linewidth=0.5)
        ax0.set_title('Layer M0')

        # Draw grid for M1
        ax1.set_xlim(0, self.cols)
        ax1.set_ylim(0, self.rows)
        ax1.set_xticks(range(self.cols))
        ax1.set_yticks(range(self.rows))
        ax1.grid(which='both', color='gray', linestyle='--', linewidth=0.5)
        ax1.set_title('Layer M1')

        # Draw obstacles as blocks
        for (layer, x, y) in self.obstacles:
            if layer == 0:
                ax0.add_patch(plt.Rectangle((x, y), 1, 1, color='black'))
            elif layer == 1:
                ax1.add_patch(plt.Rectangle((x, y), 1, 1, color='black'))

        # Draw routes as blocks
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
        color_index = 0
        for net_name, route in output_routes:
            color = colors[color_index % len(colors)]
            for layer, x, y in route:
                if layer == 0:
                    ax0.add_patch(plt.Rectangle((x, y), 1, 1, color=color))
                elif layer == 1:
                    ax1.add_patch(plt.Rectangle((x, y), 1, 1, color=color))
            color_index += 1

        ax0.legend([plt.Line2D([0], [0], color=color, lw=4) for color in colors[:color_index]], 
                   [net_name for net_name, _ in output_routes], loc='upper right')
        ax1.legend([plt.Line2D([0], [0], color=color, lw=4) for color in colors[:color_index]], 
                   [net_name for net_name, _ in output_routes], loc='upper right')
        plt.show()



# Example usage:
router = MazeRouter((100, 200), bend_penalty=5, via_penalty=20, move_cost=0)
# Adjust the initial queue to consider via at the start point
for net_name, pins in router.routes:
    start_pin = pins[0]
    initial_queue = [
        (0, start_pin[0], start_pin[1], start_pin[2], [], None),
        (router.via_penalty, 1 - start_pin[0], start_pin[1], start_pin[2], [], None)  # Consider via at start
    ]
    router.routes.append((net_name, initial_queue))
router.parse_input("input.txt")
output_routes = router.route_all()
router.save_output("output.txt", output_routes)
router.visualize(output_routes)
