import numpy as np
import matplotlib.pyplot as plt
import sys
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
        start_pin = pins[0]
        # Check if the start position is already blocked by a previous net
        if self.grid_M0[start_pin[1], start_pin[2]] == -1 or self.grid_M1[start_pin[1], start_pin[2]] == -1:
            print(f"Start node ({start_pin[1]}, {start_pin[2]}) is already blocked by a previous net's path.")
            return None  # You can choose to reroute instead of returning None if you want

        heapq.heappush(queue, (0, start_pin[0], start_pin[1], start_pin[2], [(start_pin[0], start_pin[1], start_pin[2])], None))
        heapq.heappush(queue, (self.via_penalty, 1 - start_pin[0], start_pin[1], start_pin[2], [(start_pin[0], start_pin[1], start_pin[2])], None))
        targets = set(pins[1:])

        while queue:
            cost, layer, x, y, path, prev_dir = heapq.heappop(queue)

            if (layer, x, y) in visited:
                continue
            
            visited.add((layer, x, y))
            path = path + [(layer, x, y)]

            if targets.issubset(path):
                for l, px, py in path:
                    if l == 0:
                        self.grid_M0[px, py] = -1
                    elif l == 1:
                        self.grid_M1[px, py] = -1
                print(f"Net '{net_name}' routed successfully with final path cost: {cost}")
                return path

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
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))
                    elif nl == 1 and self.grid_M1[nx, ny] != -1:
                        new_dir = (dx, dy) if d_layer == 0 else None
                        if d_layer == 0:  # Same layer
                            bend_cost = self.bend_penalty if prev_dir and prev_dir != new_dir and prev_dir is not None else 0
                            move_cost = self.move_cost + penalty + bend_cost
                        else:  # Different layer
                            move_cost = self.move_cost + self.via_penalty
                            bend_cost = 0  # No bend cost when changing layers
                        heapq.heappush(queue, (cost + move_cost, nl, nx, ny, path, new_dir))

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

            # Lighter colors for routes
            colors = ['#FFCCCC', '#CCFFFF', '#CCFFCC', '#FFCC99', '#D9B3FF', '#F2B3B3', '#FFCCE5', '#D9D9FF', '#B3FFCC', '#C9C9FF']
            color_index = 0
            for net_name, route in output_routes:
                color = colors[color_index % len(colors)]
                for i in range(len(route)):
                    layer, x, y = route[i]
                    if layer == 0:
                        ax0.add_patch(plt.Rectangle((x, y), 1, 1, color=color))
                    elif layer == 1:
                        ax1.add_patch(plt.Rectangle((x, y), 1, 1, color=color))
                    
                    # Check for vias (layer transitions)
                    if i > 0 and route[i][0] != route[i-1][0]:  # If current layer differs from previous layer
                        via_x, via_y = x, y
                        ax0.plot(via_x + 0.5, via_y + 0.5, 'o', color='black', markersize=10, label='Via')
                        ax1.plot(via_x + 0.5, via_y + 0.5, 'o', color='black', markersize=10)

                # Mark start (S) and end (E) points with transparent background
                start_layer, start_x, start_y = route[0]
                end_layer, end_x, end_y = route[-1]
                
                # Mark the start 'S' and end 'E' points with light green and no frames
                if start_layer == 0:
                    ax0.text(start_x + 0.5, start_y + 0.25, 'S', color='green', fontsize=10, fontweight='bold', ha='right', va='top')
                elif start_layer == 1:
                    ax1.text(start_x + 0.5, start_y + 0.25, 'S', color='green', fontsize=10, fontweight='bold', ha='right', va='top' )
                
                if end_layer == 0:
                    ax0.text(end_x + 0.5, end_y + 0.25, 'E', color='red', fontsize=10, fontweight='bold', ha='right', va='top')
                elif end_layer == 1:
                    ax1.text(end_x + 0.5, end_y + 0.25, 'E', color='red', fontsize=10, fontweight='bold', ha='right', va='top' )

                color_index += 1

            ax0.legend([plt.Line2D([0], [0], color=color, lw=4) for color in colors[:color_index]], 
                    [net_name for net_name, _ in output_routes], loc='upper right')
            ax1.legend([plt.Line2D([0], [0], color=color, lw=4) for color in colors[:color_index]], 
                    [net_name for net_name, _ in output_routes], loc='upper right')
            plt.show()
router = MazeRouter((100, 200), bend_penalty=5, via_penalty=20, move_cost=0)
for net_name, pins in router.routes:
    start_pin = pins[0]
    initial_queue = [
        (0, start_pin[0], start_pin[1], start_pin[2], [], None),
        (router.via_penalty, 1 - start_pin[0], start_pin[1], start_pin[2], [], None)  # Consider via at start
    ]
    router.routes.append((net_name, initial_queue))

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <input_file> <output_file>")
        sys.exit(1)
    
    input_file = sys.argv[1]  # argv[1] is the input file
    output_file = sys.argv[2]  # argv[2] is the output file
    router.parse_input(input_file)
    output_routes = router.route_all()
    router.save_output(output_file, output_routes)
    router.visualize(output_routes)

if __name__ == "__main__":
    main()