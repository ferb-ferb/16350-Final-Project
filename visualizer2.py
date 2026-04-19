import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

# --- CONFIGURATION ---
# Set SPEEDUP = 1 to see every single step.
SPEEDUP = 1 
# Set the time between frames in milliseconds (e.g., 400ms = 0.4 seconds per step)
FRAME_DELAY_MS = 400 

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'W', "Expected 'W' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'C', "Expected 'C' in the third line"
        collision_thresh = int(file.readline().strip())

        assert file.readline().strip() == 'N', "Expected 'N' in the fifth line"
        num_robots = int(file.readline().strip())

        assert file.readline().strip() == 'S', "Expected 'S' in the seventh line"
        starts = [tuple(map(int, p.split(','))) for p in file.readline().strip().split()]
        
        assert file.readline().strip() == 'G', "Expected 'G' in the ninth line"
        goals1 = [tuple(map(int, p.split(','))) for p in file.readline().strip().split()]

        assert file.readline().strip() == 'D', "Expected 'D' in the eleventh line"
        goals2 = [tuple(map(int, p.split(','))) for p in file.readline().strip().split()]

        # Skip to the 'M' line
        line = file.readline().strip()
        while line != 'M':
            line = file.readline().strip()
            
        costmap = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap.append(row)

        # Transpose so x and y align with matplotlib's coordinate system
        costmap = np.asarray(costmap)

    return x_size, y_size, collision_thresh, num_robots, starts, goals1, goals2, costmap


def parse_robot_trajectory_file(filename):
    robot_traj = []
    with open(filename, 'r') as file:
        for line in file:
            if not line.strip(): continue
            parts = line.strip().split(',')
            # Form: agent, time, x, y
            robot_traj.append({
                'n': int(parts[0]),
                't': int(parts[1]),
                'x': int(parts[2]),
                'y': int(parts[3])
            })
    return robot_traj


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python visualizer.py <map_file> <trajectory_file>")
        sys.exit(1)

    map_file = sys.argv[1]
    traj_file = sys.argv[2]

    # 1. Load the Map and Trajectories
    x_size, y_size, col_thresh, num_robots, starts, goals1, goals2, costmap = parse_mapfile(map_file)
    robot_trajectory = parse_robot_trajectory_file(traj_file)

    # Group trajectory by robot
    robots = {i: [] for i in range(num_robots)}
    for p in robot_trajectory:
        robots[p['n']].append(p)

    for n in robots:
        robots[n].sort(key=lambda p: p['t'])

    # 2. Setup the Plotting Canvas
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Use binary colormap (0=white, 1=black) for a crisp grid look
    ax.imshow(costmap, cmap='binary', origin='upper')

    ax.set_xticks(np.arange(-0.5, x_size, 1))
    ax.set_yticks(np.arange(-0.5, y_size, 1))
    
    # Hide the tick labels so numbers don't clutter the plot
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Draw a slightly more solid grid on those exact tick marks
    ax.grid(color='gray', linestyle='-', linewidth=0.8, alpha=0.7)
    
    # Prevent the tiny tick marks themselves from jutting out of the map
    ax.tick_params(axis='both', which='both', length=0)

    # Set up colors so each robot has a distinct theme
    colors = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#46f0f0', '#f032e6']
    
    # 3. Draw the static start and goal markers
    for i in range(num_robots):
        c = colors[i % len(colors)]

        # Start = Circle (o) - Lowest layer
        ax.plot(starts[i][0], starts[i][1], marker='o', color=c, markersize=8, 
                linestyle='None', alpha=0.5, label=f'R{i} Start', zorder=2)

        # Goal 2 / Dropoff = Square (s) - Middle layer (made size 12 so it acts like a base)
        ax.plot(goals2[i][0], goals2[i][1], marker='s', color=c, markersize=12, 
                linestyle='None', alpha=0.9, markeredgecolor='black', label=f'R{i} Dropoff', zorder=3)

        # Goal 1 / Pickup = Triangle (^) - Top layer (always renders above squares!)
        ax.plot(goals1[i][0], goals1[i][1], marker='^', color=c, markersize=9, 
                linestyle='None', alpha=1.0, label=f'R{i} Pickup', zorder=4)
    # Optional: Put legend outside the plot so it doesn't cover the grid
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize='small')
    plt.tight_layout()

    # 4. Setup the Animation Lines
    lines = []
    heads = [] # We'll draw a solid dot at the robot's current location
    for i in range(num_robots):
        c = colors[i % len(colors)]
        # The trail behind the robot
        line, = ax.plot([], [], lw=3, color=c, alpha=0.6)
        # The robot itself
        head, = ax.plot([], [], marker='o', color=c, markersize=10, markeredgecolor='black')
        
        lines.append(line)
        heads.append(head)

    def init():
        for line in lines:
            line.set_data([], [])
        for head in heads:
            head.set_data([], [])
        return lines + heads

    def update(frame):
        current_time = frame * SPEEDUP
        
        for i in range(num_robots):
            traj = robots[i]
            x_vals = [p['x'] for p in traj if p['t'] <= current_time]
            y_vals = [p['y'] for p in traj if p['t'] <= current_time]
            
            # Update the trail
            lines[i].set_data(x_vals, y_vals)
            
            # Update the robot's current position (the head)
            if x_vals and y_vals:
                heads[i].set_data([x_vals[-1]], [y_vals[-1]])
                
        return lines + heads

    # 5. Bake the GIF
    max_time = max([p['t'] for p in robot_trajectory])
    total_frames = int(max_time / SPEEDUP) + 1

    print(f"Baking animation into robot_path.gif ({total_frames} frames)...")
    # Notice `interval=FRAME_DELAY_MS` which slows the GIF down!
    ani = FuncAnimation(fig, update, frames=total_frames, init_func=init, blit=False, interval=FRAME_DELAY_MS)
    
    ani.save('robot_path.gif', writer='pillow')
    print("Done! Open 'robot_path.gif' to see your robots move.")
