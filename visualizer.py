import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import sys


def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'W', "Expected 'W' in the first line"
        x_size_, y_size_ = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'C', "Expected 'C' in the third line"
        collision_thresh = int(file.readline().strip())

        assert file.readline().strip() == 'N', "Expected 'N' in the fifth line"
        num_robots = int(file.readline().strip())

        assert file.readline().strip() == 'S', "Expected 'S' in the seventh line"
        line = file.readline().strip()
        while line != 'M':
            line = file.readline().strip()
            
        costmap_ = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap_.append(row)

        costmap_ = np.asarray(costmap_).T

    return x_size_, y_size_, collision_thresh, num_robots, costmap_


def parse_robot_trajectory_file(filename):
    robot_traj = []
    with open(filename, 'r') as file:
        for line in file:
            n, t, x, y = map(int, line.strip().split(','))
            robot_traj.append({'n':n, 't': t, 'x': x, 'y': y})

    return robot_traj


SPEEDUP = 10

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)

    x_size, y_size, collision_thresh, num_robots, costmap = parse_mapfile(sys.argv[1])

    robot_trajectory = parse_robot_trajectory_file('robot_trajectory.txt')
    robots = {}
    for p in robot_trajectory:
        if p['n'] not in robots:
            robots[p['n']] = []
        robots[p['n']].append(p)

    # Sort 
    for n in robots:
        robots[n].sort(key=lambda p: p['t'])

    # robot IDs
    robot_ids = sorted(robots.keys())

    fig, ax = plt.subplots()

    ax.imshow(costmap, cmap='jet')

    lines = []
    for i in robot_ids:
        # This uses the index 'i' to cycle through colors or set specific ones
        line, = ax.plot([], [], lw=2, marker='o', label=f'Robot {i}')
        lines.append(line)
    def init():
        for line in lines:
            line.set_data([], [])
        return lines

    def update(frame):
        current_time = frame * SPEEDUP
        
        for i, robot_id in enumerate(robot_ids):
            # Filter trajectory points that have occurred up to 'current_time'
            traj = robots[robot_id]
            x_vals = [p['x'] for p in traj if p['t'] <= current_time]
            y_vals = [p['y'] for p in traj if p['t'] <= current_time]
            
            lines[i].set_data(x_vals, y_vals)
            
        # CRITICAL: You must return the updated lines!
        return lines

    # 1. Automatically calculate the total frames needed based on your highest time step
    max_time = max([p['t'] for p in robot_trajectory])
    total_frames = int(max_time / SPEEDUP) + 1

    # 2. Assign to 'ani' so it doesn't get garbage collected. 
    # Set blit=False to ensure it renders flawlessly over a costmap.
    ani = FuncAnimation(fig, update, frames=total_frames, init_func=init, blit=False, interval=200)

    # 3. Explicitly save the output as a real GIF file
    print("Baking animation into robot_path.gif... this might take a few seconds.")
    ani.save('robot_path.gif', writer='pillow')
    print("Done! Open 'robot_path.gif' to see your robots move.")
    # def init():
    #     for line in lines:
    #         line.set_data([], [])
    #     return lines
    #
    # def update(frame):
    #     current_time = frame * SPEEDUP
    #
    #     for i, robot_id in enumerate(robot_ids):
    #         # Filter trajectory points that have occurred up to 'current_time'
    #         traj = robots[robot_id]
    #         x_vals = [p['x'] for p in traj if p['t'] <= current_time]
    #         y_vals = [p['y'] for p in traj if p['t'] <= current_time]
    #
    #         lines[i].set_data(x_vals, y_vals)
    #
    #     return lines
    #
    # # Determine total frames based on the longest trajectory
    # max_time = max(p['t'] for p in robot_trajectory)
    # ani = FuncAnimation(fig, update, frames=max_time // SPEEDUP, 
    #                     init_func=init, blit=True, interval=50)
    #
    # plt.legend()
    # plt.show()
    # ani.save("myGIF.gif")
