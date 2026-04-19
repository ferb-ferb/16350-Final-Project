import sys

def run_validation(map_file, traj_file):
    print(f"--- Validating {traj_file} against {map_file} ---")
    
    # 1. Parse Map File
    with open(map_file, 'r') as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]
        
    x_size, y_size = map(int, lines[1].split(','))
    col_thresh = int(lines[3])
    num_robots = int(lines[5])

    starts = [tuple(map(int, p.split(','))) for p in lines[7].split()]
    goals1 = [tuple(map(int, p.split(','))) for p in lines[9].split()]
    goals2 = [tuple(map(int, p.split(','))) for p in lines[11].split()] # New Dropoff line
    
    m_idx = lines.index('M')

    costmap = []
    for line in lines[m_idx+1:]:
        costmap.append(list(map(float, line.split(','))))

    # 2. Parse Trajectory File
    # Format: traj[agent_id][time] = (x, y)
    traj = {i: {} for i in range(num_robots)}
    max_time = 0
    with open(traj_file, 'r') as f:
        for line in f:
            if not line.strip(): continue
            agent, t, x, y = map(int, line.strip().split(','))
            traj[agent][t] = (x, y)
            max_time = max(max_time, t)

    errors = 0

    # 3. Check Start, Goal, Walls, and Valid Steps
    for agent in range(num_robots):
        agent_traj = traj[agent]
        if not agent_traj:
            print(f"[ERROR] Agent {agent} has no trajectory!")
            errors += 1
            continue
            
        # Check Start
        if agent_traj[0] != starts[agent]:
            print(f"[ERROR] Agent {agent} starts at {agent_traj[0]} but should start at {starts[agent]}")
            errors += 1
            
        last_t = max(agent_traj.keys())
        
        # Check Goal 1 (Did it visit the pickup location at some point?)
        if goals1[agent] not in agent_traj.values():
            print(f"[ERROR] Agent {agent} never visited Goal 1 (Pickup) at {goals1[agent]}!")
            errors += 1
        
        # Check Goal 2 (Did it end at the dropoff location?)
        if agent_traj[last_t] != goals2[agent]:
            print(f"[ERROR] Agent {agent} ends at {agent_traj[last_t]} but Goal 2 (Dropoff) is {goals2[agent]}")
            errors += 1

        for t in range(last_t + 1):
            if t not in agent_traj:
                print(f"[ERROR] Agent {agent} is missing time step {t}!")
                errors += 1
                continue
                
            x, y = agent_traj[t]
            
            # Check Wall
            if costmap[y][x] >= col_thresh:
                print(f"[ERROR] Agent {agent} hit a wall at ({x}, {y}) at time {t}!")
                errors += 1
                
            # Check Step Distance (Manhattan distance <= 1)
            if t > 0:
                prev_x, prev_y = agent_traj[t-1]
                dist = abs(x - prev_x) + abs(y - prev_y)
                if dist > 1:
                    print(f"[ERROR] Agent {agent} teleported from ({prev_x},{prev_y}) to ({x},{y}) at time {t}!")
                    errors += 1

    # 4. Check Agent-to-Agent Collisions
    for t in range(max_time + 1):
        # Fill in positions for this time step (agents stay at their last pos if they finished early)
        positions = {}
        for agent in range(num_robots):
            # If agent finished, it stays at its last known location
            t_check = t if t in traj[agent] else max(traj[agent].keys())
            positions[agent] = traj[agent][t_check]

        # Vertex Collisions
        seen_locations = {}
        for agent, pos in positions.items():
            if pos in seen_locations:
                print(f"[ERROR] Vertex Collision! Agent {agent} and Agent {seen_locations[pos]} both at {pos} at time {t}")
                errors += 1
            seen_locations[pos] = agent

        # Edge Collisions (Swapping)
        if t > 0:
            for a1 in range(num_robots):
                for a2 in range(a1 + 1, num_robots):
                    t1_check = t if t in traj[a1] else max(traj[a1].keys())
                    t1_prev = t-1 if t-1 in traj[a1] else max(traj[a1].keys())
                    
                    t2_check = t if t in traj[a2] else max(traj[a2].keys())
                    t2_prev = t-1 if t-1 in traj[a2] else max(traj[a2].keys())
                    
                    if (traj[a1][t1_prev] == traj[a2][t2_check]) and (traj[a1][t1_check] == traj[a2][t2_prev]):
                        print(f"[ERROR] Edge Collision! Agent {a1} and Agent {a2} swapped places at time {t}")
                        errors += 1

    if errors == 0:
        print("\n✅ SUCCESS! Trajectory is 100% valid! No collisions, valid paths, goals reached.")
    else:
        print(f"\n❌ FAILED! Found {errors} errors in the trajectory.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 validator.py <map_file> <trajectory_file>")
    else:
        run_validation(sys.argv[1], sys.argv[2])
