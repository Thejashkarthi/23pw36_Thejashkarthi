import json
import math
from itertools import permutations
from typing import List, Tuple
import sys

def load_json(filename: str):
    with open(filename, 'r') as f:
        data = json.load(f)
    
    start_pos = tuple(data["InitialPosition"])
    v = data["StageVelocity"]  # 50 for Milestone 1
    
    centers = []
    for die in data["Dies"]:
        corners = die["Corners"]
        cx = sum(c[0] for c in corners) / 4.0
        cy = sum(c[1] for c in corners) / 4.0
        centers.append((cx, cy))
    
    return start_pos, centers, v

def path_distance(start: Tuple[float, float], centers: List[Tuple[float, float]], order: tuple) -> float:
    if not order:
        return 0.0
    d = math.hypot(start[0] - centers[order[0]][0], start[1] - centers[order[0]][1])
    for i in range(len(order) - 1):
        d += math.hypot(centers[order[i]][0] - centers[order[i+1]][0],
                        centers[order[i]][1] - centers[order[i+1]][1])
    return d

def solve_tsp_exact(filename: str):
    start, centers, velocity = load_json(filename)
    n = len(centers)
    
    min_dist = float('inf')
    best_order = None
    
    for perm in permutations(range(n)):
        dist = path_distance(start, centers, perm)
        if dist < min_dist:
            min_dist = dist
            best_order = perm
    
    total_time = min_dist / velocity
    
    # Build path
    path_coords = [list(start)]
    for idx in best_order:
        path_coords.append([centers[idx][0], centers[idx][1]])
    
    result = {
        "TotalTime": round(total_time, 3),
        "Path": path_coords
    }
    
    # Print and save
    print(json.dumps(result, indent=2))
    
    output_file = filename.replace("Input_", "Output_")
    with open(output_file, 'w') as f:
        json.dump(result, f, indent=2)
    
    print(f"\nSaved to: {output_file}")

# Usage
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python solution.py <input_json_file>")
        sys.exit(1)
    solve_tsp_exact(sys.argv[1])