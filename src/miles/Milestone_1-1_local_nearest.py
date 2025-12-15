import json
import math
from typing import List, Tuple

def load_json_file():
    # Fixed path to your Testcase2
    filepath = "C:/Users/kla_user/Desktop/kla/TestCases/Milestone1/Input_Milestone1_Testcase4.json"
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    start_pos = tuple(data["InitialPosition"])
    initial_angle = data["InitialAngle"]
    v_max = data["StageVelocity"]
    a_max = data["StageAcceleration"]
    omega_max = data["CameraVelocity"]
    alpha_max = data["CameraAcceleration"]
    
    dies = data["Dies"]
    
    centers = []
    target_angles = []
    
    for die in dies:
        corners = die["Corners"]
        cx = sum(c[0] for c in corners) / 4.0
        cy = sum(c[1] for c in corners) / 4.0
        centers.append((cx, cy))
        
        dx = corners[1][0] - corners[0][0]
        dy = corners[1][1] - corners[0][1]
        angle = math.degrees(math.atan2(dy, dx))
        target_angles.append(angle)
    
    return {
        "start_pos": start_pos,
        "initial_angle": initial_angle,
        "v_max": v_max,
        "a_max": a_max,
        "omega_max": omega_max,
        "alpha_max": alpha_max,
        "centers": centers,
        "target_angles": target_angles
    }

def trapezoidal_time(distance: float, v_max: float, a_max: float) -> float:
    if a_max <= 0 or v_max <= 0:
        return distance / v_max if v_max > 0 else 0.0
    
    t_accel = v_max / a_max
    d_accel = v_max * t_accel
    d_total_ramp = 2 * d_accel
    
    if distance <= d_total_ramp:
        return math.sqrt(4 * distance / a_max)
    else:
        d_cruise = distance - d_total_ramp
        t_cruise = d_cruise / v_max
        return 2 * t_accel + t_cruise

def angular_trapezoidal_time(delta_angle: float, omega_max: float, alpha_max: float) -> float:
    delta_angle = abs(delta_angle)
    if alpha_max <= 0 or omega_max <= 0:
        return delta_angle / omega_max if omega_max > 0 else 0.0
    
    t_accel = omega_max / alpha_max
    theta_accel = omega_max * t_accel
    theta_total_ramp = 2 * theta_accel
    
    if delta_angle <= theta_total_ramp:
        return math.sqrt(4 * delta_angle / alpha_max)
    else:
        theta_cruise = delta_angle - theta_total_ramp
        t_cruise = theta_cruise / omega_max
        return 2 * t_accel + t_cruise

def segment_time(
    p1: Tuple[float, float], angle1: float,
    p2: Tuple[float, float], angle2: float,
    v_max: float, a_max: float,
    omega_max: float, alpha_max: float
) -> float:
    distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    delta_angle = (angle2 - angle1) % 360
    delta_angle = min(delta_angle, 360 - delta_angle)
    
    trans_time = trapezoidal_time(distance, v_max, a_max)
    rot_time = angular_trapezoidal_time(delta_angle, omega_max, alpha_max)
    
    return max(trans_time, rot_time)

def nearest_neighbor(data):
    start = data["start_pos"]
    centers = data["centers"]
    angles = data["target_angles"]
    n = len(centers)
    if n == 0:
        return [], 0.0
    
    unvisited = set(range(n))
    path = []
    current_pos = start
    current_angle = data["initial_angle"]
    total_t = 0.0
    
    while unvisited:
        best_idx = min(unvisited,
            key=lambda i: segment_time(
                current_pos, current_angle,
                centers[i], angles[i],
                data["v_max"], data["a_max"],
                data["omega_max"], data["alpha_max"]
            )
        )
        t = segment_time(
            current_pos, current_angle,
            centers[best_idx], angles[best_idx],
            data["v_max"], data["a_max"],
            data["omega_max"], data["alpha_max"]
        )
        total_t += t
        path.append(best_idx)
        current_pos = centers[best_idx]
        current_angle = angles[best_idx]
        unvisited.remove(best_idx)
    
    return path, total_t

def two_opt_improve(data, init_path: List[int]) -> Tuple[List[int], float]:
    n = len(init_path)
    if n < 2:
        return init_path, 0.0
    
    best_path = init_path[:]
    
    def path_time(path):
        t = 0.0
        prev_pos = data["start_pos"]
        prev_angle = data["initial_angle"]
        for idx in path:
            t += segment_time(
                prev_pos, prev_angle,
                data["centers"][idx], data["target_angles"][idx],
                data["v_max"], data["a_max"],
                data["omega_max"], data["alpha_max"]
            )
            prev_pos = data["centers"][idx]
            prev_angle = data["target_angles"][idx]
        return t
    
    best_time = path_time(best_path)
    improved = True
    
    while improved:
        improved = False
        for i in range(1, n-1):
            for j in range(i+2, n+1):
                if j - i == 1: continue
                new_path = best_path[:i] + best_path[i:j][::-1] + best_path[j:]
                new_time = path_time(new_path)
                if new_time < best_time - 1e-8:
                    best_path = new_path
                    best_time = new_time
                    improved = True
    
    return best_path, best_time

# ==================== MAIN ====================
data = load_json_file()

init_path, _ = nearest_neighbor(data)
opt_path, total_time = two_opt_improve(data, init_path)

# Build path coordinates
path_coords = [list(data["start_pos"])]
for idx in opt_path:
    path_coords.append([data["centers"][idx][0], data["centers"][idx][1]])

result = {
    "TotalTime": round(total_time, 3),
    "Path": path_coords
}

# Print to console
print(json.dumps(result, indent=2))

# Save to output JSON file
output_file = "C:/Users/kla_user/Desktop/kla/TestCase_1_4.json"
with open(output_file, 'w') as f:
    json.dump(result, f, indent=2)

print(f"\nOutput saved to: {output_file}")