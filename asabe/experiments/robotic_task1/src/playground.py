import numpy as np
import cv2
import math
import random
import imageio.v3 as iio
# ----- Constants -----
MAP_SIZE = 500
CART_SIZE = 10
LIDAR_RANGE = 200
ANGLE_STEP = 5
STEP_SIZE = 5

# ----- Map Setup -----
def create_map():
    grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1
    return grid

def random_cart_position():
    margin = CART_SIZE + 10
    x = random.randint(margin, MAP_SIZE - margin)
    y = random.randint(margin, MAP_SIZE - margin)
    return (x, y)

# ----- LIDAR Utilities -----
def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

def simulate_lidar(map_grid, cart_pos):
    cx, cy = cart_pos
    scan = []
    for angle_deg in range(0, 360, ANGLE_STEP):
        angle_rad = math.radians(angle_deg)
        ex = int(cx + LIDAR_RANGE * math.cos(angle_rad))
        ey = int(cy + LIDAR_RANGE * math.sin(angle_rad))
        line_pts = bresenham(cx, cy, ex, ey)
        for (x, y) in line_pts:
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                if map_grid[y, x] == 1:
                    dist = math.hypot(x - cx, y - cy)
                    scan.append((angle_deg, dist))
                    break
        else:
            scan.append((angle_deg, float('inf')))
    return scan

# ----- New Exploration Strategy -----
def get_distance_to_border(scan, direction):
    """Get distance to border in a specific direction (0=East, 90=North, 180=West, 270=South)"""
    min_angle_diff = float('inf')
    border_dist = float('inf')
    
    for angle, dist in scan:
        angle_diff = abs(angle - direction)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        if angle_diff < min_angle_diff and angle_diff < ANGLE_STEP:
            min_angle_diff = angle_diff
            border_dist = dist
    
    return border_dist

def detect_two_borders(scan):
    """Check if we can detect borders in two perpendicular directions"""
    threshold = CART_SIZE * 3
    directions = {
        'East': 0,
        'North': 90,
        'West': 180,
        'South': 270
    }
    
    close_borders = []
    for name, angle in directions.items():
        dist = get_distance_to_border(scan, angle)
        if dist < threshold:
            close_borders.append(name)
    
    return close_borders

def move_in_direction(cart_pos, direction, step_size=STEP_SIZE):
    """Move in a specific direction (0=East, 90=North, 180=West, 270=South)"""
    angle_rad = math.radians(direction)
    dx = int(step_size * math.cos(angle_rad))
    dy = int(step_size * math.sin(angle_rad))
    new_x = np.clip(cart_pos[0] + dx, CART_SIZE, MAP_SIZE - CART_SIZE)
    new_y = np.clip(cart_pos[1] + dy, CART_SIZE, MAP_SIZE - CART_SIZE)
    return (new_x, new_y)

def get_perpendicular_directions(direction):
    """Get the two perpendicular directions"""
    if direction in [0, 180]:  # Moving East or West
        return [90, 270]  # North and South
    else:  # Moving North or South
        return [0, 180]  # East and West

def get_opposite_direction(direction):
    """Get the opposite direction"""
    return (direction + 180) % 360

# ----- Visualization -----
def draw_scene(map_grid, cart_pos, scan_data, explored_map, vertices=None, centroid=None):
    frame = cv2.cvtColor((255 - map_grid * 255), cv2.COLOR_GRAY2BGR)
    cx, cy = cart_pos
    
    # Draw LIDAR rays
    for angle, dist in scan_data:
        if dist == float('inf'): continue
        x = int(cx + dist * math.cos(math.radians(angle)))
        y = int(cy + dist * math.sin(math.radians(angle)))
        cv2.line(frame, (cx, cy), (x, y), (180, 180, 180), 1)
    
    # Draw vertices if found
    if vertices:
        for i, v in enumerate(vertices):
            if v is not None:
                cv2.circle(frame, v, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"V{i+1}", (v[0]+5, v[1]-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # Draw centroid if calculated
    if centroid:
        cv2.circle(frame, centroid, 8, (255, 0, 255), -1)
        cv2.putText(frame, "C", (centroid[0]+5, centroid[1]-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    
    # Draw cart
    cv2.rectangle(frame, (cx - CART_SIZE//2, cy - CART_SIZE//2),
                  (cx + CART_SIZE//2, cy + CART_SIZE//2), (0, 255, 0), -1)
    
    # Overlay explored map
    explored_overlay = cv2.cvtColor(explored_map, cv2.COLOR_GRAY2BGR)
    blended = cv2.addWeighted(frame, 0.8, explored_overlay, 0.2, 0)
    return blended

def mark_explored(explored_map, cart_pos):
    x, y = cart_pos
    cv2.circle(explored_map, (x, y), 3, 255, -1)

# ----- Main Simulation Loop -----
def main():
    map_grid = create_map()
    
    # Random starting position
    cart_pos = random_cart_position()
    print(f"Starting position: {cart_pos}")
    
    explored_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    
    # Direction mapping: 0=East, 90=North, 180=West, 270=South
    directions = [0, 90, 180, 270]
    direction_names = {0: "East", 90: "North", 180: "West", 270: "South"}
    
    # Choose random initial direction
    current_direction = random.choice(directions)
    print(f"Starting direction: {direction_names[current_direction]}")
    
    vertices = [None, None, None, None]
    vertices_found = 0
    phase = "move_to_first_border"
    centroid = None
    visited_directions = set()
    # Add the initial direction to visited
    visited_directions.add(current_direction)

    # Initialize the gif frame
    gif_frames = []
    
    for step in range(3000):
        scan = simulate_lidar(map_grid, cart_pos)
        mark_explored(explored_map, cart_pos)
        
        # Check which borders are close
        close_borders = detect_two_borders(scan)
        
        if phase == "move_to_first_border":
            border_dist = get_distance_to_border(scan, current_direction)
            if border_dist < CART_SIZE * 3:
                print(f"Reached first border moving {direction_names[current_direction]}")
                # Now move perpendicular to find a corner
                perpendicular_dirs = get_perpendicular_directions(current_direction)
                current_direction = random.choice(perpendicular_dirs)
                print(f"Moving {direction_names[current_direction]} to find corner")
                phase = "move_to_first_corner"
                visited_directions.add(current_direction)
            else:
                cart_pos = move_in_direction(cart_pos, current_direction)
        
        elif phase == "move_to_first_corner":
            if len(close_borders) >= 2:
                vertices[vertices_found] = cart_pos
                vertices_found += 1
                print(f"Found vertex {vertices_found} at {cart_pos} (borders: {close_borders})")
                
                # Choose next direction from unused directions
                # Exclude the opposite of the current direction (where we came from)
                opposite = get_opposite_direction(current_direction)
                possible_dirs = []
                for d in directions:
                    if d not in visited_directions and d != opposite:
                        possible_dirs.append(d)
                
                if possible_dirs:
                    current_direction = random.choice(possible_dirs)
                    visited_directions.add(current_direction)
                    print(f"Moving {direction_names[current_direction]} to next corner")
                    phase = "move_to_next_corner"
                else:
                    print("No valid direction found!")
                    break
            else:
                cart_pos = move_in_direction(cart_pos, current_direction)
        
        elif phase == "move_to_next_corner":
            if len(close_borders) >= 2 and cart_pos != vertices[vertices_found-1]:
                # Check if this is a new corner (not the same as previous)
                is_new_corner = True
                for v in vertices[:vertices_found]:
                    if v and math.hypot(cart_pos[0] - v[0], cart_pos[1] - v[1]) < CART_SIZE * 2:
                        is_new_corner = False
                        break
                
                if is_new_corner:
                    vertices[vertices_found] = cart_pos
                    vertices_found += 1
                    print(f"Found vertex {vertices_found} at {cart_pos} (borders: {close_borders})")
                    
                    if vertices_found >= 3:
                        # Calculate fourth vertex
                        v0, v1, v2 = vertices[0], vertices[1], vertices[2]
                        # Find which vertex is diagonal to v1
                        # In a rectangle, if we have 3 vertices, the 4th is calculated by:
                        # v3 = v0 + v2 - v1 (vector addition)
                        vertices[3] = (v0[0] + v2[0] - v1[0], v0[1] + v2[1] - v1[1])
                        print(f"Calculated fourth vertex at {vertices[3]}")
                        
                        # Calculate centroid
                        centroid_x = sum(v[0] for v in vertices) // 4
                        centroid_y = sum(v[1] for v in vertices) // 4
                        centroid = (centroid_x, centroid_y)
                        print(f"Centroid at {centroid}")
                        phase = "navigate_to_centroid"
                    else:
                        # Continue to find next corner
                        # Choose from unused directions, excluding opposite of current
                        opposite = get_opposite_direction(current_direction)
                        possible_dirs = []
                        for d in directions:
                            if d not in visited_directions and d != opposite:
                                possible_dirs.append(d)
                        
                        if possible_dirs:
                            current_direction = random.choice(possible_dirs)
                            visited_directions.add(current_direction)
                            print(f"Moving {direction_names[current_direction]} to find vertex {vertices_found + 1}")
                        else:
                            print("No valid direction found!")
                            break
            else:
                cart_pos = move_in_direction(cart_pos, current_direction)
        
        elif phase == "navigate_to_centroid":
            if centroid:
                dist_to_centroid = math.hypot(cart_pos[0] - centroid[0], 
                                            cart_pos[1] - centroid[1])
                if dist_to_centroid < STEP_SIZE * 2:
                    print("Reached centroid!")
                    phase = "complete"
                else:
                    # Move towards centroid
                    angle_to_centroid = math.degrees(math.atan2(
                        centroid[1] - cart_pos[1], 
                        centroid[0] - cart_pos[0]
                    ))
                    # Normalize angle to 0-360
                    if angle_to_centroid < 0:
                        angle_to_centroid += 360
                    cart_pos = move_in_direction(cart_pos, angle_to_centroid)
        
        elif phase == "complete":
            print("Exploration complete!")
            break
        
        frame = draw_scene(map_grid, cart_pos, scan, explored_map, vertices, centroid)
        gif_frames.append(frame)
        
        # Add phase info to display
        cv2.putText(frame, f"Phase: {phase}", (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Vertices found: {vertices_found}/3", (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        cv2.imshow("Exploration (ESC to Exit)", frame)
        key = cv2.waitKey(30)
        if key == 27: break  # ESC to stop
    
    cv2.waitKey(0)
    iio.imwrite("exploration.gif", gif_frames, duration=0.001, loop=0)
    # Save the gif
    print("Exploration GIF saved as 'exploration.gif'")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()