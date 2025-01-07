import os
import numpy as np
import matplotlib.pyplot as plt
import rps.robotarium as robotarium
from rps.utilities.transformations import create_si_to_uni_mapping
from rps.utilities.barrier_certificates import create_single_integrator_barrier_certificate
from rps.utilities.controllers import create_si_position_controller
from matplotlib.patches import Circle
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

# Parameters
N = 5  # Number of robots
iterations = 4000  # Simulation iterations
sensing_radius = 0.2  # Sensing radius for obstacle avoidance
max_velocity = 0.1  # Maximum velocity to avoid actuator limit errors
fire_radius = 0.21  # Radius of the fire zone
fire_reduction_rate = 0.001  # Rate at which fire is reduced
fire_threshold = 0.01  # Threshold to consider fire extinguished
k_att = 1.0  # Attractive potential scaling factor
k_rep = 0.3  # Repulsive potential scaling factor
random_seed = 42  # Set a random seed for reproducibility

# Battery-related parameters
battery_levels = np.array([95, 90, 95, 55, 85], dtype=float)  # Initial battery percentages
battery_decay_rate = 0.025  # Battery reduction rate per step
low_battery_threshold = 25  # Threshold below which robots go to charging station
charging_station = np.array([0.9, -0.9])  # Bottom-right corner for charging station
charging_radius = 0.1  # Radius of the charging station

# Set random seed for reproducibility
np.random.seed(random_seed)

# Random obstacles (trees)
num_trees = 3  # Number of random obstacles
tree_positions = np.random.uniform(0.4, -0.4, (2, num_trees))  # Random positions for trees

# Create Robotarium instance
r = robotarium.Robotarium(number_of_robots=N, show_figure=True)

# Set up directory for saving frames
frame_dir = "D:/ASU/SEM3/MAE-598/final_project/frames"  # specify your frame directory here
os.makedirs(frame_dir, exist_ok=True)

# Initialize frame counter
frame_counter = 0

# Initialize robots at random positions within a bounded region
x_min, x_max = -1, 1  # Bounds for x-coordinate
y_min, y_max = -1, 1  # Bounds for y-coordinate

# Generate random positions for robots
initial_positions_x = np.random.uniform(x_min, x_max, size=N)  # Random x-coordinates
initial_positions_y = np.random.uniform(y_min, y_max, size=N)  # Random y-coordinates
initial_positions = np.vstack((initial_positions_x, initial_positions_y))  # Combine x and y

# Create transformations and controller
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()
si_barrier_certificate = create_single_integrator_barrier_certificate()
si_position_controller = create_si_position_controller()

# Predefined fire locations
fires = [np.array([[-0.8], [0.5]]),  # Fire 1
         np.array([[0.0], [-0.5]]),  # Fire 2
         np.array([[0.5], [0.5]])]   # Fire 3

fire_status = [1.0, 1.0, 1.0]  # Fire intensity (1.0 = full intensity)

# Add fire emoji as markers
fire_images = []
for fire in fires:
    fire_image = OffsetImage(plt.imread("fire.png"), zoom=0.07)  # Adjust initial zoom
    fire_box = AnnotationBbox(fire_image, fire.flatten(), frameon=False)
    r.axes.add_artist(fire_box)
    fire_images.append(fire_box)

# Draw a green square for the charging station
charging_square = plt.Rectangle((charging_station[0] - 0.05, charging_station[1] - 0.05),
                                 0.1, 0.1, color='green', alpha=0.5)
r.axes.add_patch(charging_square)

# Draw tree obstacles as small dark green circles
tree_circles = []
for i in range(num_trees):
    tree_circle = Circle(tree_positions[:, i], 0.05, color='darkgreen')
    r.axes.add_patch(tree_circle)
    tree_circles.append(tree_circle)

# Initialize a list to store battery annotations
battery_annotations = []
for i in range(N):
    annotation = r.axes.text(
        initial_positions[0, i], 
        initial_positions[1, i], 
        f"{battery_levels[i]:.0f}%", 
        fontsize=10, 
        color='blue'
    )
    battery_annotations.append(annotation)

# Move robots to their random initial positions
for _ in range(150):  # Run for a few iterations to move to initial positions
    poses = r.get_poses()
    si_states = uni_to_si_states(poses)
    velocities = si_position_controller(si_states, initial_positions)
    velocities = si_barrier_certificate(velocities, si_states)
    # Normalize velocities
    norms = np.linalg.norm(velocities, axis=0)
    mask = norms > max_velocity
    velocities[:, mask] = velocities[:, mask] / norms[mask] * max_velocity
    velocities = si_barrier_certificate(velocities, si_states)
    uni_velocities = si_to_uni_dyn(velocities, poses)
    r.set_velocities(np.arange(N), uni_velocities)
    r.step()

# Function to compute attractive potential gradient
def attractive_force(position, goal):
    return -k_att * (position - goal)

# Function to compute repulsive potential gradient
def repulsive_force(position, obstacles, fire_zones, sensing_radius):
    force = np.zeros_like(position)
    for obstacle in obstacles.T:
        diff = position - obstacle
        distance = np.linalg.norm(diff)
        if distance < sensing_radius:
            force += k_rep * (1/distance - 1/sensing_radius) * (1/distance**2) * (diff / distance)
    for fire_zone in fire_zones:
        diff = position - fire_zone.flatten()
        distance = np.linalg.norm(diff)
        if distance < fire_radius:
            force += k_rep * (1/distance - 1/fire_radius) * (1/distance**2) * (diff / distance)
    return force

# Function to check if robots are within the target distance of the fire
def within_fire_zone(positions, fire_location):
    distances = np.linalg.norm(positions - fire_location, axis=0)
    return distances <= fire_radius

# Main loop for robot motion
fire_index = 0
for _ in range(iterations):
    poses = r.get_poses()
    si_states = uni_to_si_states(poses)
    velocities = np.zeros_like(si_states)
    
    for i in range(si_states.shape[1]):
        if battery_levels[i] < low_battery_threshold:
            # Robot moves to the charging station
            f_att = attractive_force(si_states[:, i], charging_station)
            concatenated_obstacles = np.hstack([tree_positions, *fires]) if fires else tree_positions
            f_rep = repulsive_force(si_states[:, i], concatenated_obstacles, fires, sensing_radius)
            velocities[:, i] = f_att + f_rep
        else:
            # Normal behavior toward fire
            target_fire = fires[fire_index]
            f_att = attractive_force(si_states[:, i], target_fire.flatten())
            concatenated_obstacles = np.hstack([tree_positions, *fires]) if fires else tree_positions
            f_rep = repulsive_force(si_states[:, i], concatenated_obstacles, fires, sensing_radius)
            velocities[:, i] = f_att + f_rep
    
    # Normalize velocities
    norms = np.linalg.norm(velocities, axis=0)
    mask = norms > max_velocity
    velocities[:, mask] = velocities[:, mask] / norms[mask] * max_velocity
    velocities = si_barrier_certificate(velocities, si_states)
    uni_velocities = si_to_uni_dyn(velocities, poses)
    r.set_velocities(np.arange(N), uni_velocities)
    r.step()
    
    # Update battery levels
    battery_levels -= battery_decay_rate
    battery_levels = np.clip(battery_levels, 0, 100)
    
    # Update battery annotations
    for i, annotation in enumerate(battery_annotations):
        annotation.set_position((si_states[0, i], si_states[1, i]))
        annotation.set_text(f"{battery_levels[i]:.0f}%")
    
    # Handle fire extinguishing
    in_zone = within_fire_zone(si_states, fires[fire_index])
    if np.any(in_zone) and np.all(battery_levels >= low_battery_threshold):  # Only active robots extinguish fire
        fire_status[fire_index] -= fire_reduction_rate * np.sum(in_zone)
        fire_status[fire_index] = max(fire_status[fire_index], 0.0)
    if fire_status[fire_index] <= fire_threshold:
        fire_images[fire_index].remove()
        fire_index += 1
        if fire_index >= len(fires):
            break
    
    # Capture the frame
    plt.savefig(os.path.join(frame_dir, f"frame_{frame_counter:04d}.png"))
    frame_counter += 1

    # Update charging robots and check if they have reached the charging station
    for i in range(N):
        if battery_levels[i] < low_battery_threshold:
            distance_to_charging = np.linalg.norm(si_states[:, i] - charging_station)
            if distance_to_charging <= charging_radius:
                battery_levels[i] = 100  # Fully recharge battery
    
    # Update fire visualization: resize or hide fire emoji
    if fire_status[fire_index] <= fire_threshold:
        fire_images[fire_index].remove()  # Remove the marker when fire is extinguished
        fire_index += 1
        if fire_index >= len(fires):
            break  # All fires extinguished
    else:
        # Remove and replace the fire marker with updated zoom
        fire_images[fire_index].remove()
        fire_image = OffsetImage(plt.imread("fire.png"), zoom=fire_status[fire_index] * 0.07)
        fire_box = AnnotationBbox(fire_image, fires[fire_index].flatten(), frameon=False)
        r.axes.add_artist(fire_box)
        fire_images[fire_index] = fire_box


# End the simulation
r.call_at_scripts_end()

# Create the video from saved frames
create_video(frame_dir, "simulation_output.mp4")