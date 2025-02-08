from picarx import Picarx
import time

# Configuration parameters
MAX_POWER = 20      # Maximum motor power for forward movement
MIN_POWER = 5       # Minimum motor power for precise control
SafeDistance = 40   # Distance threshold for normal operation
DangerDistance = 20 # Distance threshold for emergency maneuvers
SCAN_ANGLES = [-30, -10, 0, 10, 30]  # Angles for environmental scanning
FULL_SCAN_INTERVAL = 25     # Number of iterations between full scans

def read_distance(px):
    """
    Read and validate distance from ultrasonic sensor
    Args:
        px: Picarx instance
    Returns:
        Valid distance reading in cm or None if invalid
    """
    distance = px.ultrasonic.read()
    if distance is None or distance < 0:
        return None
    return round(distance, 2)

def scan_direction(px, angle):
    """
    Measure distance at a specific angle
    Args:
        px: Picarx instance
        angle: Servo angle for measurement
    Returns:
        Measured distance in cm or large value if reading invalid
    """
    px.set_dir_servo_angle(angle)
    time.sleep(0.1)  # Allow servo to stabilize
    
    distance = read_distance(px)
    if distance is None:
        return SafeDistance + 10  # Return a safe value if reading invalid
    return distance

def calculate_power(distance):
    """
    Calculate motor power based on obstacle distance
    Args:
        distance: Measured distance to obstacle in cm
    Returns:
        Calculated motor power value
    """
    if distance >= SafeDistance:
        return MAX_POWER
    elif distance <= DangerDistance:
        return MIN_POWER
    else:
        # Linear interpolation for smooth power adjustment
        power_range = MAX_POWER - MIN_POWER
        distance_range = SafeDistance - DangerDistance
        power = MIN_POWER + power_range * (distance - DangerDistance) / distance_range
        return round(power)

def get_min_distance_angle(distances):
    """
    Find the angle with minimum distance
    Args:
        distances: Dictionary of angles and their distances
    Returns:
        Angle with minimum distance
    """
    min_distance = float('inf')
    min_angle = 0
    
    for angle, distance in distances.items():
        if distance < min_distance:
            min_distance = distance
            min_angle = angle
            
    return min_angle

def perform_full_scan(px):
    """
    Perform a complete environmental scan at all defined angles
    Args:
        px: Picarx instance
    Returns:
        Tuple of (best_angle, distances_dict) or (None, distances_dict) if no safe direction
    """
    # Stop movement during scan
    px.forward(0)
    
    max_distance = 0
    best_angle = 0
    distances = {}
    
    # Scan at each defined angle
    for angle in SCAN_ANGLES:
        distance = scan_direction(px, angle)
        distances[angle] = distance
        if distance > max_distance:
            max_distance = distance
            best_angle = angle
    
    # Check if all directions are unsafe
    if all(d <= DangerDistance for d in distances.values()):
        return None, distances
    
    return best_angle, distances

def initialize_robot(px):
    """
    Initialize robot to starting position
    Args:
        px: Picarx instance
    """
    print("Initializing robot position...")
    px.set_dir_servo_angle(0)  # Set direction servo to zero
    time.sleep(0.5)  # Allow servo to reach position

def cleanup_robot(px):
    """
    Clean up robot state before termination
    Args:
        px: Picarx instance
    """
    print("Cleaning up...")
    # First stop all movement
    px.forward(0)
    time.sleep(0.1)  # Brief pause to ensure motion has stopped
    
    # Then set servo to zero position
    px.set_dir_servo_angle(0)
    time.sleep(0.1)  # Allow servo to reach position
    
    print("Robot stopped and servo centered")

def main():
    px = None
    try:
        px = Picarx()
        # px = Picarx(ultrasonic_pins=['D2','D3']) # tring, echo
        
        # Initialize direction servo to zero
        initialize_robot(px)
        print("Robot initialized and ready")
        
        current_angle = 0    # Current steering angle
        iteration_count = 0  # Counter for full scan timing
        
        # Initial pause to let sensor stabilize
        time.sleep(0.5)
        
        while True:
            iteration_count += 1
            
            # Regular forward distance measurement
            forward_distance = read_distance(px)
            
            # Handle invalid reading
            if forward_distance is None:
                print("Invalid distance reading - pausing briefly")
                px.forward(0)
                time.sleep(0.1)
                continue
            
            print(f"Forward distance: {forward_distance}")
            
            # Check for immediate danger first
            if forward_distance <= DangerDistance:
                print("Obstacle too close - moving backward")
                # Quick scan to determine reverse direction
                _, distances = perform_full_scan(px)
                min_angle = get_min_distance_angle(distances)
                reverse_angle = -min_angle  # Turn opposite to closest obstacle
                print(f"Reversing with angle: {reverse_angle}°")
                px.set_dir_servo_angle(reverse_angle)
                px.backward(MAX_POWER)
                time.sleep(1.0)
                px.forward(0)
                time.sleep(0.2)
                continue
            
            # Perform full scan periodically
            if iteration_count >= FULL_SCAN_INTERVAL:
                print("Performing full scan...")
                best_angle, distances = perform_full_scan(px)
                print("Scan results:", distances)
                
                if best_angle is None:
                    # All directions blocked - reverse away from closest obstacle
                    print("All directions blocked - reversing")
                    min_angle = get_min_distance_angle(distances)
                    reverse_angle = -min_angle  # Turn opposite to closest obstacle
                    print(f"Reversing with angle: {reverse_angle}°")
                    px.set_dir_servo_angle(reverse_angle)
                    px.backward(MAX_POWER)
                    time.sleep(1.0)
                    px.forward(0)
                    time.sleep(0.2)
                    iteration_count = 0
                    continue
                
                # Update steering based on scan results
                current_angle = best_angle
                iteration_count = 0
            
            # Calculate appropriate power
            power = calculate_power(forward_distance)
            
            # Determine steering angle
            if forward_distance >= SafeDistance:
                # Clear path - gradually straighten
                target_angle = 0
            else:
                # Adjust angle based on proximity
                angle_factor = (SafeDistance - forward_distance) / (SafeDistance - DangerDistance)
                target_angle = current_angle * angle_factor
            
            # Smooth angle transition
            current_angle = current_angle * 0.7 + target_angle * 0.3
            
            # Apply movement controls
            px.set_dir_servo_angle(current_angle)
            print(f"Moving: angle={current_angle:.1f}°, power={power}")
            px.forward(power)
            
            # Brief delay between measurements
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        if px:
            cleanup_robot(px)
            print("Program ended")

if __name__ == "__main__":
    main()
