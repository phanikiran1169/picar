## PicarX

<p align="center">
    <img src="https://github.com/user-attachments/assets/c25ca09d-8d7f-4e89-b808-32345767032c" width="45%" alt="Obstacle Detection">
    <img src="https://github.com/user-attachments/assets/623586e9-4bf3-436a-8609-51f461cc6ef9" width="45%" alt="Path Selection">
</p>

## Obstacle Avoidance Algorithm

The PicarX robot autonomously moves forward while using an ultrasonic sensor to measure distances and avoid obstacles. It continuously adjusts its speed and direction based on the environment.

When the sensor detects no obstacles beyond 40 cm, the robot moves straight at full speed. If an obstacle is detected within 40 cm, it slows down and begins adjusting its steering angle to avoid a collision. If an obstacle is too close (≤ 20 cm), the robot immediately stops, moves backward, and scans its surroundings.

To make better navigation decisions, the robot performs a full scan every few cycles, measuring distances at multiple angles (-30° to 30°). It selects the best direction with the most clearance and adjusts its steering. If all directions are blocked, it reverses to reassess the environment before moving forward again.

This cycle of scanning, adjusting, and moving continues indefinitely, allowing the robot to navigate obstacles dynamically
