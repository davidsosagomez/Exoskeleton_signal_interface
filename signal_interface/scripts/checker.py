import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

q1 = np.array([0.03937, 0.04345, 0.17379, 0.98304])
q2 = np.array([-0.17269, 0.04627, 0.17560, 0.96809])

# Calculation of euclidean distance and overall deviation
overall_deviation = np.linalg.norm(q1 - q2)
print(f"Euclidean distance between q1 and q2: {overall_deviation}")
print(f"Overall deviation between q1 and q2: {overall_deviation*100:.2f}%")

# Check if both quaternions describe the same orientation
same_orientation = np.allclose(q1, q2, atol=1e-8) or np.allclose(q1, -q2, atol=1e-8)
if not same_orientation:
    # Normalize quaternions to unit quaternions
    q1_normalized = q1 / np.linalg.norm(q1)
    q2_normalized = q2 / np.linalg.norm(q2)
    # Calculate the angle difference in degrees
    dot_product = np.clip(np.dot(q1_normalized, q2_normalized), -1.0, 1.0)
    angle_difference_degrees = np.degrees(np.arccos(dot_product))
else:
    angle_difference_degrees = 0

print(f"Same orientation: {same_orientation}")
print(f"Angle difference (if not same orientation): {angle_difference_degrees} degrees")

# Convert quaternions to vectors
v1 = q1[1:]
v2 = q2[1:]

# Visualization of orientation vectors
max_limit = np.max(np.abs(np.vstack([v1, v2]))) * 1.1
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='r', length=0.1, normalize=True, label='Orientation of q1')
ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='b', length=0.1, normalize=True, label='Orientation of q2')
ax.set_xlim([-max_limit, max_limit])
ax.set_ylim([-max_limit, max_limit])
ax.set_zlim([-max_limit, max_limit])
#ax.plot([0, max_limit/2], [0, 0], [0, 0], '-k', label='X axis')
#ax.plot([0, 0], [0, max_limit/2], [0, 0], '-g', label='Y axis')
#ax.plot([0, 0], [0, 0], [0, max_limit/2], '-b', label='Z axis')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('Orientation Vectors of q1 and q2 in Cartesian Space')
plt.show()
