
'''
this program load the data from the csv file with the following format:
- [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, X, Y, Z]
disorganize the data and plot the data in a 3D plot
'''

import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
column_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'X', 'Y', 'Z']

# Load data from CSV file
data = pd.read_csv(r'FINAL\coffee-dispenser-project\robot_ur3e_perception\robot_ur3e_perception\data.csv', header=None, names=column_names)
# Print column names to verify
print(data.columns)

# Extract the relevant columns
joint_angles = data.iloc[:, :6]
coordinates = data.iloc[:, 6:]

# Plot the data in a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(coordinates['X'], coordinates['Y'], coordinates['Z'])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()