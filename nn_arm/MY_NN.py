
'''
this program load the data from the csv file with the following format:
- [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, X, Y, Z]
disorganize the data and plot the data in a 3D plot
'''

import pandas as pd
column_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'X', 'Y', 'Z']
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt

# Load data from CSV file
data = pd.read_csv(r'FINAL\coffee-dispenser-project\robot_ur3e_perception\robot_ur3e_perception\data.csv', header=None, names=column_names)
# Print column names to verify
print(data.columns)

# round to 3 decimals
data = data.round(3)
# print the first 5 rows
print(data.head())
# disorder the data by rows and reset the index
data = data.sample(frac=1).reset_index(drop=True)
# print the first 5 rows
print(data.head())

# Extract the relevant columns
joint_angles = data.iloc[:, :6]
coordinates = data.iloc[:, 6:]
# split the data 
X_train, X_test, y_train, y_test = train_test_split(coordinates, joint_angles, test_size=0.2, random_state=42)

# normalize the data
X_mean, X_std = X_train.mean(axis=0), X_train.std(axis=0)
y_mean, y_std = y_train.mean(axis=0), y_train.std(axis=0)
X_train_norm = (X_train - X_mean) / X_std
X_test_norm = (X_test - X_mean) / X_std
y_train_norm = (y_train - y_mean) / y_std
y_test_norm = (y_test - y_mean) / y_std

# Define the model
model = tf.keras.Sequential([
    tf.keras.layers.Dense(64, activation='relu', input_shape=(3,)), # Input is XYZ coordinates
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(6)  # Output is 6 joint angles
])
# Compile the model
model.compile(optimizer='adam', loss='mse')

# # Train the model normalized 
# history = model.fit(X_train_norm, y_train_norm, epochs=100, validation_split=0.2, verbose=1)
# # Evaluate the model normalized
# test_loss = model.evaluate(X_test_norm, y_test_norm)
# print(f"Test loss: {test_loss}")
# Train the model
history = model.fit(X_train, y_train, epochs=100, validation_split=0.2, verbose=1)
# Evaluate the model
test_loss = model.evaluate(X_test, y_test)
print(f"Test loss: {test_loss}")

# Use the model to predict angles for a new XYZ position
new_position = np.array([[0.25, 0.25, 0.35]])  # Replace with desired XYZ
predicted_angles = model.predict(new_position)
predicted_angles = predicted_angles.round(3)
print(f"Predicted angles(rad) for position {new_position[0]}: {predicted_angles[0]*np.pi}")
print(f"Predicted angles(deg) for position {new_position[0]}: {np.round(np.degrees(predicted_angles[0]*np.pi), 3)}")

# Plot training history
plt.figure(figsize=(10, 6))
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Model Training History')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()
plt.show()

# Evaluate model performance without normalization
y_pred = model.predict(X_test)
# Convert radians to degrees
y_test_deg = np.degrees(y_test)
y_pred_deg = np.degrees(y_pred)
# Calculate metrics
mse_deg = np.mean((y_test_deg - y_pred_deg)**2, axis=0) # Mean Squared Error
rmse_deg = np.sqrt(mse_deg) # Root Mean Squared Error
mae_deg = np.mean(np.abs(y_test_deg - y_pred_deg), axis=0) # Mean Absolute Error
# Create a DataFrame
results_df = pd.DataFrame({
    'MSE (Degrees)': mse_deg,
    'RMSE (Degrees)': rmse_deg,
    'MAE (Degrees)': mae_deg
}, index=column_names[:6])
print(results_df)

###############################################################################

# # Function to predict angles for a new XYZ position (if normalization was used)
# def predict_angles(xyz):
#     xyz_norm = (xyz - X_mean) / X_std
#     predicted_angles_norm = model.predict(xyz_norm)
#     predicted_angles = predicted_angles_norm * y_std + y_mean
#     return predicted_angles[0]

# # Evaluate model performance if normalization was used
# y_pred_norm = model.predict(X_test_norm)
# y_pred = y_pred_norm * y_std + y_mean
# mse = np.mean((y_test - y_pred)**2, axis=0)
# print("Mean Squared Error for each joint:")
# for i, joint in enumerate(column_names[:6]):
#     print(f"{joint}: {mse[i]}")    
    