
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
# print(data.columns)

# round to 3 decimals
data = data.round(3)
# print the first 5 rows
# print(data.head())
# disorder the data by rows and reset the index
data = data.sample(frac=1).reset_index(drop=True)
# print the first 5 rows
# print(data.head())

# Extract the relevant columns
joint_angles = data.iloc[:, :6]
coordinates = data.iloc[:, 6:]
# split the data 
X_train, X_test, y_train, y_test = train_test_split(coordinates, joint_angles, test_size=0.2, random_state=42)

# normalize the data, they are angles from -pi to pi
X_train_norm = X_train/np.pi
X_test_norm = X_test/np.pi 
y_train_norm = y_train/np.pi
y_test_norm =  y_test/np.pi

# Define the model
model = tf.keras.Sequential([
    tf.keras.layers.Dense(64, activation='relu', input_shape=(3,)), # Input is XYZ coordinates
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(6, activation='tanh')  # Output is 6 joint angles
])
# Compile the model
model.compile(optimizer='adam', loss='mse')
# Show the model summary
# model.summary()

# Train the model normalized 
early_stopping = tf.keras.callbacks.EarlyStopping(patience=50, restore_best_weights=True)
history = model.fit(X_train_norm, y_train_norm, epochs=1000, validation_split=0.2, 
                    callbacks=[early_stopping], verbose=1)
# Evaluate the model normalized
test_loss = model.evaluate(X_test_norm, y_test_norm)
print(f"Test loss: {test_loss}")

# Use the model to predict angles for a new XYZ position
new_position = np.array([[0.25, 0.25, 0.35]])  # Replace with desired XYZ
predicted_angles = model.predict(new_position)
predicted_angles = predicted_angles.round(3)
print(f"Predicted angles(rad) for position {new_position[0]}: {predicted_angles[0]*np.pi}")
print(f"Predicted angles(deg) for position {new_position[0]}: {np.round(np.degrees(predicted_angles[0]*np.pi), 3)}")

# Plot training history
plt.figure(figsize=(6, 6))
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Model Training History')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()
plt.grid(True)  # Add grid
plt.show()

# Correct approach for evaluating the normalized model performance
y_pred_norm = model.predict(X_test_norm)  # Use normalized test data for prediction
y_pred = y_pred_norm * np.pi  # Denormalize predictions to radians before converting to degrees

# Convert radians to degrees
y_test_deg = np.degrees(y_test)  # y_test is already in radians, so just convert to degrees
y_pred_deg = np.degrees(y_pred)  # Convert predicted radians to degrees

# Calculate metrics
mse_deg = np.mean((y_test_deg - y_pred_deg)**2, axis=0)  # Mean Squared Error
rmse_deg = np.sqrt(mse_deg)  # Root Mean Squared Error
mae_deg = np.mean(np.abs(y_test_deg - y_pred_deg), axis=0)  # Mean Absolute Error

# Create a DataFrame for results
results_df = pd.DataFrame({
    'MSE (Degrees)': mse_deg,
    'RMSE (Degrees)': rmse_deg,
    'MAE (Degrees)': mae_deg
}, index=column_names[:6])
print(results_df)

# Plotting test values vs predicted values for each joint angle
fig, axes = plt.subplots(3, 2, figsize=(8, 8))  # Adjust the size as needed
axes = axes.flatten()
# Convert y_test_deg and y_pred_deg to NumPy arrays if they are pandas DataFrames
y_test_deg_np = y_test_deg.to_numpy() if isinstance(y_test_deg, pd.DataFrame) else y_test_deg
y_pred_deg_np = y_pred_deg.to_numpy() if isinstance(y_pred_deg, pd.DataFrame) else y_pred_deg

# Then use these NumPy arrays for plotting
for i, col_name in enumerate(column_names[:6]):
    axes[i].scatter(y_test_deg_np[:, i], y_pred_deg_np[:, i], alpha=0.5)
    axes[i].plot([y_test_deg_np[:, i].min(), y_test_deg_np[:, i].max()], [y_test_deg_np[:, i].min(), y_test_deg_np[:, i].max()], 'k--', lw=2)
    axes[i].set_xlabel('Actual')
    axes[i].set_ylabel('Predicted')
    axes[i].set_title(f'{col_name} (Degrees)')

plt.tight_layout()
plt.show()