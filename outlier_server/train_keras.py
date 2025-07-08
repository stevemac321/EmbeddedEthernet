import numpy as np
import pandas as pd
import tensorflow as tf

# 📂 Load and inspect the data
X = pd.read_csv('real_voltage_raw.txt', header=None, delim_whitespace=True).values
assert X.shape[1] == 128, "Input data must have 128 features per row"

# 🧼 Normalize safely
mean = np.mean(X, axis=0)
std = np.std(X, axis=0)
std[std == 0] = 1.0  # Avoid division by zero for constant columns
X_norm = (X - mean) / std

# 🧠 Define an expressive autoencoder
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(128,)),
    tf.keras.layers.Dense(96, activation='relu'),
    tf.keras.layers.Dense(48, activation='relu'),
    tf.keras.layers.Dense(96, activation='relu'),
    tf.keras.layers.Dense(128, activation='linear')
])
model.compile(optimizer='adam', loss='mse')

# 🏋️ Train the model
history = model.fit(X_norm, X_norm, epochs=100, batch_size=8, verbose=1)

# 🧪 Analyze reconstruction error
recon = model.predict(X_norm)
errors = np.mean(np.square(X_norm - recon), axis=1)
print(f"Reconstruction Error → min: {errors.min():.6f}, max: {errors.max():.6f}, mean: {errors.mean():.6f}")

# 💾 Save artifacts
model.save('voltage.keras')
np.savez('voltage_stats.npz', mean=mean, std=std)
print("✅ Model trained and saved successfully.")