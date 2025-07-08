from flask import Flask, request, jsonify, render_template
import numpy as np
from tensorflow import keras
from datetime import datetime
import os

app = Flask(__name__)

# Load trained model
model = keras.models.load_model("voltage.keras")

# Load or regenerate normalization stats
try:
    mean = np.load("mean.npy")
    std = np.load("std.npy")
    print("✅ Loaded mean/std from .npy files.")
except FileNotFoundError:
    print("⚠️  mean.npy or std.npy not found. Regenerating from real_voltage_raw.txt...")
    X = np.loadtxt("real_voltage_raw.txt", dtype=np.float32)
    mean = np.mean(X, axis=0)
    std = np.std(X, axis=0)
    np.save("mean.npy", mean)
    np.save("std.npy", std)
    print("📦 mean.npy and std.npy saved.")

# Frozen threshold based on validated training pass
threshold = 1.57

# For web log display
results = []

@app.route("/predict", methods=["POST"])
def predict():
    try:
        data = request.get_json(force=True)
        arr = np.array(data["input"], dtype=np.float32)

        if arr.shape != (128,):
            return jsonify({"error": f"Expected shape (128,), got {arr.shape}"}), 400

        norm_arr = (arr - mean) / (std + 1e-8)
        output = model.predict(norm_arr.reshape(1, -1), verbose=0)
        error = float(np.mean((norm_arr - output[0]) ** 2))
        is_outlier = error > threshold

        result = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "output": output.flatten().astype(float).tolist(),
            "error": error,
            "anomaly": is_outlier
        }

        results.append(result)
        results[:] = results[-100:]

        return jsonify(result)

    except Exception as e:
        return jsonify({"error": f"{type(e).__name__}: {str(e)}"}), 500

@app.route("/")
def index():
    return render_template("index.html", results=results[::-1])

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)