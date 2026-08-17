import joblib
import pandas as pd
import common as cmn

from sklearn.model_selection import train_test_split
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC, SVR
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, mean_absolute_error
from sklearn.calibration import CalibratedClassifierCV

DIR_DATA_FILE = f"datasets/pressure_training({cmn.dir_dataset}).csv"
DIR_MODEL_FILE = f"models/pressure_svm({cmn.dir_dataset}).pkl"
SPD_DATA_FILE = f"datasets/speed_training({cmn.spd_dataset}).csv"
SPD_MODEL_FILES = {
    "forward": f"models/fwd_spd_svr({cmn.spd_dataset}).pkl",
    "backward": f"models/bwd_spd_svr({cmn.spd_dataset}).pkl",
    "strafe_left": f"models/sl_spd_svr({cmn.spd_dataset}).pkl",
    "strafe_right": f"models/sr_spd_svr({cmn.spd_dataset}).pkl"
}

feature_columns = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)])


# ===== DIRECTION MODEL ======

dir_data = pd.read_csv(DIR_DATA_FILE)

X_dir = dir_data[feature_columns]
y_dir = dir_data["direction"]

print("DIRECTION MODEL")
print("===================")
print(f"Total samples: {len(dir_data)}")
print(f"Samples per class:\n{y_dir.value_counts()}\n")


X_train, X_test, y_train, y_test = train_test_split(X_dir, y_dir, test_size=0.2, random_state=42, stratify=y_dir)


dir_model = Pipeline([
    ("scaler", StandardScaler()),
    ("svm", CalibratedClassifierCV(SVC(kernel="rbf", C=10, gamma="scale"), ensemble=False))
])


dir_model.fit(X_train, y_train)

dir_predictions = dir_model.predict(X_test)

labels = ["forward", "backward", "strafe_left", "strafe_right", "none"]

accuracy = accuracy_score(y_test, dir_predictions)

print(f"Accuracy: {accuracy * 100:.2f}%\n")

cr = classification_report(y_test, dir_predictions, labels=labels, target_names=labels)

print(f"Classification report:\n{cr}")

cm = confusion_matrix(y_test, dir_predictions, labels=labels)

print("Rows = actual, columns = predicted")
print(f"Confusion matrix:\n{cm}")

joblib.dump(dir_model, DIR_MODEL_FILE)

print(f"\nSaved direction model to: {DIR_MODEL_FILE}\n")


# disp = ConfusionMatrixDisplay(
#     confusion_matrix=cm,
#     display_labels=labels
# )
# disp.plot(cmap=plt.cm.Blues)
# plt.show()


# ===== SPEED MODELS ======

spd_data = pd.read_csv(SPD_DATA_FILE)

print("\nSPEED MODELS")
print("===================")
print(f"Total speed samples: {len(spd_data)}\n")


for direction, model_file in SPD_MODEL_FILES.items():
    direction_data = spd_data[(spd_data["direction"] == direction) | (spd_data["direction"] == "none")]

    if len(direction_data) < 2:
        print(f"{direction}: not enough samples")
        continue

    X_speed = direction_data[feature_columns]
    y_speed = direction_data["speed"]

    print(direction)
    print(f"Samples: {len(direction_data)}")
    print("Speed sample counts:")
    print(y_speed.value_counts().sort_index())

    X_train, X_test, y_train, y_test = train_test_split(X_speed, y_speed, test_size=0.2, random_state=42, stratify=y_speed)

    spd_model = Pipeline([
        ("scaler", StandardScaler()),
        ("svr", SVR(kernel="linear", C=1))
    ])

    spd_model.fit(X_train, y_train)
    train_predictions = spd_model.predict(X_speed)

    # print("50% samples:")
    # print(train_predictions[y_speed.to_numpy() == 0.5])

    # print("100% samples:")
    # print(train_predictions[y_speed.to_numpy() == 1.0])

    spd_predictions = spd_model.predict(X_test)
    # print("Actual speeds:")
    # print(y_test.to_numpy())

    # print("Predicted speeds:")
    # print(spd_predictions)

    # print(
    #     f"Prediction range: "
    #     f"{spd_predictions.min():.3f} - {spd_predictions.max():.3f}"
    # )

    mae = mean_absolute_error(y_test, spd_predictions)

    print(f"Mean absolute error: {mae:.3f}")

    joblib.dump(spd_model, model_file)

    print(f"Saved: {model_file}\n")


print("Finished training all models.")