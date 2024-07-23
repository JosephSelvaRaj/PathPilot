import numpy as np
import pandas as pd
from sklearn.feature_selection import SelectKBest
from sklearn.feature_selection import f_classif
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
import xgboost as xgb
from sklearn.metrics import accuracy_score, classification_report
from micromlgen import port


data = pd.read_csv('C:\\Users\\josep\\Documents\\Github Repo\\PathPilot\\PathPilot\\MASTERDATA240CIRCLE.TXT', header=None)
data.rename(columns={data.columns[-1]: 'Label'}, inplace=True)
print(f"Label counts before cleaning the data: \n {data['Label'].value_counts()}")
data = data[data['Label'].isin(['F', 'L', 'R'])]
data.reset_index(drop=True, inplace=True)
print(f"Label counts after cleaning the data: \n {data['Label'].value_counts()}")
X = data.iloc[:, :-1]
y = data.iloc[:, -1]

label_encoder = LabelEncoder()
y = label_encoder.fit_transform(y)
label_mapping = dict(zip(label_encoder.classes_, label_encoder.transform(label_encoder.classes_)))
print(f"Label encoding mapping for motor control in Arduino code: {label_mapping}")

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Optimal k value found previously
optimal_k = 180

# Feature selection with the optimal k
k_best = SelectKBest(score_func=f_classif, k=optimal_k)
k_best = k_best.fit(X_train, y_train)

selected_feature_indices = k_best.get_support(indices=True)
print("Format of selected features (Angles) to be copied into Arduino code:\n", selected_feature_indices)


clf = xgb.XGBClassifier(max_depth=6,random_state=42)
clf.fit(X_train.iloc[:, selected_feature_indices], y_train)

y_pred = clf.predict(X_test.iloc[:, selected_feature_indices])

accuracy = accuracy_score(y_test, y_pred)
print(f'Accuracy: {accuracy * 100:.2f}%')

class_names = label_encoder.classes_
report = classification_report(y_test, y_pred, target_names=class_names, zero_division=0)
print('Classification Report:\n', report)

arduino_code = open("xgboost2.h", mode="w+")
arduino_code.write(port(clf))
arduino_code.close()