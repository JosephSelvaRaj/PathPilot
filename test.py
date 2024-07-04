import numpy as np
import pandas as pd
from sklearn.feature_selection import SelectKBest, f_classif
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report

# Load and preprocess the data
data = pd.read_csv('C:\\Users\\josep\\Documents\\Github Repo\\PathPilot\\PathPilot\\MASTERDATA240CIRCLE.TXT', header=None)
data.rename(columns={data.columns[-1]: 'Label'}, inplace=True)
data = data[data['Label'].isin(['F', 'L', 'R'])]
data.reset_index(drop=True, inplace=True)
X = data.iloc[:, :-1]
y = data.iloc[:, -1]

label_encoder = LabelEncoder()
y = label_encoder.fit_transform(y)

# Split the data
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Function to find the optimal k
def find_optimal_k(X_train, y_train, X_test, y_test, max_k, step=10):
    best_accuracy = 0
    best_k = 0

    for k in range(10, max_k + 1, step):
        k_best = SelectKBest(score_func=f_classif, k=k)
        X_train_selected = k_best.fit_transform(X_train, y_train)
        X_test_selected = k_best.transform(X_test)

        clf = RandomForestClassifier(max_depth=6, random_state=42)
        clf.fit(X_train_selected, y_train)
        y_pred = clf.predict(X_test_selected)
        
        accuracy = accuracy_score(y_test, y_pred)
        print(f'k={k}, Accuracy={accuracy}')

        if accuracy > best_accuracy:
            best_accuracy = accuracy
            best_k = k

    return best_k, best_accuracy

# Determine the optimal k
max_k = X_train.shape[1]
optimal_k, best_accuracy = find_optimal_k(X_train, y_train, X_test, y_test, max_k)

print(f'Optimal k: {optimal_k}, Best Accuracy: {best_accuracy}')