import csv
import json
import sys

def csv_to_json(csv_path, json_path):
    X = []
    Y = []

    with open(csv_path) as f:
        reader = csv.DictReader(f)

        if 'X' not in reader.fieldnames or 'Y' not in reader.fieldnames:
            raise ValueError("CSV must contain 'X' and 'Y' columns")

        for row in reader:
            X.append(float(row['X']))
            Y.append(float(row['Y']))

    data = {
        "X": X,
        "Y": Y
    }

    with open(json_path, 'w') as f:
        json.dump(
            data,
            f,
            separators=(',', ':'),   # ← 공백 제거
            ensure_ascii=False
        )

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python csv_to_json.py input.csv output.json")
        sys.exit(1)

    csv_to_json(sys.argv[1], sys.argv[2])