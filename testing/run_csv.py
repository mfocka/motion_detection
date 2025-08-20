import csv
import math
import sys

def read_csv(path):
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('usage: python run_csv.py data.csv')
        sys.exit(1)
    rows = read_csv(sys.argv[1])
    print('rows', len(rows))

