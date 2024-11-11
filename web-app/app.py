from flask import Flask, request, jsonify, render_template
import json
import threading
import os
import time
app = Flask(__name__)

# Percorso del file dove verranno salvate le matrici
FILE_PATH = os.path.join('data', 'matrices.json')
FILE_PATH_PROCESSED = os.path.join('data', 'processed_matrices.json')
file_lock = threading.Lock()

def save_matrix(matrix):
    with file_lock:
        try:
            # Legge le matrici esistenti dal file
            with open(FILE_PATH, 'w') as f:
                json.dump(matrix, f)
        except (FileNotFoundError, json.JSONDecodeError):
            data = []
        print(" [x] Matrice salvata nel file")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/save-priorities', methods=['POST'])
def save_priorities():
    data = request.get_json()
    if 'priorities' not in data:
        return jsonify({'error': 'Nessuna priorità fornita'}), 400
    priorities = data['priorities']
    save_matrix(priorities)
    return jsonify({'status': 'Matrice ricevuta con successo'}), 200
    
@app.route('/get-processed-matrix', methods=['GET'])
def get_processed_matrix():
    # Create an empty 6x6 matrix filled with zeros
    empty_matrix = [[0 for _ in range(6)] for _ in range(6)]
    max_attempts = 60  # e.g., timeout after 60 seconds
    attempts = 0

    with file_lock:
        # Write the empty matrix to the file
        with open(FILE_PATH_PROCESSED, 'w') as f:
            json.dump(empty_matrix, f)

    # Now start checking the file every second
    while attempts < max_attempts:
        time.sleep(1)  # Wait 1 second before checking again
        attempts += 1

        with file_lock:
            try:
                with open(FILE_PATH_PROCESSED, 'r') as f:
                    data = json.load(f)
                
                # Check if any value in data has changed from zero
                if data != empty_matrix:
                    print(data)
                    return jsonify(data)
                    
            except (FileNotFoundError, json.JSONDecodeError):
                continue  # Ignore errors and continue the loop

    # If timeout occurs
    return jsonify({'error': 'Timeout waiting for processed matrix'}), 504




if __name__ == '__main__':
    app.run(debug=True)
