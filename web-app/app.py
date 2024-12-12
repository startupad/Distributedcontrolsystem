from flask import Flask, request, jsonify, render_template
import json
import threading
import os
import time
app = Flask(__name__)

# Percorso del file dove verranno salvate le matrici
FILE_PATH = 'data/matrices.json'
FILE_PATH_PROCESSED = 'data/processed_matrices.json'
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
        
        
def save_matrix_processed(matrix):
    with file_lock:
        try:
            # Legge le matrici esistenti dal file
            with open(FILE_PATH_PROCESSED, 'w') as f:
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
    # Crea una matrice vuota 6x6 piena di zeri
    empty_matrix = [[0 for _ in range(6)] for _ in range(6)]
    max_attempts = 60  # Timeout dopo 60 secondi
    attempts = 0

    # Assicurati che la cartella `data` esista
    os.makedirs(os.path.dirname(FILE_PATH_PROCESSED), exist_ok=True)

    with file_lock:
        # Se il file esiste, cancellalo per garantirne uno nuovo
        if os.path.exists(FILE_PATH_PROCESSED):
            os.remove(FILE_PATH_PROCESSED)
        
        # Crea un nuovo file con la matrice vuota
        with open(FILE_PATH_PROCESSED, 'w') as f:
            json.dump(empty_matrix, f)
            print("File 'processed_matrices.json' creato con una matrice vuota.")

    # Inizia a controllare il file ogni secondo
    while attempts < max_attempts:
        time.sleep(1)  # Aspetta 1 secondo prima di controllare di nuovo
        attempts += 1

        with file_lock:
            try:
                # Prova ad aprire il file per leggere la matrice
                with open(FILE_PATH_PROCESSED, 'r') as f:
                    data = json.load(f)
                
                # Stampa i dati letti per verificare cosa viene letto dal file
                print("Dati letti dalla matrice:", data)

                # Controlla se la matrice ha cambiato valori rispetto a quella vuota
                if data != empty_matrix:
                    print("Matrice modificata rilevata:", data)
                    return jsonify(data)
                    
            except (FileNotFoundError, json.JSONDecodeError) as e:
                print(f"Errore nella lettura del file: {e}")
                continue  # Ignora l'errore e continua il ciclo

    # Se si verifica il timeout dopo 60 tentativi (60 secondi)
    print("Timeout: nessuna modifica rilevata nella matrice entro 60 secondi.")
    return jsonify({'error': 'Timeout waiting for processed matrix'}), 504


if __name__ == '__main__':
    app.run(debug=True)
