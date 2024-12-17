from flask import Flask, request, jsonify, render_template, send_from_directory
import os
import threading
import json
import sys
from os.path import abspath, dirname
import matplotlib
matplotlib.use('TkAgg')  
# Aggiungi il percorso di CoppeliaSim_project a sys.path
sys.path.append(abspath(dirname(__file__) + '/../coppeliasim_project'))

# Importa la funzione main dal file main.py per avviare la simulazione di CoppeliaSim
from main import main
import api

app = Flask(__name__)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # La cartella 'web-app'

FILE_PATH = os.path.join(BASE_DIR, '..', 'data', 'matrices.json')
FILE_PATH = os.path.abspath(FILE_PATH)  # Assicurati che il percorso sia assoluto

FILE_PATH_PROCESSED = os.path.join(BASE_DIR, '..', 'data', 'processed_matrices.json')
FILE_PATH_PROCESSED = os.path.abspath(FILE_PATH_PROCESSED)  # Assicurati che il percorso sia assoluto

file_lock = threading.Lock()

# Stato per controllare l'esecuzione della simulazione
simulation_thread = None
simulation_running = threading.Event()

def ensure_directory_exists(file_path):
    """Crea la directory per il file se non esiste."""
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

def save_matrix(matrix):
    with file_lock:
        try:
            ensure_directory_exists(FILE_PATH)  # Assicura che la directory esista
            # Salva la matrice nel file
            with open(FILE_PATH, 'w') as f:
                json.dump(matrix, f)
            print(" [x] Matrice salvata nel file")
        except Exception as e:
            print(f"Errore durante il salvataggio della matrice: {e}")

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
    start_simulation()
    return jsonify({'status': 'Matrice ricevuta con successo'}), 200

@app.route('/start-simulation', methods=['POST'])
def start_simulation():
    global simulation_thread
    print(\
        "Avvio della simulazione. Attendere qualche minuto per il completamento della simulazione.")

    if simulation_running.is_set():
        return jsonify({'error': 'La simulazione è già in esecuzione'}), 400

    def run_simulation():
        try:
            simulation_running.set()
            main()
        except Exception as e:
            print(f"Errore durante l'esecuzione della simulazione: {e}")
        finally:
            simulation_running.clear()

    # Avvia un thread per la simulazione
    simulation_thread = threading.Thread(target=run_simulation, daemon=True)
    simulation_thread.start()
    return jsonify({'status': 'Simulazione avviata con successo'}), 200


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Necessario per proiezioni 3D



@app.route('/get-processed-matrix', methods=['GET'])
def get_processed_matrix():
    simulation_end = api.get_simulation_end() 
    list1_copy, list2_copy, list3_copy = api.get_coordinates()

    x_values = [list1_copy[0], list2_copy[0], list3_copy[0]]
    y_values = [list1_copy[1], list2_copy[1], list3_copy[1]]
    z_values = [list1_copy[2], list2_copy[2], list3_copy[2]]

    if not simulation_end:
        # Restituisci i punti e errore 400
        return jsonify({
            'points': {
                'x': x_values,
                'y': y_values,
                'z': z_values
            },
            'error': 'La simulazione non è ancora terminata'
        }), 400
    else:
        # Assicurati che la cartella `data` esista
        os.makedirs(os.path.dirname(FILE_PATH_PROCESSED), exist_ok=True)

        with file_lock:
            try:
                # Prova ad aprire il file per leggere la matrice
                with open(FILE_PATH_PROCESSED, 'r') as f:
                    data = json.load(f)
                print("Dati letti dalla matrice:", data)
                return jsonify(data)
            except (FileNotFoundError, json.JSONDecodeError) as e:
                print(f"Errore nella lettura del file: {e}")
                return jsonify({'error': 'Nessuna matrice elaborata disponibile'}), 404
  
            
@app.route('/get-texture', methods=['GET'])
def get_texture():
    """Endpoint per restituire l'immagine texture.png."""
    texture_path = os.path.join(BASE_DIR, '..', 'texture.png')
    
    # Verifica che il file esista prima di tentare di inviarlo
    if os.path.exists(texture_path):
        return send_from_directory(os.path.dirname(texture_path), 'texture.png')
    else:
        return jsonify({'error': 'File texture.png non trovato'}), 404


if __name__ == '__main__':
    # Avvia il server Flask sul thread principale
    app.run(debug=True)
