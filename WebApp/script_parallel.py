import json
import time
import threading
import os

# Percorso dei file
FILE_PATH = os.path.join('data', 'matrices.json')
PROCESSED_FILE_PATH = os.path.join('data', 'processed_matrices.json')
file_lock = threading.Lock()

def process_matrix(matrix):
    # Funzione fittizia per elaborare la matrice
    print("Elaborazione della matrice:", matrix)
    # Aggiungi qui la logica di elaborazione
    # Ad esempio, calcoliamo la trasposta
    transposed = [list(row) for row in zip(*matrix)]
    print("Matrice trasposta:", transposed)
    return transposed

def read_and_process_matrices():
    while True:
        with file_lock:
            try:
                # Legge le matrici dal file
                if not os.path.exists(FILE_PATH):
                    # Se il file non esiste, aspetta e riprova
                    time.sleep(1)
                    continue

                with open(FILE_PATH, 'r') as f:
                    data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                # Se il file non esiste o è vuoto, aspetta e riprova
                time.sleep(1)
                continue

            if not data:
                # Se non ci sono matrici da processare, aspetta e riprova
                time.sleep(1)
                continue

            # Prende la prima matrice dalla lista
            matrix = data.pop(0)

            # Aggiorna il file con le matrici rimanenti
            with open(FILE_PATH, 'w') as f:
                json.dump(data, f)

        # Elabora la matrice fuori dal blocco con il lock
        result = process_matrix(matrix)

        # Salva il risultato in un altro file (opzionale)
        save_processed_matrix(matrix, result)

        # Attende un breve intervallo prima di controllare nuovamente
        time.sleep(1)

def save_processed_matrix(original, result):
    with file_lock:
        try:
            # Legge le matrici elaborate esistenti dal file
            with open(PROCESSED_FILE_PATH, 'r') as f:
                processed_data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            # Se il file non esiste o è vuoto, inizializza una lista vuota
            processed_data = []

        # Aggiunge la matrice originale e il risultato
        processed_data.append({
            'original': original,
            'result': result
        })

        # Salva la lista aggiornata nel file
        with open(PROCESSED_FILE_PATH, 'w') as f:
            json.dump(processed_data, f)
        print(" [x] Risultato salvato nel file delle matrici elaborate")

if __name__ == '__main__':
    read_and_process_matrices()
