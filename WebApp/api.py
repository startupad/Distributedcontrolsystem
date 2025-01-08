# api.py
import json
import os
import threading

file_lock = threading.Lock()

def ensure_directory_exists(file_path):
    """Crea la directory per il file se non esiste."""
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

def save_matrix_processed(file_path, matrix):
    """Salva la matrice nel file specificato."""
    with file_lock:
        try:
            ensure_directory_exists(file_path)  # Assicura che la directory esista
            with open(file_path, 'w') as f:
                json.dump(matrix, f)
            print(f" [x] Matrice salvata nel file: {file_path}")
        except Exception as e:
            print(f"Errore durante il salvataggio della matrice: {e}")
            
def get_priority_matrix(file_path):
    with file_lock:
        try:
            with open(file_path, 'r') as f:
                matrix = json.load(f)
            print(f" [x] Matrice caricata dal file: {file_path}")
            return matrix
        except Exception as e:
            print(f"Errore durante il caricamento della matrice: {e}")
            return None
    
            
# api.js
# api.py
from threading import Lock

simulation_end = False
simulation_end_lock = Lock()

def set_simulation_end(value):
    with simulation_end_lock:
        global simulation_end
        simulation_end = value
        print(f"simulation_end è stato impostato a: {simulation_end}")

def get_simulation_end():
    with simulation_end_lock:
        return simulation_end

list1_lock = Lock()
list2_lock = Lock()
list3_lock = Lock()

# Le liste da modificare
list1 = [0.0, 0.0, 0.0]
list2 = [0.0, 0.0, 0.0]
list3 = [0.0, 0.0, 0.0]

def set_coordinates(lista1, lista2, lista3):
    """
    Funzione che setta le tre liste in modo thread-safe.

    :param list1: Lista di 3 float da aggiornare
    :param list2: Lista di 3 float da aggiornare
    :param list3: Lista di 3 float da aggiornare
    """
    with list1_lock, list2_lock, list3_lock:
        global list1, list2, list3
        list1[:] = lista1 
        list2[:] = lista2
        list3[:] = lista3

def get_coordinates():
    """
    Funzione che legge le tre liste in modo thread-safe.
    Restituisce i valori correnti di list1, list2 e list3.
    """
    with list1_lock, list2_lock, list3_lock:
        # Restituisce una copia delle liste per evitare modifiche accidentali
        return list1[:], list2[:], list3[:]