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

file_path = "simulation_end.txt"

# Assicuriamoci che il file esista con un valore predefinito
if not os.path.exists(file_path):
    with open(file_path, "w") as file:
        file.write("False")  # Valore predefinito

# Lock per sincronizzare l'accesso alla variabile
simulation_end_lock = Lock()

def set_simulation_end(value):
    with simulation_end_lock:
        # Scriviamo il valore nel file
        with open(file_path, "w") as file:
            file.write(str(value))  # Salviamo il valore come stringa
        print(f"simulation_end è stato impostato a: {value}")

def get_simulation_end():
    with simulation_end_lock:
        # Leggiamo il valore dal file
        with open(file_path, "r") as file:
            value = file.read().strip()  # Rimuoviamo eventuali spazi bianchi
        return value.lower() == 'true'  # Converto la stringa in un valore booleano

list1_lock = Lock()
list2_lock = Lock()
list3_lock = Lock()

# Le liste da modificare
list1 = [0.0, 0.0, 0.]
list2 = [0.0, 0.0, 0.0]
list3 = [0.0, 0.0, 0.0]
# Nome del file in cui salviamo le liste
file_name = "coordinates.txt"

def save_coordinates_to_file(list1, list2, list3):
    """
    Salva le coordinate in un file di testo.
    """
    with open(file_name, "w") as file:
        # Scriviamo ogni lista su una riga separata
        file.write("list1: " + str(list1) + "\n")
        file.write("list2: " + str(list2) + "\n")
        file.write("list3: " + str(list3) + "\n")

def load_coordinates_from_file():
    """
    Carica le coordinate da un file di testo.
    Se il file non esiste, ritorna liste vuote.
    """
    if os.path.exists(file_name):
        with open(file_name, "r") as file:
            lines = file.readlines()
            # Estraiamo i valori dalle righe del file
            list1 = eval(lines[0].strip().split(": ")[1])
            list2 = eval(lines[1].strip().split(": ")[1])
            list3 = eval(lines[2].strip().split(": ")[1])
            return list1, list2, list3
    else:
        # Se il file non esiste, ritorna liste vuote
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

def set_coordinates(lista1, lista2, lista3):
    """
    Funzione che salva le tre liste in un file di testo in modo thread-safe.

    :param lista1: Lista di 3 float da aggiornare
    :param lista2: Lista di 3 float da aggiornare
    :param lista3: Lista di 3 float da aggiornare
    """
    with list1_lock, list2_lock, list3_lock:
        # Salviamo le nuove liste nel file
        save_coordinates_to_file(lista1, lista2, lista3)

def get_coordinates():
    """
    Funzione che legge le tre liste dal file di testo in modo thread-safe.
    Restituisce i valori correnti di list1, list2 e list3.
    """
    with list1_lock, list2_lock, list3_lock:
        list1, list2, list3 = load_coordinates_from_file()
        return list1, list2, list3