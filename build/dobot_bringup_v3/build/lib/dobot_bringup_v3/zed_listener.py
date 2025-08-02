import subprocess
import time
import sys

def start_rosbridge_websocket():
    """Avvia il nodo rosbridge_websocket in background."""
    try:
        # Lancia il nodo rosbridge_websocket in background
        process = subprocess.Popen(
            ["ros2", "run", "rosbridge_server", "rosbridge_websocket"],  # Rimosso --ros-args se non necessario
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,  # Per ottenere output come stringhe
            encoding='utf-8' #Specifico l'encoding
        )
        print("Avviato rosbridge_websocket in background.")
        return process  # Restituisce l'oggetto Popen per poterlo gestire in seguito

    except FileNotFoundError:
        print("Errore: Il comando 'ros2' non è stato trovato. Assicurati che ROS 2 sia installato e che l'ambiente sia correttamente configurato.", file=sys.stderr)
        return None
    except Exception as e:
        print(f"Errore imprevisto durante l'esecuzione di rosbridge_websocket: {e}", file=sys.stderr)
        return None

def main():
    """Funzione principale dello script."""
    rosbridge_process = start_rosbridge_websocket()
    if rosbridge_process:
        time.sleep(5)  # Aggiungi un breve ritardo per permettere al server di avviarsi

        # Opzionale: Monitorare il processo e gestire eventuali errori
        if rosbridge_process.poll() is not None:
            print(f"rosbridge_websocket è terminato inaspettatamente con codice: {rosbridge_process.returncode}", file=sys.stderr)
            # Puoi leggere l'output di errore per ulteriori dettagli:
            # error_output = rosbridge_process.stderr.read()
            # print(f"Errore: {error_output}", file=sys.stderr)

if __name__ == "__main__":
    main()