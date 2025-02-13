import socket
import threading

# Configuration
LOCAL_HOST = '0.0.0.0'  # Listen on all interfaces
LOCAL_PORT = 9991        # Port to listen on
REMOTE_HOST = 'localhost'  # Destination host
REMOTE_PORT = 10077         # Destination port

def handle_client(client_socket, remote_host, remote_port):
    """Handles the client connection and forwards data."""
    try:
        remote_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        remote_socket.connect((remote_host, remote_port))

        # Start forwarding data
        threading.Thread(target=forward_data, args=(client_socket, remote_socket)).start()
        threading.Thread(target=forward_data, args=(remote_socket, client_socket)).start()
    except Exception as e:
        print(f"Connection error: {e}")
        client_socket.close()

def forward_data(source, destination):
    """Forwards data between two sockets."""
    try:
        while True:
            data = source.recv(4096)
            if not data:
                break
            destination.sendall(data)
    except Exception as e:
        print(f"Forwarding error: {e}")
    finally:
        source.close()
        destination.close()

def start_forwarder():
    """Starts the port forwarder."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((LOCAL_HOST, LOCAL_PORT))
    server.listen(5)
    print(f"[*] Listening on {LOCAL_HOST}:{LOCAL_PORT}, forwarding to {REMOTE_HOST}:{REMOTE_PORT}")

    while True:
        client_socket, addr = server.accept()
        print(f"[*] Connection received from {addr}")
        threading.Thread(target=handle_client, args=(client_socket, REMOTE_HOST, REMOTE_PORT)).start()

if __name__ == "__main__":
    start_forwarder()

    import pandas as pd

    pd.DataFrame.iterrows()
