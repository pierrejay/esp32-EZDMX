#!/usr/bin/env python3
import asyncio
import websockets
import json
import time
import random
import argparse
import logging

# Configuration
logging.basicConfig(
    level=logging.DEBUG,  # Changer en DEBUG pour plus de détails
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

DEFAULT_IP = "192.168.0.228"
DEFAULT_PORT = 80
DURATION = 30  # Animation duration in seconds
MAX_STEP = 1   # Maximum value change per step

# Maximum values for each channel
MAX_CH = [40, 40, 25]  # Max values for channels 1, 2, 3

async def run_fade_animation(uri):
    print(f"Démarrage de l'animation aléatoire progressive sur les canaux 1, 2 et 3...")
    print(f"L'animation va durer {DURATION} secondes. Appuyez sur Ctrl+C pour arrêter.")

    try:
        async with websockets.connect(uri) as websocket:
            logger.info(f"Connecté à {uri}")
            
            # Attendre un peu pour s'assurer que la connexion est bien établie
            await asyncio.sleep(0.5)
            
            # Valeurs actuelles et cibles pour chaque canal
            current_vals = [0, 0, 0]
            target_vals = [random.randint(0, max_val) for max_val in MAX_CH]

            print(f"Cibles initiales: {target_vals}")
            
            end_time = time.time() + DURATION
            last_status = time.time()

            while time.time() < end_time:
                # Mettre à jour les valeurs pour chaque canal
                for i in range(3):
                    if current_vals[i] == target_vals[i]:
                        target_vals[i] = random.randint(0, MAX_CH[i])
                    elif current_vals[i] < target_vals[i]:
                        step = random.randint(1, MAX_STEP)
                        current_vals[i] = min(current_vals[i] + step, target_vals[i])
                    else:
                        step = random.randint(1, MAX_STEP)
                        current_vals[i] = max(current_vals[i] - step, target_vals[i])

                # Remplacer le format actuel par un tableau simple
                dmx_data = [0] * 512  # Initialiser tous les canaux à 0
                # ATTENTION: Les canaux DMX commencent à 1, donc on utilise les indices 0-2 pour les canaux 1-3
                dmx_data[0] = current_vals[0]  # Canal 1
                dmx_data[1] = current_vals[1]  # Canal 2
                dmx_data[2] = current_vals[2]  # Canal 3

                # Ajouter des logs pour déboguer
                message = json.dumps(dmx_data, separators=(',', ':'))  # Compact JSON
                logger.debug(f"Envoi des données DMX: {message[:50]}...")
                await websocket.send(message)
                
                # Mise à jour de statut (une fois par seconde)
                if time.time() - last_status >= 1:
                    time_left = int(end_time - time.time())
                    print(f"Valeurs: {current_vals} | Cibles: {target_vals} | Temps restant: {time_left}s")
                    last_status = time.time()

                await asyncio.sleep(0.1)

            # Fondu final
            print("Animation terminée, fondu final...")
            for step in range(10, -1, -1):
                factor = step / 10
                fade_vals = [int(val * factor) for val in current_vals]
                
                dmx_data = [0] * 512
                dmx_data[0] = fade_vals[0]
                dmx_data[1] = fade_vals[1]
                dmx_data[2] = fade_vals[2]
                
                await websocket.send(json.dumps(dmx_data))
                await asyncio.sleep(0.05)

            # Pour la commande clear, utiliser le même format que HTTP
            clear_command = {"clear": True}
            await websocket.send(json.dumps(clear_command))

    except websockets.exceptions.ConnectionClosedError as e:
        logger.error(f"Connexion fermée: {e}")
        print("Vérifiez que le serveur est en marche et que l'URL WebSocket est correcte.")
        
    except Exception as e:
        logger.error(f"Erreur inattendue: {e}")
        
    except KeyboardInterrupt:
        print("\nAnimation interrompue, effacement des canaux...")
        try:
            clear_command = {"clear": True}
            async with websockets.connect(uri) as websocket:
                await websocket.send(json.dumps(clear_command))
        except:
            pass

    print("Animation terminée!")

async def test_connection(uri):
    try:
        async with websockets.connect(uri) as websocket:
            logger.info(f"Connecté à {uri}")
            
            # Attendre un peu pour s'assurer que la connexion est bien établie
            await asyncio.sleep(0.5)
            
            # Test simple : allumer le premier canal
            test_data = [255] + [0] * 511  # Premier canal à 255, les autres à 0
            message = json.dumps(test_data, separators=(',', ':'))  # Compact JSON
            logger.info(f"Envoi test: {message[:50]}...")
            await websocket.send(message)
            await asyncio.sleep(1)
            
            # Éteindre
            test_data = [0] * 512
            message = json.dumps(test_data, separators=(',', ':'))  # Compact JSON
            logger.info("Envoi extinction...")
            await websocket.send(message)
            
    except Exception as e:
        logger.error(f"Erreur: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Animation DMX WebSocket')
    parser.add_argument('--ip', default=DEFAULT_IP, help='Adresse IP du serveur DMX')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='Numéro de port')
    parser.add_argument('--test', action='store_true', help='Mode test simple')
    
    args = parser.parse_args()
    
    uri = f"ws://{args.ip}:{args.port}/dmx/stream"
    
    if args.test:
        asyncio.run(test_connection(uri))
    else:
        asyncio.run(run_fade_animation(uri))