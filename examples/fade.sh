#!/bin/bash

# Configuration
IP="192.168.0.228"  # Votre adresse IP
PORT=80
DELAY=0.0002         # Délai ultra-court entre les requêtes
DURATION=30         # Durée de l'animation en secondes
MAX_STEP=1          # Variation maximale par étape (plus petit = transition plus douce)

MAX_CH1=40          # Valeur maximale pour chaque canal
MAX_CH2=40          # Valeur maximale pour chaque canal
MAX_CH3=25          # Valeur maximale pour chaque canal

# S'assurer que DMX est en cours d'exécution
curl -s -X POST "http://$IP:$PORT/dmx/set" \
  -H "Content-Type: application/json" \
  -d '{"run": true}'

echo "Démarrage de l'animation aléatoire fluide sur les canaux 1, 2 et 3..."
echo "L'animation durera $DURATION secondes. CTRL+C pour arrêter avant."

# Initialiser les valeurs actuelles et cibles
val1=0
val2=0
val3=0

target1=$((RANDOM % (MAX_CH1 + 1)))
target2=$((RANDOM % (MAX_CH2 + 1)))
target3=$((RANDOM % (MAX_CH3 + 1)))

echo "Premières cibles: $target1, $target2, $target3"

# Calculer le temps de fin
end_time=$(($(date +%s) + DURATION))

# Boucle principale
while [ $(date +%s) -lt $end_time ]; do
    # Si une valeur a atteint sa cible, générer une nouvelle cible
    if [ $val1 -eq $target1 ]; then
        target1=$((RANDOM % (MAX_CH1 + 1)))
    fi
    if [ $val2 -eq $target2 ]; then
        target2=$((RANDOM % (MAX_CH2 + 1)))
    fi
    if [ $val3 -eq $target3 ]; then
        target3=$((RANDOM % (MAX_CH3 + 1)))
    fi
    
    # Calculer les pas pour chaque canal (direction vers la cible)
    if [ $val1 -lt $target1 ]; then
        step1=$((RANDOM % MAX_STEP + 1))
        val1=$((val1 + step1))
        if [ $val1 -gt $target1 ]; then val1=$target1; fi
    elif [ $val1 -gt $target1 ]; then
        step1=$((RANDOM % MAX_STEP + 1))
        val1=$((val1 - step1))
        if [ $val1 -lt $target1 ]; then val1=$target1; fi
    fi
    
    if [ $val2 -lt $target2 ]; then
        step2=$((RANDOM % MAX_STEP + 1))
        val2=$((val2 + step2))
        if [ $val2 -gt $target2 ]; then val2=$target2; fi
    elif [ $val2 -gt $target2 ]; then
        step2=$((RANDOM % MAX_STEP + 1))
        val2=$((val2 - step2))
        if [ $val2 -lt $target2 ]; then val2=$target2; fi
    fi
    
    if [ $val3 -lt $target3 ]; then
        step3=$((RANDOM % MAX_STEP + 1))
        val3=$((val3 + step3))
        if [ $val3 -gt $target3 ]; then val3=$target3; fi
    elif [ $val3 -gt $target3 ]; then
        step3=$((RANDOM % MAX_STEP + 1))
        val3=$((val3 - step3))
        if [ $val3 -lt $target3 ]; then val3=$target3; fi
    fi
    
    # Afficher un retour périodique (une fois par seconde environ)
    if [ $((RANDOM % 1000)) -eq 0 ]; then
        time_left=$((end_time - $(date +%s)))
        echo "Valeurs: $val1, $val2, $val3 | Cibles: $target1, $target2, $target3 | Temps restant: $time_left s"
    fi
    
    # Envoyer les valeurs en une seule requête
    curl -s -X POST "http://$IP:$PORT/dmx/set" \
      -H "Content-Type: application/json" \
      -d "[{\"channel\": 1, \"value\": $val1}, {\"channel\": 2, \"value\": $val2}, {\"channel\": 3, \"value\": $val3}]"
    
    sleep $DELAY
done

# Fondu vers zéro à la fin
echo "Animation terminée, fondu final..."
for ((i=10; i>=0; i--)); do
    factor=$((i * 10))
    new_val1=$((val1 * factor / 100))
    new_val2=$((val2 * factor / 100))
    new_val3=$((val3 * factor / 100))
    
    curl -s -X POST "http://$IP:$PORT/dmx/set" \
      -H "Content-Type: application/json" \
      -d "[{\"channel\": 1, \"value\": $new_val1}, {\"channel\": 2, \"value\": $new_val2}, {\"channel\": 3, \"value\": $new_val3}]"
    
    sleep 0.05
done

# Effacer tous les canaux à la fin
curl -s -X POST "http://$IP:$PORT/dmx/set" \
  -H "Content-Type: application/json" \
  -d '{"clear": true}'

echo "Animation terminée!"%   