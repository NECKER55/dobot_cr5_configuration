from launch import LaunchDescription           # Classe che descrive un file launch
from launch_ros.actions import Node            # Classe che descrive l'avvio di un nodo ROS

def generate_launch_description():             # Funzione che genera il file launch
    return LaunchDescription([                 # Restituisce la descrizione del file launch
        Node(                                  # Configura l'avvio di un nodo
            package='dobot_bringup_v3',         # Nome del pacchetto ROS in cui si trova il nodo
            executable='dobot_bringup',         # Nome del file eseguibile del nodo
        ),
        Node(                                  # Configura l'avvio di un secondo nodo
            package='dobot_bringup_v3',         # Ancora il pacchetto 'dobot_bringup_v3'
            executable='feedback',             # Nome del secondo file eseguibile da lanciare
        ),
    ])
