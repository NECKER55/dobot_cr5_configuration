from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Genera la descrizione del lancio del file."""
    return LaunchDescription([
        Node(
            package='cr5_moveit_cpp_demo',  # Sostituisci con il nome del tuo pacchetto
            executable='reworked_map_node',   # Sostituisci con il nome dell'eseguibile del tuo nodo di elaborazione
            name='reworked_map',
            output='screen',
        ),
        # se non ho connessione a un vero ZED, lancio un publisher fake (pubblica su Boing coordinate fittizie)
        Node(
            package='cr5_moveit_cpp_demo',  # Sostituisci con il nome del tuo pacchetto
            executable='fake_zed_node', # Sostituisci con il nome dell'eseguibile del tuo publisher
            name='fake_zed',
            output='screen', # Mostra l'output del nodo nel terminale
        ),
        # Nodo che si sottoscrive al topic matlab_slave per ricevere pose e muovere il robot
        Node(
            package='cr5_moveit_cpp_demo',
            executable='matlab_slave_node',
            name='matlab_slave',
            output='screen',
        ),
    ])