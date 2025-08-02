/*
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <custom_messages/msg/optimal_point.hpp>
#include <custom_messages/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/circumference.hpp>


#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <limits> 



custom_messages::msg::Map map;

double raggio = 0.4;
int num_punti = 64;
int num_static_optimalities = 8;

double field_of_vision_z = 0.3;
double tollerance = 0.05; // quanto si tollera che superi il campo visivo

void generaCirconferenzeOttimale(custom_messages::msg::Object& plant);
void verificaAreaLavoro(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map);
void points_inside_other_objects(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map);
std::vector<custom_messages::msg::Point>  calculate_centers(custom_messages::msg::Object& plant);
void modify_map(custom_messages::msg::Map& map);



class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node")
    {
        // Configurazione QoS per il subscriber (latching)
        rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
        sub_qos.transient_local(); // Imposta la durability a TRANSIENT_LOCAL usando il metodo

        // Crea il subscriber per il topic 'Boing'
        map_subscriber_ = this->create_subscription<custom_messages::msg::Map>(
            "Boing",
            sub_qos,
            std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1)
        );

        // Configurazione QoS per il publisher (latching)
        rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
        pub_qos.transient_local();

        // Crea il publisher per il topic 'reworked_map'
        map_publisher_ = this->create_publisher<custom_messages::msg::Map>("reworked_map", pub_qos);
    }

private:
    rclcpp::Subscription<custom_messages::msg::Map>::SharedPtr map_subscriber_;
    rclcpp::Publisher<custom_messages::msg::Map>::SharedPtr map_publisher_;

    void mapCallback(const custom_messages::msg::Map::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ricevuta mappa da Boing, inizio rielaborazione...");
        map = *msg; // Crea una copia della mappa ricevuta
        modify_map(map);
        map_publisher_->publish(map);
        RCLCPP_INFO(this->get_logger(), "Mappa rielaborata e pubblicata su reworked_map.");
    }
};


void generaCirconferenzeOttimale(custom_messages::msg::Object& plant) {
    if (num_static_optimalities <= 0) {
        std::cerr << "Errore: num_static_optimalities deve essere maggiore di zero.\n";
        return; // Esci dalla funzione per evitare la divisione per zero
    }
    //std::cout << 0 << std::endl;
    int steps = num_punti / num_static_optimalities;  // quanti punti saltare per settare l'ottimalità piena (0/1) 
    std::vector<custom_messages::msg::Point>  plant_centers = calculate_centers(plant);
    double angolo_incremento = 2.0 * M_PI / num_punti;

    std::cout << plant_centers.size() << std::endl;

    for (auto target_center : plant_centers){
        
        std::vector<custom_messages::msg::OptimalPoint> circumference_size(num_punti);
        custom_messages::msg::Circumference trajectry;
        trajectry.circumference = circumference_size;

        
        bool max_optimality = true;


        for (int j = 0; j < num_punti; ++j) {
            double angolo = j * angolo_incremento;
            trajectry.circumference[j].x = target_center.x + raggio * cos(angolo);
            trajectry.circumference[j].y = target_center.y + raggio * sin(angolo);
            trajectry.circumference[j].z = target_center.z;

            double angolo_gradi = fmod(angolo * 180.0 / M_PI, 360.0);
            if (angolo_gradi < 0) angolo_gradi += 360.0;

            // setta le ottimalità base
            if (j % steps == 0){
                if (max_optimality){
                    trajectry.circumference[j].optimality = 1.0;
                    max_optimality = false;
                } else {
                    trajectry.circumference[j].optimality = 0.0;
                    max_optimality = true;
                }
            } else {
                trajectry.circumference[j].optimality = -1.0;
            }
        }
        // Secondo Loop: Interpolazione
        for (int j = 0; j < num_punti; ++j) {
            if (trajectry.circumference[j].optimality == -1.0) {
                // Trova l'indice del punto "chiave" precedente
                int prev_key_index = (j / steps) * steps;
                if (prev_key_index < 0) prev_key_index = 0;

                // Trova l'indice del punto "chiave" successivo
                int next_key_index = ((j / steps) + 1) * steps;
                if (next_key_index >= num_punti) next_key_index = 0; // Gestisci la chiusura del cerchio

                // Calcola la distanza tra i punti "chiave"
                int distance = next_key_index - prev_key_index;
                if (distance < 0) distance += num_punti; // Gestisci la chiusura del cerchio

                // Calcola la distanza del punto corrente dal punto "chiave" precedente
                int current_distance = j - prev_key_index;
                if (current_distance < 0) current_distance += num_punti; // Gestisci la chiusura del cerchio

                // Esegui l'interpolazione lineare
                if (distance > 0) {
                    double weight = static_cast<double>(current_distance) / distance;
                    trajectry.circumference[j].optimality = (1.0 - weight) * trajectry.circumference[prev_key_index].optimality + weight * trajectry.circumference[next_key_index].optimality;
                } else {
                    // Caso in cui i punti "chiave" coincidono (dovrebbe accadere solo se num_punti < 8)
                    trajectry.circumference[j].optimality = trajectry.circumference[prev_key_index].optimality;
                }
            }
        }
        //std::cout << plant.possible_trajectories.size() << std::endl;
        plant.possible_trajectories.push_back(trajectry);
    }
}

void verificaAreaLavoro(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map) {  // verifica se i punti sono raggiungibili
    custom_messages::msg::BoundingBox ws_limits = map.work_space;
    custom_messages::msg::Point low = ws_limits.low_left;
    custom_messages::msg::Point top = ws_limits.top_right;
    
    for (auto& trajectory : plant.possible_trajectories){
        for (auto& punto : trajectory.circumference) { // auto indica che il compilatore capirà automaticamente il tipo di variabile
            if (punto.x < low.x || punto.x > top.x ||
                punto.y < low.y || punto.y > top.y ||
                punto.z < low.z || punto.z > top.z) {
                punto.optimality = -2.0;
            }
        }
    }

}

void points_inside_other_objects(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map) { // verifica che i punti non siano dentro altri oggetti (anche se stesso)
    
    for (auto& object : map.objects){
        for (auto& trajectory : plant.possible_trajectories) {
            //if (&object != &plant) {
                for (auto& punto : trajectory.circumference) {
                    custom_messages::msg::Point low = object.shape.low_left;
                    custom_messages::msg::Point top = object.shape.top_right;
                    if (punto.x >= low.x && punto.x <= top.x &&
                        punto.y >= low.y && punto.y <= top.y &&
                        punto.z >= low.z && punto.z <= top.z) {
                        punto.optimality = -3.0;
                    }
                }
           // }
        }
    }
}

std::vector<custom_messages::msg::Point>  calculate_centers(custom_messages::msg::Object& plant) {
    custom_messages::msg::Point low_left = plant.shape.low_left;
    custom_messages::msg::Point top_right = plant.shape.top_right;
    std::cout << "xx :" << low_left.x << " yy:" << low_left.y << " zz:" << top_right.x << std::endl;

    double height = top_right.z - low_left.z;
    std::vector<custom_messages::msg::Point> centers;

    if (height <= field_of_vision_z + tollerance) {
        // Un solo centro
        custom_messages::msg::Point center;
        center.x = low_left.x + (top_right.x - low_left.x) / 2.0;
        center.y = low_left.y + (top_right.y - low_left.y) / 2.0;
        center.z = low_left.z + (1 + 0.25) * field_of_vision_z;
        centers.push_back(center);
    } else {
        // Calcola il numero di divisioni necessarie
        int num_divisions = height / field_of_vision_z;
        if (height - (num_divisions * field_of_vision_z) > tollerance) { // Considera un piccolo epsilon per confronti floating-point
            num_divisions++;  // il +1 sta perche tronchiamo via una parte che sarà quella finale < di field_of_vision_y
        }


        for (int i = 0; i < num_divisions; i++) {
            custom_messages::msg::Point center;
            center.x = low_left.x + (top_right.x - low_left.x) / 2.0;
            center.y = low_left.y + (top_right.y - low_left.y) / 2.0;

            if (i != num_divisions -1){
                center.z = low_left.z + (i + 0.5) * field_of_vision_z;
            } else {
                center.z = low_left.z + ((i + 1 ) + 0.25) * field_of_vision_z; // l'ultimo cerchio è rialzato rispetto alla pianta
            }
            std::cout << "x :" << center.x << " y:" << center.y << " z:" << center.z << std::endl;
            centers.push_back(center);
        }
    }
    return centers;

}

void modify_map(custom_messages::msg::Map& map){
    for (auto& object : map.objects){
        //std::cout << object.target << std::endl;
        if (object.target){
            //std::cout << 1 << std::endl;
            generaCirconferenzeOttimale(object);
            verificaAreaLavoro(object, map);
            points_inside_other_objects(object, map);
        }
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>()); // Usa rclcpp::spin per far girare il nodo e ricevere i callback

   

    const int grid_size = 41;
    char grid[grid_size][grid_size];

    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            grid[i][j] = ' ';
        }
    }

    double scale = (double)(grid_size - 1) / (2.0 * raggio);
    int center_grid = grid_size / 2;


    custom_messages::msg::Object punti_ottimali = map.objects[0];

    //std::cout <<map.objects.size() << std::endl;
    std::cout <<map.objects[0].possible_trajectories.size() << std::endl;

    for (auto& circumference : punti_ottimali.possible_trajectories){
        for (const auto& punto : circumference.circumference) {
            int grid_x = static_cast<int>(round(center_grid + (punto.x) * scale));
            int grid_y = static_cast<int>(round(center_grid + (punto.y) * scale));
    
            if (grid_x >= 0 && grid_x < grid_size && grid_y >= 0 && grid_y < grid_size) {
                int optimalita_int;
                if (punto.optimality < 0){
                    optimalita_int = 0;
                } else {
                    optimalita_int = static_cast<int>(round(punto.optimality * 9)); // Scala da 0 a 9
                }
    
                if (optimalita_int <= 9) {
                    grid[grid_y][grid_x] = optimalita_int + '0'; // Converte l'int in char
                }
            }
            std::cout << "x: " << punto.x << ", y: " << punto.y << ", z: " << punto.z << ", ottimalità: " << punto.optimality << std::endl;
        }
            
    
        std::cout << "Visualizzazione 2D (Numero = Ottimalità * 9):\n";
        for (int i = grid_size - 1; i >= 0; --i) {
            for (int j = 0; j < grid_size; ++j) {
                std::cout << grid[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }
    rclcpp::shutdown();

    return 0;
}

*/


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <custom_messages/msg/optimal_point.hpp>
#include <custom_messages/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/circumference.hpp>


#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <limits>


custom_messages::msg::Map map;

double raggio = 0.3;
int num_punti = 64;
// int num_static_optimalities = 8; // Not used in the binary optimality model

double field_of_vision_z = 0.2;
double tollerance = 0.05; // quanto si tollera che superi il campo visivo

void generaCirconferenzeOttimale(custom_messages::msg::Object& plant);
void verificaAreaLavoro(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map);
void points_inside_other_objects(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map);
std::vector<custom_messages::msg::Point>  calculate_centers(custom_messages::msg::Object& plant);
void modify_map(custom_messages::msg::Map& map);



class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node")
    {
        // Configurazione QoS per il subscriber (latching)
        rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
        sub_qos.transient_local(); // Imposta la durability a TRANSIENT_LOCAL usando il metodo

        // Crea il subscriber per il topic 'Boing'
        map_subscriber_ = this->create_subscription<custom_messages::msg::Map>(
            "Boing",
            sub_qos,
            std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1)
        );

        // Configurazione QoS per il publisher (latching)
        rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
        pub_qos.transient_local();

        // Crea il publisher per il topic 'reworked_map'
        map_publisher_ = this->create_publisher<custom_messages::msg::Map>("reworked_map", pub_qos);
    }

private:
    rclcpp::Subscription<custom_messages::msg::Map>::SharedPtr map_subscriber_;
    rclcpp::Publisher<custom_messages::msg::Map>::SharedPtr map_publisher_;

    void mapCallback(const custom_messages::msg::Map::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ricevuta mappa da Boing, inizio rielaborazione...");
        map = *msg; // Crea una copia della mappa ricevuta
        modify_map(map);
        map_publisher_->publish(map);
        const int grid_size = 41;
        char grid[grid_size][grid_size];

        for (int i = 0; i < grid_size; ++i) {
            for (int j = 0; j < grid_size; ++j) {
                grid[i][j] = ' ';
            }
        }

        double scale = (double)(grid_size - 1) / (2.0 * raggio);
        int center_grid = grid_size / 2;


        custom_messages::msg::Object punti_ottimali = map.objects[0];

        //std::cout <<map.objects.size() << std::endl;
        std::cout <<map.objects[0].possible_trajectories.size() << std::endl;

        for (auto& circumference : punti_ottimali.possible_trajectories){
            for (const auto& punto : circumference.circumference) {
                int grid_x = static_cast<int>(round(center_grid + (punto.x) * scale));
                int grid_y = static_cast<int>(round(center_grid + (punto.y) * scale));

                if (grid_x >= 0 && grid_x < grid_size && grid_y >= 0 && grid_y < grid_size) {
                    if (punto.optimality == 1.0) {
                        grid[grid_y][grid_x] = '1';
                    } else {
                        grid[grid_y][grid_x] = '0';
                    }
                }
                std::cout << "x: " << punto.x << ", y: " << punto.y << ", z: " << punto.z << ", ottimalità: " << punto.optimality << std::endl;
            }


            std::cout << "Visualizzazione 2D (1 = Raggiungibile, 0 = Non Raggiungibile):\n";
            for (int i = grid_size - 1; i >= 0; --i) {
                for (int j = 0; j < grid_size; ++j) {
                    std::cout << grid[i][j] << " ";
                }
                std::cout << std::endl;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Mappa rielaborata e pubblicata su reworked_map.");
    }
};


void generaCirconferenzeOttimale(custom_messages::msg::Object& plant) {
    // num_static_optimalities is not relevant for binary optimality (0 or 1)
    std::vector<custom_messages::msg::Point>  plant_centers = calculate_centers(plant);
    double angolo_incremento = 2.0 * M_PI / num_punti;

    std::cout << plant_centers.size() << std::endl;

    for (auto target_center : plant_centers){

        std::vector<custom_messages::msg::OptimalPoint> circumference_size(num_punti);
        custom_messages::msg::Circumference trajectry;
        trajectry.circumference = circumference_size;

        for (int j = 0; j < num_punti; ++j) {
            double angolo = j * angolo_incremento;
            trajectry.circumference[j].x = target_center.x + raggio * cos(angolo);
            trajectry.circumference[j].y = target_center.y + raggio * sin(angolo);
            trajectry.circumference[j].z = target_center.z;
            trajectry.circumference[j].optimality = 1.0; // Initially set all points as optimal (1)
        }
        plant.possible_trajectories.push_back(trajectry);
    }
}

void verificaAreaLavoro(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map) {  // verifica se i punti sono raggiungibili
    custom_messages::msg::BoundingBox ws_limits = map.work_space;
    custom_messages::msg::Point low = ws_limits.low_left;
    custom_messages::msg::Point top = ws_limits.top_right;

    for (auto& trajectory : plant.possible_trajectories){
        for (auto& punto : trajectory.circumference) { // auto indica che il compilatore capirà automaticamente il tipo di variabile
            // If a point is already marked as non-optimal, don't change it back to optimal
            if (punto.optimality != 0.0) {
                if (punto.x < low.x || punto.x > top.x ||
                    punto.y < low.y || punto.y > top.y ||
                    punto.z < low.z || punto.z > top.z) {
                    punto.optimality = 0.0; // Mark as non-optimal
                }
            }
        }
    }

}

void points_inside_other_objects(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map) { // verifica che i punti non siano dentro altri oggetti (anche se stesso)

    for (auto& object : map.objects){
        for (auto& trajectory : plant.possible_trajectories) {
            // Check if the current object being iterated through is the plant itself.
            // If it is, we usually don't want to mark points inside the plant as "non-optimal" due to the plant's own bounding box.
            // The original comment "if (&object != &plant)" suggests this intent, but it was commented out.
            // For a binary 0/1 optimality, it generally means "can the robot reach this point without colliding with ANYTHING".
            // If a point is inside the plant, and the plant is an obstacle *to itself* in this context, then it should be marked 0.
            // If the plant is NOT an obstacle to itself, then the check below should only apply to *other* objects.
            // For now, assuming it applies to all objects to be conservative (i.e., a point inside *any* object is not reachable).
            // To make it ignore the plant itself, uncomment the `if (&object != &plant)` line.
            // if (&object != &plant) {
                for (auto& punto : trajectory.circumference) {
                    // If a point is already marked as non-optimal, don't change it back to optimal
                    if (punto.optimality != 0.0) {
                        custom_messages::msg::Point low = object.shape.low_left;
                        custom_messages::msg::Point top = object.shape.top_right;
                        if (punto.x >= low.x && punto.x <= top.x &&
                            punto.y >= low.y && punto.y <= top.y &&
                            punto.z >= low.z && punto.z <= top.z) {
                            punto.optimality = 0.0; // Mark as non-optimal
                        }
                    }
                }
            // }
        }
    }
}

std::vector<custom_messages::msg::Point>  calculate_centers(custom_messages::msg::Object& plant) {
    custom_messages::msg::Point low_left = plant.shape.low_left;
    custom_messages::msg::Point top_right = plant.shape.top_right;
    std::cout << "xx :" << low_left.x << " yy:" << low_left.y << " zz:" << top_right.x << std::endl;

    double height = top_right.z - low_left.z;
    std::vector<custom_messages::msg::Point> centers;

    if (height <= field_of_vision_z + tollerance) {
        // Un solo centro
        custom_messages::msg::Point center;
        center.x = low_left.x + (top_right.x - low_left.x) / 2.0;
        center.y = low_left.y + (top_right.y - low_left.y) / 2.0;
        center.z = low_left.z - (1 + 0.2) * field_of_vision_z;
        centers.push_back(center);
    } else {
        // Calcola il numero di divisioni necessarie
        int num_divisions = height / field_of_vision_z;
        if (height - (num_divisions * field_of_vision_z) > tollerance) { // Considera un piccolo epsilon per confronti floating-point
            num_divisions++;  // il +1 sta perche tronchiamo via una parte che sarà quella finale < di field_of_vision_y
        }


        for (int i = 0; i < num_divisions; i++) {
            custom_messages::msg::Point center;
            center.x = low_left.x + (top_right.x - low_left.x) / 2.0;
            center.y = low_left.y + (top_right.y - low_left.y) / 2.0;

            if (i != num_divisions -1){
                center.z = low_left.z + (i + 0.5) * field_of_vision_z;
            } else {
                center.z = low_left.z + ((i + 1 ) + 0.25) * field_of_vision_z; // l'ultimo cerchio è rialzato rispetto alla pianta
            }
            std::cout << "x :" << center.x << " y:" << center.y << " z:" << center.z << std::endl;
            centers.push_back(center);
        }
    }
    return centers;

}

void modify_map(custom_messages::msg::Map& map){
    for (auto& object : map.objects){
        //std::cout << object.target << std::endl;
        if (object.target){
            //std::cout << 1 << std::endl;
            generaCirconferenzeOttimale(object);
            verificaAreaLavoro(object, map);
            points_inside_other_objects(object, map);
        }
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // This must be called after your test code if you want the node to spin and receive messages.
    // If you only want to run the static test, you can keep it commented out.
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();

    return 0;
}