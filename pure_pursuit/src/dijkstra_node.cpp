#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <unordered_map>
#include <algorithm>

ros::Publisher path_pub;

// struct con los puntos del mapa, agrega las funcionalidades de los operadores ==, < y >
struct Point 
{
    int x, y;
    bool operator==(const Point& other) const 
    {
        return x == other.x && y == other.y;
    }
    bool operator<(const Point& other) const 
    {
        return x < other.x && y < other.y;
    }
    bool operator>(const Point& other) const 
    {
        return x > other.x && y > other.y;
    }
};

// funcion hash para usar un punto como clave en un diccionario
struct PointHasher 
{
    std::size_t operator()(const Point& p) const 
    {
        return p.x * 31 + p.y;
    }
};

// calculo de la distancia euclideana entre dos puntos
double getDistance(const Point &a, const Point &b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// funcion callback para recibir el mapa de costes
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) 
{
    // extraemos el tamaño y resolucion del mapa del mensaje (argumento de la funcion)
    int width = map_msg->info.width;
    int height = map_msg->info.height;
    float resolution = map_msg->info.resolution;

    // inicializamos las coordenedas de origen y destino a un valor por defcto
    int start_x{0}, start_y{0}, goal_x{50}, goal_y{50};

    // si es posible actualizamos las coordenadas de origen y destino con los parametros del .launch
    ros::param::get("~start_x", start_x);
    ros::param::get("~start_y", start_y);
    ros::param::get("~goal_x", goal_x);
    ros::param::get("~goal_y", goal_y);
    
    // cargamos las coordenadas en puntos (struct Point)
    Point start{ start_x, start_y};
    Point goal{ goal_x, goal_y};

    // extraemos el mapa del mensaje
    const std::vector<int8_t>& data = map_msg->data;

    // creamos una tabla de distancias y una tabla de padres para almacenar informacion con la cual reconstruir el camino
    std::unordered_map<Point, double, PointHasher> distance;
    distance[start] = 0;
    std::unordered_map<Point, Point, PointHasher> parent;

    // configuramos una cola de prioridad para explorar de manera ordenada los puntos 
    std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>>> pq;
    pq.push({0, start});

    // creamos una matriz de puntos vecinos
    std::vector<Point> neighbors = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0 , -1},          {0 , 1},
        {1 , -1}, {1 , 0}, {1 , 1}
    };

    // bucle para calcular las distancias entre los puntos del mapa, ademas de la visitabilidad de estos
    // se ejecutará siempre que la cola de prioridad no esté vacía y en ella se almacenan los puntos a visitar
    while (!pq.empty()) {

        // extraemos el punto de la cola con la distancia mas pequeña
        auto cur = pq.top().second;

        // y extraemos la distancia acumulada 
        double cur_distance = pq.top().first;
        pq.pop();

        // si el punto actual es igual al objetivo se considera el camino encontrado y se sale del bucle
        if (cur == goal) 
        {
            break;
        }

        // elegido un punto iteramos sobre cada uno de sus vecinos
        for (const auto& offset : neighbors) 
        {
            Point next{cur.x + offset.x, cur.y + offset.y};

            // comprobamos si el punto esta dentro de los limites del mapa
            if (next.x >= 0 && next.x < width && next.y >= 0 && next.y < height) 
            {
                // comprobamos que el punto no es un obstaculo
                int index = next.y * width + next.x;
                if (data[index] == 0) 
                {
                    // si se cumple lo anterior calculamos la nueva distancia acumulada
                    double new_distance= cur_distance + getDistance(cur, next);

                    // si la nueva distancia es menor que la actual o si el punto no está en la lista se agrega el punto vecino a la cola de prioridad
                    if (distance.find(next) == distance.end() || new_distance < distance[next]) 
                    {
                        distance[next] = new_distance;
                        parent[next] = cur;
                        pq.push({new_distance, next});
                    }
		        }
	        }
	    }
    }
    

    // reconstruimos el camino mas corto comenzando por el punto destino
    nav_msgs::Path path;
    path.header = map_msg->header;
    path.poses.resize(0);
    Point current = goal;
    
    // iteramos hasta que el punto actual sea el punto de origen
    while (!(current == start)) 
    {   
        // se itera siguiendo las conexiones de los puntos y añadiendolos a la trayectoria (variable path)
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = current.x * resolution + map_msg->info.origin.position.x;
        pose.pose.position.y = current.y * resolution + map_msg->info.origin.position.y;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);

        current = parent[current];
    }

    // invertimos el orden del camino para que esté en la dirección correcta
    std::reverse(path.poses.begin(), path.poses.end());

    // publicamos el camino encontrado en el topic /path
    path_pub.publish(path);

}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "map_navigation");
    ros::NodeHandle nh;

    // configuramos el NodeHandle para susbscribirnos al topic /map a travel del cual recibimos el mapa
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);

    // configuramos el NodeHandle para publicar la trayectoria global en el topic /path
    path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

    ros::spin();

    return 0;
}

