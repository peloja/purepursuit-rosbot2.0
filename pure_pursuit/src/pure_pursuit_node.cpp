#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <string.h>
#include <tf/tf.h>
#include <fstream>

ros::Publisher cmd_vel_pub_;
ros::Subscriber pose_sub_;
ros::Subscriber path_sub_;
nav_msgs::Path path_;
geometry_msgs::Pose robot_pose;
geometry_msgs::Point _target_point;

class PurePursuitController
{
public:
    // constructor de la clase, carga los parámetro del controlador 
    PurePursuitController(double lookahead_distance, double max_linear_velocity, double max_angular_velocity) :
    _lookahead_distance(lookahead_distance),
    _max_linear_velocity(max_linear_velocity),
    _max_angular_velocity(max_angular_velocity)
    {
    }

    // calculo de la velocidad para llegar al punto objetivo
    geometry_msgs::Twist getCmdVel(const geometry_msgs::Pose &robot_pose)
    {
        // primero obtenemos el punto mas cercano con la funcion getWaypoint
        int nearest_waypoint = getWaypoint(robot_pose);
        //ROS_INFO("PUNTO MAS CERCANO=%d", nearest_waypoint);

        // a partir del punto mas cercano obtenemos el punto objetivo
        _target_point = getTarget(nearest_waypoint, robot_pose);

        // por ultimo obtenemos las velocidades angular y lineal para llegar a ese punto 
        geometry_msgs::Twist cmd_vel;

        // si el punto mas cercano es igual al final de la trayectoria global y la distancia al punto final es menor a un umbral, paramos el robot
        if (nearest_waypoint == path_.poses.size() - 1 || getDist(path_.poses.back().pose.position, robot_pose.position) < 0.3)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        } 

        // en caso contrario calculamos las velocidades en funcion del angulo entre el robot y el objetivo
        else 
        {
            cmd_vel.linear.x = _max_linear_velocity;
            double alpha = std::atan2(_target_point.y - robot_pose.position.y, _target_point.x - robot_pose.position.x) - robot_pose.orientation.z;        
            cmd_vel.angular.z = _max_angular_velocity * std::sin(alpha);
        }
        return cmd_vel;
    }


private:
    
    // calculo de la distancia entre dos puntos como el modulo del vector entre estos
    double getDist(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // devuelve el indice correspondiente al punto de la trayectoria global mas cercano al robot
    int getWaypoint(const geometry_msgs::Pose &robot_pose)
    {
        double min_distance = std::numeric_limits<double>::max();
        int nearest_waypoint = -1;

        // para cada punto de la trayectoria global calcula la distancia al robot y devuelve el indice del punto mas cercano
        for (int i = 0; i < path_.poses.size(); i++)
        {
            double distance = getDist(path_.poses[i].pose.position, robot_pose.position);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_waypoint = i;
            }
        }
        return nearest_waypoint;
    }

    // a partir la pose del robot y el indice al punto mas cercano de la trayectoria global, devuelve un punto objetivo
    geometry_msgs::Point getTarget(int nearest_waypoint, const geometry_msgs::Pose &robot_pose)
    {
        geometry_msgs::Point target_point;

        // para cada punto de la trayectoria desde el mas cercano al final de esta, calcula la distancia al robot
        for (int i = nearest_waypoint; i < path_.poses.size(); i++)     
        {
            double distance = getDist(path_.poses[i].pose.position, robot_pose.position);

            // el primer punto para el que la distancia al robot sea mayor que Lookahead distance se establece como punto objetivo
            if (distance > _lookahead_distance)
            {
                target_point = path_.poses[i-1].pose.position;
                break;
            }
        }
        return target_point;
    }

private:
    double _lookahead_distance;
    double _max_linear_velocity;
    double _max_angular_velocity;
};


// funcion Callback para el topic /odom
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    
    double roll, pitch, yaw;   
    tf::Quaternion orientation;

    // recibimos la orientación en forma de cuaternion así que la pasamos a una matrix 3x3 con los angulos de euler correspondientes
    tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // actualizamos la variable global robot_pose con los valores recibidos de odometria
    robot_pose.position = msg->pose.pose.position;

    // respecto a la orientacion, al tratarse de un robot terrestre solo nos interesa el angulo de guiñada (yaw) 
    robot_pose.orientation.z = yaw; 
    
    // guardamos la información de odometría en un archivo de texto. asi como la informacion de la trayectoria global 
    std::ofstream file("/home/arosa/Escritorio/odom.txt", std::ios_base::app);
    if (file.is_open()) 
    {   
        // ROS_INFO(" Punto objetivo:(%f, %f). Odometria:(%f, %f, %f)",_target_point.x, _target_point.y, robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.z );
        file << ros::Time::now() << ", " << _target_point.x << ", " << _target_point.y << ", " << robot_pose.position.x << ", " << robot_pose.position.y << ", " << robot_pose.orientation.z << ", "<< std::endl;
        file.close();
    }
}

// funcion Callback para el topic /path
void pathCallback(const nav_msgs::PathConstPtr &msg)
{
    // almacenamos el vector de waypoints en la variable _path
    path_ = *msg;    
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");    
    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);

    // configuramos el NodeHandle para publicar las velocidades calculadas en el topic /cmd_vel
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // configuramos el NodeHamdle para suscribirnos a los topics /odom y /path de donde recibimos pa pose del robot y la trayectoria global
    pose_sub_ = nh_.subscribe("/odom", 10, odomCallback);    
    path_sub_ = nh_.subscribe("/path", 10, pathCallback);   

    // inicializamos valores por defecto para los parametros del controlador
    double lookahead_distance{0.2};
    double max_linear_velocity{0.2};
    double max_angular_velocity{3.0};
    
    // si es posible actualizamos el valor de estos parametros con los aportados en el .launch
    ros::param::get("~lookahead_distance", lookahead_distance);
    ros::param::get("~max_linear_velocity", max_linear_velocity);
    ros::param::get("~max_angular_velocity", max_angular_velocity);
    
    // sacamos por pantalla los parámetros del controlador
    ROS_INFO("Controller params: \n");
    ROS_INFO("Lookahead distance= %f    Max linear velocity= %f     Max angular velocity= %f \n", lookahead_distance, max_linear_velocity, max_angular_velocity);

    // inicializamos el controlador con los parametros establecidos
    PurePursuitController control(lookahead_distance, max_linear_velocity, max_angular_velocity); 
     
    // comenzamos el bucle de funcionamiento dle sistema
    while (ros::ok())
    {
        // calculamos las velocidades a aplicar con la funcion getCmdVel() del controlador
        geometry_msgs::Twist cmd_vel = control.getCmdVel(robot_pose);

        // si aun no se ha recibido una trayectoria a seguir, dejamos las velocidades a 0
        if(path_.poses.empty())
        {
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
        }

        // publicamos las velocidades 
        cmd_vel_pub_.publish(cmd_vel);

        // dejamos el sistema en bucle a la espera de recibir mas informacion (nueva pose o trayectoria)
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

