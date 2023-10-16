#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char * * argv)
{
  ros::init(argc, argv, "costmap_node");
  ros::NodeHandle nh;

  // creamos 3 flags para la seleccion del costmap a utilizar y las inicializamos a flase
  bool costmap1{false};
  bool costmap2{false};
  bool costmap3{false};
  
  // actualizamos el valor de estas con los parámetros del .launch, eligiendo desde allí el costmap a usar
  ros::param::get("~costmap1", costmap1);
  ros::param::get("~costmap2", costmap2);
  ros::param::get("~costmap3", costmap3);
  
  // creamos un publisher para el costmap
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);

  // creamos el costmap
  nav_msgs::OccupancyGrid costmap;
  costmap.header.frame_id = "odom";
  costmap.info.resolution = 0.05;
  

  // Costmap 1
  if(costmap1)
  {
    // elegimos el tamaño y origen de coordenadas el costmap
    costmap.info.width = 100;   
    costmap.info.height = 100;
    costmap.info.origin.position.x = 0.0;
    costmap.info.origin.position.y = 0.0;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 0.0;
    costmap.data.resize(costmap.info.width * costmap.info.height);

    // recorremos cada punto del mapa 
    for (unsigned int i = 0; i < costmap.info.width; ++i)
    {
      for (unsigned int j = 0; j < costmap.info.height; ++j)
      {
        double x = costmap.info.origin.position.x + i * costmap.info.resolution;
        double y = costmap.info.origin.position.y + j * costmap.info.resolution;
        int index = j * costmap.info.width + i;
        costmap.data[index] = 0;

        // elegimos la posición de los obstáculos
        float obstacle_x[2] = {1.0, 2.0};
        float obstacle_y[2] = {1.0, 2.0};
        float obstacle2_x[2] = {3.0, 4.0};
        float obstacle2_y[2] = {3.0, 4.0};
        float obstacle3_x[2] = {3.0, 4.0};
        float obstacle3_y[2] = {1.0, 2.0};
        float obstacle4_x[2] = {1.0, 2.0};
        float obstacle4_y[2] = {3.0, 4.0};
        float security_dist = 0.25;

        // para los puntos donde haya un obstaculo pintamos de negro y los margenes de seguridad de gris
        if (x >= obstacle_x[0]-security_dist && x <= obstacle_x[1]+security_dist && y >= obstacle_y[0]-security_dist && y <= obstacle_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle2_x[0]-security_dist && x <= obstacle2_x[1]+security_dist && y >= obstacle2_y[0]-security_dist && y <= obstacle2_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle3_x[0]-security_dist && x <= obstacle3_x[1]+security_dist && y >= obstacle3_y[0]-security_dist && y <= obstacle3_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle4_x[0]-security_dist && x <= obstacle4_x[1]+security_dist && y >= obstacle4_y[0]-security_dist && y <= obstacle4_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }

        if (x >= obstacle_x[0] && x <= obstacle_x[1] && y >= obstacle_y[0] && y <= obstacle_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle2_x[0] && x <= obstacle2_x[1] && y >= obstacle2_y[0] && y <= obstacle2_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle3_x[0] && x <= obstacle3_x[1] && y >= obstacle3_y[0] && y <= obstacle3_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle4_x[0] && x <= obstacle4_x[1] && y >= obstacle4_y[0] && y <= obstacle4_y[1])
        {
          costmap.data[index] = 100;
        }
      }
    }
    // indicamos que el mapa se ha cargado
    ROS_INFO("Costmap 1 loaded \n");
  }
  
  // Costmap 2
  else if(costmap2)
  {
    costmap.info.width = 220;
    costmap.info.height = 220;
    costmap.info.origin.position.x = -0.5;
    costmap.info.origin.position.y = -0.5;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 0.0;
    costmap.data.resize(costmap.info.width * costmap.info.height);

    // Set the costmap data 2
    for (unsigned int i = 0; i < costmap.info.width; ++i)
    {
      for (unsigned int j = 0; j < costmap.info.height; ++j)
      {
        double x = costmap.info.origin.position.x + i * costmap.info.resolution;
        double y = costmap.info.origin.position.y + j * costmap.info.resolution;
        int index = j * costmap.info.width + i;
        costmap.data[index] = 0;

        // generate the borders of the scene
        float borde0_x[2] = { -0.5, 10.5};
        float borde0_y[2] = { -0.5, -0.25};
        float borde1_x[2] = { -0.5, 10.5};
        float borde1_y[2] = { 10.25, 10.5};
        float borde2_x[2] = { -0.5, -0.25};
        float borde2_y[2] = { -0.5, 10.5};
        float borde3_x[2] = { 10.25, 10.5};
        float borde3_y[2] = { -0.5, 10.5};
        
        float obstacle0_x[2] = { 5.75, 6.0};
        float obstacle0_y[2] = { -0.5, 3.5};
        float obstacle1_x[2] = { 5.75, 10.5};
        float obstacle1_y[2] = { 5.5, 5.75};
        
        float obstacle2_x[2] = { 2.5, 7.0};
        float obstacle2_y[2] = { 8.0, 8.25};
        float obstacle3_x[2] = { 2.5, 2.75};
        float obstacle3_y[2] = { 3.0, 8.0};
        float obstacle4_x[2] = { -0.5, 2.75};
        float obstacle4_y[2] = { 3.0, 3.25};
        float security_dist = 0.25;

        // Check if the cell is inside one of the boxes
        if (x >= borde0_x[0]-security_dist && x <= borde0_x[1]+security_dist && y >= borde0_y[0]-security_dist && y <= borde0_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= borde1_x[0]-security_dist && x <= borde1_x[1]+security_dist && y >= borde1_y[0]-security_dist && y <= borde1_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= borde2_x[0]-security_dist && x <= borde2_x[1]+security_dist && y >= borde2_y[0]-security_dist && y <= borde2_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= borde3_x[0]-security_dist && x <= borde3_x[1]+security_dist && y >= borde3_y[0]-security_dist && y <= borde3_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle0_x[0]-security_dist && x <= obstacle0_x[1]+security_dist && y >= obstacle0_y[0]-security_dist && y <= obstacle0_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle1_x[0]-security_dist && x <= obstacle1_x[1]+security_dist && y >= obstacle1_y[0]-security_dist && y <= obstacle1_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle2_x[0]-security_dist && x <= obstacle2_x[1]+security_dist && y >= obstacle2_y[0]-security_dist && y <= obstacle2_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle3_x[0]-security_dist && x <= obstacle3_x[1]+security_dist && y >= obstacle3_y[0]-security_dist && y <= obstacle3_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle4_x[0]-security_dist && x <= obstacle4_x[1]+security_dist && y >= obstacle4_y[0]-security_dist && y <= obstacle4_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }

        if (x >= borde0_x[0] && x <= borde0_x[1] && y >= borde0_y[0] && y <= borde0_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= borde1_x[0] && x <= borde1_x[1] && y >= borde1_y[0] && y <= borde1_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= borde2_x[0] && x <= borde2_x[1] && y >= borde2_y[0] && y <= borde2_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= borde3_x[0] && x <= borde3_x[1] && y >= borde3_y[0] && y <= borde3_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle0_x[0] && x <= obstacle0_x[1] && y >= obstacle0_y[0] && y <= obstacle0_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle1_x[0] && x <= obstacle1_x[1] && y >= obstacle1_y[0] && y <= obstacle1_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle2_x[0] && x <= obstacle2_x[1] && y >= obstacle2_y[0] && y <= obstacle2_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle3_x[0] && x <= obstacle3_x[1] && y >= obstacle3_y[0] && y <= obstacle3_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle4_x[0] && x <= obstacle4_x[1] && y >= obstacle4_y[0] && y <= obstacle4_y[1])
        {
          costmap.data[index] = 100;
        }
      }
    }

    ROS_INFO("Costmap 2 loaded \n");
  }

  // Costmap 3
  else if(costmap3)
  {
    costmap.info.width = 100;   
    costmap.info.height = 100;
    costmap.info.origin.position.x = 0.0;
    costmap.info.origin.position.y = 0.0;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 0.0;
    costmap.data.resize(costmap.info.width * costmap.info.height);

    // Set the costmap data 3
    for (unsigned int i = 0; i < costmap.info.width; ++i)
    {
      for (unsigned int j = 0; j < costmap.info.height; ++j)
      {
        double x = costmap.info.origin.position.x + i * costmap.info.resolution;
        double y = costmap.info.origin.position.y + j * costmap.info.resolution;
        int index = j * costmap.info.width + i;
        costmap.data[index] = 0;

        // Set the position for the two obstacles
        float obstacle2_y[2] = {0.65, 5.0};
        float obstacle2_x[2] = {0.0, 0.5};
        float obstacle3_y[2] = {0.0, 4.0};
        float obstacle3_x[2] = {1.0, 1.35};
        float obstacle4_y[2] = {4.75, 5.0};
        float obstacle4_x[2] = {0.35, 2.0};
        float obstacle5_y[2] = {2.0, 5.0};
        float obstacle5_x[2] = {2.0, 2.35};
        float obstacle6_y[2] = {0.0, 1.35};
        float obstacle6_x[2] = {1.35, 5.0};
        float obstacle7_y[2] = {1.35, 5.0};
        float obstacle7_x[2] = {4.65, 5.0};
        float obstacle8_y[2] = {4.65, 5.0};
        float obstacle8_x[2] = {2.65, 5.0};
        float obstacle9_y[2] = {3.0, 5.0};
        float obstacle9_x[2] = {2.0, 3.0};
        float obstacle10_y[2] = {2.0, 4.35};
        float obstacle10_x[2] = {3.65, 4.0};
        float obstacle11_y[2] = {2.0, 2.35};
        float obstacle11_x[2] = {2.0, 4.0};
        float security_dist = 0.1;

        // Check if the cell is inside one of the boxes
        
        if (x >= obstacle2_x[0]-security_dist && x <= obstacle2_x[1]+security_dist && y >= obstacle2_y[0]-security_dist && y <= obstacle2_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle3_x[0]-security_dist && x <= obstacle3_x[1]+security_dist && y >= obstacle3_y[0]-security_dist && y <= obstacle3_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle4_x[0]-security_dist && x <= obstacle4_x[1]+security_dist && y >= obstacle4_y[0]-security_dist && y <= obstacle4_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle5_x[0]-security_dist && x <= obstacle5_x[1]+security_dist && y >= obstacle5_y[0]-security_dist && y <= obstacle5_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle6_x[0]-security_dist && x <= obstacle6_x[1]+security_dist && y >= obstacle6_y[0]-security_dist && y <= obstacle6_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle7_x[0]-security_dist && x <= obstacle7_x[1]+security_dist && y >= obstacle7_y[0]-security_dist && y <= obstacle7_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        else if (x >= obstacle8_x[0]-security_dist && x <= obstacle8_x[1]+security_dist && y >= obstacle8_y[0]-security_dist && y <= obstacle8_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        
        else if (x >= obstacle9_x[0]-security_dist && x <= obstacle9_x[1]+security_dist && y >= obstacle9_y[0]-security_dist && y <= obstacle9_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        
        else if (x >= obstacle10_x[0]-security_dist && x <= obstacle10_x[1]+security_dist && y >= obstacle10_y[0]-security_dist && y <= obstacle10_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        
        else if (x >= obstacle11_x[0]-security_dist && x <= obstacle11_x[1]+security_dist && y >= obstacle11_y[0]-security_dist && y <= obstacle11_y[1]+security_dist)
        {
          costmap.data[index] = 50;
        }
        
    
        if (x >= obstacle2_x[0] && x <= obstacle2_x[1] && y >= obstacle2_y[0] && y <= obstacle2_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle3_x[0] && x <= obstacle3_x[1] && y >= obstacle3_y[0] && y <= obstacle3_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle4_x[0] && x <= obstacle4_x[1] && y >= obstacle4_y[0] && y <= obstacle4_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle5_x[0] && x <= obstacle5_x[1] && y >= obstacle5_y[0] && y <= obstacle5_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle6_x[0] && x <= obstacle6_x[1] && y >= obstacle6_y[0] && y <= obstacle6_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle7_x[0] && x <= obstacle7_x[1] && y >= obstacle7_y[0] && y <= obstacle7_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle8_x[0] && x <= obstacle8_x[1] && y >= obstacle8_y[0] && y <= obstacle8_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle9_x[0] && x <= obstacle9_x[1] && y >= obstacle9_y[0] && y <= obstacle9_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle10_x[0] && x <= obstacle10_x[1] && y >= obstacle10_y[0] && y <= obstacle10_y[1])
        {
          costmap.data[index] = 100;
        }
        else if (x >= obstacle11_x[0] && x <= obstacle11_x[1] && y >= obstacle11_y[0] && y <= obstacle11_y[1])
        {
          costmap.data[index] = 100;
        }
      }
    }

    ROS_INFO("Costmap 3 loaded \n");
  }

  // si no se indica ningun costmap a utilizar enviamos un mensaje de erro al terminal
  else ROS_ERROR("Costmap can't be loaded");

  // iniciamos el bucle de publicacion del costmap
  ros::Rate rate(10);
  while (ros::ok())
  {
    costmap.header.stamp = ros::Time::now();
    pub.publish(costmap);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
