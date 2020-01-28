#include <ros/ros.h>
#include <termios.h>
#include <geometry_msgs/PoseStamped.h>

#define ARROW 1
#define COORDINATE 2

void printMenu();
void publishArrow(char c, ros::Publisher pub, geometry_msgs::PoseStamped msg);
char getch();

int mode;
int x = 0;
int y = 0;
int z = 2;

int main(int argc, char **argv){
    ros::init(argc, argv, "keyboard_interface");
    ros::NodeHandle nh;

    ros::Publisher local_waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint_", 10);
    ros::Rate loop_rate(100);
    geometry_msgs::PoseStamped msg;

    do{
        printMenu();
    } while (!mode);

    if (mode == ARROW){
        std::cout << "\nw:↑, a:←, d: →, s: ↓, q: ↶, e: ↷, u: ascend, l: descent" << std::endl;
        std::cout << "current location is (" << x << ", " << y << ", " << z << ")" << std::endl;
        while(ros::ok()){
            int c = 0;
            c = getch();
            if(c != NULL){
                publishArrow(c, local_waypoint_pub, msg);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    else if (mode == COORDINATE)
    {
        while (ros::ok())
        {
            std::cout << "current location is (" << x << ", " << y << ", " << z << ")" << std::endl;
            std::cout << "x: ";
            std::cin >> x;
            std::cout << "y: ";
            std::cin >> y;
            std::cout << "z: ";
            std::cin >> z;

            msg.header.stamp = ros::Time::now();
            msg.pose.position.x = x;
            msg.pose.position.y = y;
            msg.pose.position.z = z;
            local_waypoint_pub.publish(msg);
        }
    }

    // while(ros::ok()){
    //     msg.header.stamp = ros::Time::now();
    //     msg.pose.position.x = 0;
    //     msg.pose.position.y = 1;
    //     msg.pose.position.z = 1;
    //     local_waypoint_pub.publish(msg);

    //     char a = getch();
    //     std::cout << a << std::endl;
    //     if(a != NULL)
    //         ROS_INFO("%c", a);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}

void printMenu(){

    ROS_INFO("----------------Select Mode----------------");
    ROS_INFO("          1. Arrow, 2. COORDINATE          ");
    ROS_INFO("-------------------------------------------");

    std::cin >> mode;

    if (mode != ARROW && mode != COORDINATE)
    {
        ROS_INFO("Type 1 or 2");
        mode = 0;
        return;
    }
}

void publishArrow(char c, ros::Publisher pub, geometry_msgs::PoseStamped msg){
    switch (c)
    {
    case 'w':
        x++;
        break;
    case 'a':
        y--;
        break;
    case 's':
        x--;
        break;
    case 'd':
        y++;
        break;
    case 'q':
        
        break;
    case 'e':
        
        break;
    case 'u':
        z++;
        break;
    case 'l':
        z--;
        break;
    default:

        break;
    }
    if (z <= 0)
        z = 0;
    if (z >= 3)
        z = 3;

    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    pub.publish(msg);
    std::cout << "current location is (" << x << ", " << y << ", " << z << ")" << std::endl;
}

char getch(){
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;

    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");
    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0)
        ;
    //ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len);

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}