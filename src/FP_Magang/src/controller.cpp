#include <ros/ros.h>

#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <FP_Magang/PC2PC.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

using namespace std;

class Controller
{
private:
    ros::NodeHandle nh;
    ros::Publisher pc2bs_pub;
    ros::Subscriber bs2pc_sub;
    ros::Subscriber pc2pc_sub;

    float interval = 0;

    void updatePos(float enc_left, float enc_right)
    {
        robot[0] = (enc_right * sin(45 * M_PI / 180)) + (enc_left * sin(45 * M_PI / 180));
        robot[1] = (enc_right * cos(45 * M_PI / 180)) - (enc_left * cos(45 * M_PI / 180));
    }

    float sigmoid(float interval)
    {
        return 1.0 / (1.0 + exp(-2.5 * (2 * interval - 1))); // GA NGERTI
    }

    char getKey() // VIA CHAT-GPT
    {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        new_tio.c_cc[VMIN] = 1;
        new_tio.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval timeout = {0, 0};

        char ch;
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0)
        {
            read(STDIN_FILENO, &ch, 1);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

        return ch;
    }

    //------------------------------------------------------------------------ [ HANDLERS ]

    void bs2pcHandler(const FP_Magang::BS2PC &msg)
    {
        if (msg.status == 1) {
            manualSteer = true;
        }
        if (prevMsg.status != msg.status && (msg.status == 2 || msg.status == 4)) {
            step = 0;
        }
        if (msg.status == 3) {
            target[0] = msg.tujuan_x;
            target[1] = msg.tujuan_y;
        }

        ROS_INFO("STATUS %d", msg.status);

        prevMsg = msg;
        updatePos(msg.enc_left, msg.enc_right);
    }

    void pc2pcHandler(const FP_Magang::PC2PC &msg)
    {
        ball[0] = msg.follow_x;
        ball[1] = msg.follow_y;

        if (prevMsg.status == 2) {

            // TARGET TO BALL
            target[0] = ball[0];
            target[1] = ball[1];
        }
        else if (prevMsg.status == 4) {

            float x = ball[0] - robot[0];
            float y = ball[1] - robot[1];

            target[0] = ball[0] - 100 * cos(atan2(y, x));
            target[1] = ball[1] - 100 * sin(atan2(y, x));

            step = 1;
        }
    }

public:
    FP_Magang::BS2PC prevMsg;
    FP_Magang::PC2BS msg;

    float robot[2] = {0, 0};
    float ball[2] = {0, 0};
    float target[2] = {0, 0};

    bool grabbing = false;
    bool manualSteer = false;

    int step = 0;
    
    Controller()
    {
        pc2bs_pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 10);
        bs2pc_sub = nh.subscribe("/bs2pc", 5, &Controller::bs2pcHandler, this);
        pc2pc_sub = nh.subscribe("/pc2pc", 5, &Controller::pc2pcHandler, this);
    }


    void start()
    {
        float svDeg = 0;

        ros::Rate rate(30);
        while (ros::ok())
        {
            ros::spinOnce();
            if (prevMsg.status == 1)
            {
                manualSteer = true;
                Steer();
                updateMotor(0, 0, 0);
            }
            else if (prevMsg.status == 2)
            {
                manualSteer = false;
                msg.bola_x = ball[0];
                msg.bola_y = ball[1];
                pc2bs_pub.publish(msg);
                ChaseImage();
            }
            else if (prevMsg.status == 3)
            {
                manualSteer = false;
                ChasePoint();
            }
            else if (prevMsg.status == 4)
            {
                manualSteer = false;
                ChaseAndRotate(svDeg);
            }
            rate.sleep();
        }
    }

    void launchBall(float power)
    {
        FP_Magang::PC2BS msg;
        float ct = prevMsg.th;
        float new_x = ball[0] + (power * sin(ct * M_PI / 180));
        float new_y = ball[1] + (power * cos(ct * M_PI / 180));

        // 27.7430 926.866219 31.4311 627.356796
        if (new_x + 27.7430 < 0) {
            new_x = -27.7430;
        }
        else if (new_x - 926.866219 > 0) {
            new_x = 926.866219;
        }

        if (new_y + 31.4311 < 0) {
            new_y = -31.4311;
        }
        else if (new_y - 627.356796 > 0) {
            new_y = 627.356796;
        }

        ball[0] = new_x;
        ball[1] = new_y;

        msg.bola_x = new_x;
        msg.bola_y = new_y;

        pc2bs_pub.publish(msg);

        ros::spinOnce();
    }

    void updateMotor(float x, float y, float theta)
    {
        FP_Magang::PC2BS msg;
        if (manualSteer)
        {
            float ct = -prevMsg.th;
            float sinus = sin(ct * M_PI / 180);
            float cosinus = cos(ct * M_PI / 180);

            float cx = x;
            float cy = y;

            x = (cx * cosinus) + (cy * -sinus);
            y = (cx * sinus) + (cy * cosinus);
        }

        if (!manualSteer && interval < 1)
        {
            interval += 0.02;
        }
        else
        {
            interval = 1;
        }

        int fieldWidth = 1016, fieldHeight = 716;
        float batasX = fieldWidth - 50 * 2;
        float batasY = fieldHeight - 50 * 2;
        if (((robot[0] + (x / 50)) < 0) || ((robot[0] + (x / 50)) > batasX))
        {
            x = 0;
        }
        if (((robot[1] + (y / 50)) < 0) || ((robot[1] + (y / 50)) > batasY))
        {
            y = 0;
        }

        (round(x) == 0) ? x = 0 : x = x;
        (round(y) == 0) ? y = 0 : y = y;
        (round(theta) == 0) ? theta = 0 : theta = theta;

        if ((round(x) == 0) && (round(y) == 0) && (round(theta / 10) == 0))
        {
            interval = 0;
            if (prevMsg.status == 4)
            {
                step++;
            }
            return;
        }

        msg.motor1 = ((x * 2 / 3) + (y * 0) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor2 = ((x * -1 / 3) + (y * (sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor3 = ((x * -1 / 3) + (y * -(sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);

        msg.bola_x = ball[0];
        msg.bola_y = ball[1];
        pc2bs_pub.publish(msg);
        ros::spinOnce();
    }

    void Steer()
    {
        char key = getKey();

        switch (key)
        {
        case 'w':
            updateMotor(1, 0, 0); break;
        case 's':
            updateMotor(-1, 0, 0); break;
        case 'a':
            updateMotor(0, 1, 0); break;
        case 'd':
            updateMotor(0, -10, 0); break;
        case 'q':
            updateMotor(0, 0, -10); break;
        case 'e':
            updateMotor(0, 0, 10); break;
        case 'f':
            updateMotor(0, 0, 0); break;
        }
    }

    void ChaseImage()
    {
        float dist = sqrt(pow((target[0] - robot[0]), 2) + pow((target[1] - robot[1]), 2));
        if (dist < 30 && step == 1)
        {
            grabbing = true;
            // msg.status = 2;
            // pc2bs_pub.publish(msg);
        }
        else if (dist < 20)
        {
            launchBall(30);
        }
    }

    void ChasePoint()
    {
        float dist = sqrt(pow((target[0] - robot[0]), 2) + pow((target[1] - robot[1]), 2));
        if (dist < 10 && step == 1)
        {
            grabbing = true;
            // msg.status = 2;
            // pc2bs_pub.publish(msg);
        }
        else if (dist < 20)
        {
            launchBall(10);
        }
    }

    void ChaseAndRotate(float svDeg)
    {
        // float dist = sqrt(pow((target[0] - robot[0]), 2) + pow((target[1] - robot[1]), 2));
        // if (dist < 100)
        // {

        // }
        // NYERAH !!!!!!!!!!!!!!!!!!!
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    Controller controller;
    controller.start();

    return 0;
}
