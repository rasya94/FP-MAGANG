#include <ros/ros.h>

#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <FP_Magang/PC2PC.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace ros;

class Controller
{
private:

    NodeHandle nh;

    Publisher pubPC2BS;
    Subscriber subBS2PC;
    Subscriber subPC2PC;

    float interval = 0;

    float degToRad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    void updateRobotPos(float enc_left, float enc_right)
    {
        robotPos[0] = (enc_right * sin(45 * M_PI / 180)) + (enc_left * sin(45 * M_PI / 180));
        robotPos[1] = (enc_right * cos(45 * M_PI / 180)) - (enc_left * cos(45 * M_PI / 180));
    }

    float sigmoid(float interval)
    {
        return 1.0 / (1.0 + exp(-2.5 * (2 * interval - 1)));
    }

    float normalizeAngle(float angle) {
        
        angle = fmod(angle, 360.0);
        // if (angle < 0) {
        //     angle += 360.0;
        // }
        return angle;
    }

    float angleBetweenVectors(float ax, float ay, float bx, float by) {
        
        float x = bx - ax;
        float y = by - ay;

        if (x >= 0 && y >= 0) {
            return 225.0f;
        } else if (x < 0 && y >= 0) {
            return 135.0f;
        } else if (x >= 0 && y < 0) {
            return 315.0f;
        } else if (x < 0 && y < 0) {
            return 45.0f;
        }
    }

    char getKey()
    {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        new_tio.c_cc[VMIN] = 1; // Minimum number of characters to read
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

    void bs2pcCallback(const FP_Magang::BS2PC &msg)
    {

        if (msg.status == 3)
        {
            if (target[0] != msg.tujuan_x || target[1] != msg.tujuan_y) {
                blankFrame = 0;
                currStep = 1;
                ROS_INFO("Y %.2f %.2f", target[0], msg.tujuan_x);

                target[0] = msg.tujuan_x;
                target[1] = msg.tujuan_y;
            }

            target[0] = msg.tujuan_x;
            target[1] = msg.tujuan_y;
        }

        if (prevMsg.status != msg.status && (msg.status == 2 || msg.status == 4))
        {
            blankFrame = 0;
            currStep = 0;
        }

        prevMsg = msg;
        prevMsg.th = normalizeAngle(prevMsg.th);

        updateRobotPos(msg.enc_left, msg.enc_right);
    }

    void pc2pcCallback(const FP_Magang::PC2PC &msg)
    {
        if (prevMsg.status == 4 || prevMsg.status == 2) {
            ballPos[0] = msg.follow_x;
            ballPos[1] = msg.follow_y;
        }
        // ballPos[0] = msg.follow_x;
        // ballPos[1] = msg.follow_y;

        if (prevMsg.status == 2)
        {
            target[0] = ballPos[0];
            target[1] = ballPos[1];

            currStep = 1;
        }
        else if (prevMsg.status == 4)
        {
            float x = ballPos[0] - robotPos[0];
            float y = ballPos[1] - robotPos[1];

            target[0] = ballPos[0] - 100 * cos(atan2(y, x));
            target[1] = ballPos[1] - 100 * sin(atan2(y, x));

            currStep = 1;
        }
    }

public:
    FP_Magang::BS2PC prevMsg;
    FP_Magang::PC2BS msg;

    float robotPos[2] = {0, 0};
    float ballPos[2] = {0, 0};
    float target[2] = {0, 0};

    bool grabbingBall = false;

    int currStep = 0;
    int blankFrame = 0;

    Controller()
    {
        pubPC2BS = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 50);

        subBS2PC = nh.subscribe("/bs2pc", 1, &Controller::bs2pcCallback, this);
        subPC2PC = nh.subscribe("/pc2pc", 1, &Controller::pc2pcCallback, this);
    }

    void shootBall(float power)
    {
        float ct = prevMsg.th;
        float x = ballPos[0] + (power * sin(degToRad(ct)));
        float y = ballPos[1] + (power * cos(degToRad(ct)));

        // if (x + 27.7430 < 0)
        // {
        //     x = -27.7430;
        // }
        // else if (x - 926.866219 > 0)
        // {
        //     x = 926.866219;
        // }

        // if (y + 31.4311 < 0)
        // {
        //     y = -31.4311;
        // }
        // else if (y - 627.356796 > 0)
        // {
        //     y = 627.356796;
        // }

        ballPos[0] = x;
        ballPos[1] = y;

        FP_Magang::PC2BS msg;
        msg.bola_x = x;
        msg.bola_y = y;
        pubPC2BS.publish(msg);
        spinOnce();
    }

    void updateMotor(float x, float y, float theta, bool manual = false)
    {

        float motorSpeed = 1000.0f;

        if (manual)
        {
            float ct = -prevMsg.th;
            float sinus = sin(degToRad(ct));
            float cosinus = cos(degToRad(ct));

            float tempX = x;
            float tempY = y;

            x = (tempX * cosinus) + (tempY * -sinus);
            y = (tempX * sinus) + (tempY * cosinus);
        }

        if (!manual && interval < 1) {
            interval += 0.02;
        } else {
            interval = 1;
        }

        int areaY = 1016, areaX = 716;
        float batasX = areaY - 58 * 2;
        float batasY = areaX - 58 * 2;
        if (((robotPos[0] + (x / 50)) < 0) || ((robotPos[0] + (x / 50)) > batasX))
        {
            x = 0;
        }
        if (((robotPos[1] + (y / 50)) < 0) || ((robotPos[1] + (y / 50)) > batasY))
        {
            y = 0;
        }

        (round(x) == 0) ? x = 0 : x = x;
        (round(y) == 0) ? y = 0 : y = y;
        (round(theta) == 0) ? theta = 0 : theta = theta;

        if ((round(x) == 0) && (round(y) == 0) && (round(theta / 10) == 0))
        {
            interval = 0;
            // if (prevMsg.status == 4)
            // {
            //     currStep++;
            // }
            return;
        }

        FP_Magang::PC2BS msg;
        msg.motor1 = ((x * 2 / 3) + (y * 0) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor2 = ((x * -1 / 3) + (y * (sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor3 = ((x * -1 / 3) + (y * -(sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);

        msg.bola_x = ballPos[0];
        msg.bola_y = ballPos[1];
        pubPC2BS.publish(msg);
        spinOnce();
    }

    void function_1()
    {
        ROS_INFO("Manual Steering");

        char key = getKey();
        float distToBall = 0;

        float MOVE_SPEED = 300;

        switch (key) {
        case 'w':
            updateMotor(0, 1*MOVE_SPEED, 0, true); break;
        case 's':
            updateMotor(0, -1*MOVE_SPEED, 0, true); break;
        case 'a':
            updateMotor(1*MOVE_SPEED, 0, 0, true); break;
        case 'd':
            updateMotor(-1*MOVE_SPEED, 0, 0, true); break;
        case 'q':
            updateMotor(0, 0, 10.0f*MOVE_SPEED, true); break;
        case 'e':
            updateMotor(0, 0, -10.0f*MOVE_SPEED, true); break;
        case 'z':
            distToBall = sqrt(pow((ballPos[0] - robotPos[0]), 2) + pow((ballPos[1] - robotPos[1]), 2));
            grabbingBall = distToBall <= 20 ? true : grabbingBall; 
            break;
        case 'x':
            grabbingBall = false;
            break;
        case 'p':
            if (grabbingBall) {
                grabbingBall = false;
                shootBall(300);
            }
            break;
        case 'o':
            if (grabbingBall) {
                grabbingBall = false;
                shootBall(100);
            }
            break;
        case 'l':
            ROS_INFO("Leaving Manual Steering Mode");
            
            shutdown();
            break;
        }

        if (grabbingBall)
        {
            ballPos[0] = robotPos[0];
            ballPos[1] = robotPos[1];
            msg.bola_x = ballPos[0];
            msg.bola_y = ballPos[1];
            pubPC2BS.publish(msg);
        }
    }

    void function_2()
    {

        if (blankFrame < 10) {
            blankFrame++;
            return;
        }

        // float x = target[0] - robotPos[0];
        // float y = target[1] - robotPos[1];

        // if (x == 0 && y == 0)
        //     return;

        // float th = 90 - degToRad(atan2(y, x));

        // updateMotor(x, y, (th - prevMsg.th));

        float x = target[0] - robotPos[0];
        float y = target[1] - robotPos[1];

        if (fabs(x) <= 0.1 && fabs(y) <= 0.1) {
            return;
        }
        if (currStep == 1) {
            float th = 50 - degToRad(atan2(y, x));
            float rotation = th - prevMsg.th;

            if (fabs(rotation) < 5.0) {
                currStep = 2;
            } else {
                updateMotor(0, 0, rotation*5.0f);
            }

            ROS_INFO("ROTATING %.2f %.2f", th, rotation);
        }
        else if (currStep == 2) {
            
            // float moveSpeed = sqrt(x * x + y * y);

            // float speedFactor = 1.0;
            // if (moveSpeed < 5.0) {
            //     speedFactor = moveSpeed / 5.0;
            //     currStep = 1;
            // }

            // updateMotor(0, moveSpeed * speedFactor, 0, true);
            // ROS_INFO("MOVING %.2f", moveSpeed * speedFactor);
            updateMotor(x, y, 0);
        }
    }

    void function_3()
    {
        // float x = target[0] - robotPos[0];
        // float y = target[1] - robotPos[1];

        // if (x == 0 && y == 0)
        //     return;

        // float th = 90 - degToRad(atan2(y, x));

        // updateMotor(x, y, (th - prevMsg.th));

        if (blankFrame < 5) {
            blankFrame++;
            return;
        }

        float x = target[0] - robotPos[0];
        float y = target[1] - robotPos[1];

        if (fabs(x) <= 0.1 && fabs(y) <= 0.1) {
            return;
        }
        // float th = 50 - degToRad(atan2(y, x));
        float th = angleBetweenVectors(target[0], target[1], robotPos[0], robotPos[1]);
        float rotation = th - prevMsg.th;
        ROS_INFO("TO %.2f %.2f", th, rotation);

        if (currStep == 1) {
            if (fabs(rotation) < 10.0) {
                currStep = 2;
            } else {
                updateMotor(0, 0, 200.0f);
            }
        } else if (currStep == 2) {
            updateMotor(x, y, 0);
        }
    }

    void function_4(float &offset) {
    
        float x;
        float y;
        float th;

        if (blankFrame < 10) {
            blankFrame++;
            return;
        }

        x = target[0] - robotPos[0];
        y = target[1] - robotPos[1];

        if (fabs(x) <= 0.1 && fabs(y) <= 0.1) {
            return;
        }
        if (currStep == 1) {
            float th = 50 - degToRad(atan2(y, x));
            float rotation = th - prevMsg.th;

            if (fabs(rotation) < 5.0) {
                currStep = 2;
            } else {
                updateMotor(0, 0, rotation*5.0f);
            }

            ROS_INFO("ROTATING %.2f %.2f", th, rotation);
        }
        else if (currStep == 2) {
            // UNFINISHED :))
            updateMotor(x, y, 0);
        }
    }

    void run()
    {
        float offset = 0;

        Rate rate(50);
        while (ok())
        {
            spinOnce();

            if (prevMsg.status == 1)
            {
                function_1();
                updateMotor(0, 0, 0, true);

            } else if (prevMsg.status == 2)
            {
                msg.bola_x = ballPos[0];
                msg.bola_y = ballPos[1];
                pubPC2BS.publish(msg);
                function_2();

            } else if (prevMsg.status == 3)
            {
                function_3();

            } else if (prevMsg.status == 4)
            {
                function_4(offset);
            }

            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    init(argc, argv, "robot_controller");

    Controller controller;
    controller.run();

    return 0;
}
