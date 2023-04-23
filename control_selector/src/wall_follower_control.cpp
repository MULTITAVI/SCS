#include "wall_follower_control.h"

void WallFollowerControl::setLaserData(const std::vector<float>& data) {
    //проверим нет ли вблизи робота препятствия
    for (size_t i = 0; i<data.size(); ++i) {
        if (data[i] < kMinRange) {
		    obstacle = true;
		    break;
	    }
        else {
            obstacle = false;
        }
    }
}

void WallFollowerControl::setRobotPose(double x, double y, double theta) {
    theta = theta * 180 / PI;

    if ((x > -7 && x < -6) && 
        ((theta < 100 && theta > 80) || 
        (theta > -100 && theta < -80))) {
        turnLeft = false;
        turnRight = false;
        straight = true;
    }
    else if ((x > -7 && x < -6) && 
        ((theta < 180 && theta > 100) || 
        (theta > -180 && theta < -100))) {
        turnLeft = false;
        turnRight = true;
        straight = false;
    }
    else if ((x > -7 && x < -6) && 
        ((theta < 100 && theta > 0) || 
        (theta > -100 && theta < 0))) {
        turnLeft = true;
        turnRight = false;
        straight = false;
    }
    else if (((theta> -80 && theta < -10) || (theta < 170 && theta > 100)) && !(x > -7 && x < -6)) {
        turnLeft = true;
        turnRight = false;
        straight = false;
    }
    else if (((theta > -170 && theta < -100) ||
            (theta > -80 && theta < -10)) && !(x > -7 && x < -6) ) {
        turnRight = true;
        turnLeft = false;
        straight = false;
    }
    else if ((theta > -10 && theta < 10) && x > -6) {
        turnLeft = true;
        turnRight = false;
        straight = false;
    }
    else if ((theta > -10 && theta < 10) && x < -7) {
        turnLeft = false;
        turnRight = false;
        straight = true;
    }
    else if ((theta < -170 || theta > 170) && x > -6) {
        turnLeft = false;
        turnRight = false;
        straight = true;
    }
    else if ((theta > -170 || theta < 170) && x < -7) {
        turnLeft = false;
        turnRight = true;
        straight = false;
    }
}

void WallFollowerControl::getControl(double& v, double& w) {
    if (obstacle) {
        v = 0;
        w = 0.5;
    }
    else if (!obstacle && straight && !turnLeft && !turnRight) {
        v = 0.5;
        w = 0;
    }
    else if (!obstacle && turnLeft && !straight && !turnRight) {
        v = 0;
        w = 0.5;
    }
    else if (!obstacle && turnRight && !straight && !turnLeft) {
        v = 0;
        w = -0.5;
    }
    else {
        v = 0.5;
        w = 0;
    }
}
