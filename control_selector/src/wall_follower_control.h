#pragma once

#include <vector>
#include "control.h"

class WallFollowerControl : public Control
{
private:
    bool obstacle = false;
    bool turnRight = false;
    bool turnLeft = true;
    bool straight = false;
    const double kMinRange = 0.5;
    const double PI = 3.14159265;

public:
    // ~WallFollowerControl();

    // установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    // установка текущей позиции робота
    void setRobotPose(double x, double y, double theta) override;

    // получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "Wall Follower"; }
};
