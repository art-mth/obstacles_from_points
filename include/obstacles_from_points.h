#ifndef OBSTACLES_FROM_POINTS_H
#define OBSTACLES_FROM_POINTS_H

#include <lms/module.h>

class ObstaclesFromPoints : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();

   private:
    lms::ReadDataChannel<lms::math::polyLine2f> points;
    lms::ReadDataChannel<lms::math::polyLine2f> centerLine;

    lms::WriteDataChannel<street_environment::EnvironmentObjects> obstacles;
};

#endif  // OBSTACLES_FROM_POINTS_H
