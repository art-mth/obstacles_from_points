#ifndef OBSTACLES_FROM_POINTS_H
#define OBSTACLES_FROM_POINTS_H

#include <lms/module.h>

class ObstaclesFromPoints : public lms::Module {
   public:
    bool initialize();
    bool deinitialize();
    void configsChanged() override;
    bool cycle();
};

#endif  // OBSTACLES_FROM_POINTS_H
