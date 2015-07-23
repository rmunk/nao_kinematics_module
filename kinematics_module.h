#ifndef KINEMATICS_MODULE_H
#define KINEMATICS_MODULE_H

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include "3rd_party/NAOKinematics/KMat.hpp"
#include "3rd_party/NAOKinematics/NAOKinematics.h"

namespace AL
{
  // This is a forward declaration of AL:ALBroker which
  // avoids including <alcommon/albroker.h> in this header
  class ALBroker;
}

class kinematics_module : public AL::ALModule
{
private:
    std::vector<float> get_joints();

public:
    kinematics_module(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);

    virtual ~kinematics_module();

    virtual void init();

    void say_hello();
    void say_text(const std::string& to_say);
    int say_text_and_return_length(const std::string& to_say);

    /**
     * @fn KVecFloat3 get_com()
     * @brief Get the center of mass of the robot
     * */
    AL::ALValue get_com();

};

#endif // KINEMATICS_MODULE_H
