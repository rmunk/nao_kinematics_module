#include "kinematics_module.h"
#include <iostream>
#include <alcommon/albroker.h>
#include <qi/log.hpp>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>

using namespace AL;
using namespace KMath::KMat;
using namespace KDeviceLists;

kinematics_module::kinematics_module(boost::shared_ptr<AL::ALBroker> broker, const std::string& name) : ALModule(broker, name)
{
    /** Describe the module here. This will appear on the webpage*/
    setModuleDescription("Kinematics module.");

    /** Define callable methods with their descriptions:
    * This makes the method available to other cpp modules
    * and to python.
    * The name given will be the one visible from outside the module.
    * This method has no parameters or return value to describe
    */
    functionName("say_hello", getName(), "Say hello to the world");
    BIND_METHOD(kinematics_module::say_hello);

    functionName("say_text", getName(), "Say a given sentence.");
    /**
    * This enables to document the parameters of the method.
    * It is not compulsory to write this line.
    */
    addParam("to_say", "The sentence to be said.");
    BIND_METHOD(kinematics_module::say_text);

    functionName("say_text_and_return_length", getName(),
                 "Say a given sentence, and return its length");
    addParam("to_say", "The sentence to be said.");
    /**
    * This enables to document the return of the method.
    * It is not compulsory to write this line.
    */
    setReturn("sentence_length", "Length of the said sentence.");
    BIND_METHOD(kinematics_module::say_text_and_return_length);

    functionName("get_com", getName(),
                 "Return current position of COM");
    setReturn("com", "Position of COM.");
    BIND_METHOD(kinematics_module::get_com);
}

kinematics_module::~kinematics_module() {}

void kinematics_module::init()
{
    say_hello();
}

void kinematics_module::say_text(const std::string &to_say)
{
    std::cout << "Saying the phrase in the console..." << std::endl;
    std::cout << to_say << std::endl;
    try
    {
      /** Create a proxy to TTS.*/
      ALTextToSpeechProxy tts(getParentBroker());
      /** Call the say method. */
      tts.say(to_say);
      /** Note: on the desktop you won't hear anything, but you should see
      * some logs on the naoqi you are connected to. */
    }
    catch(const AL::ALError&)
    {
      qiLogError("module.example") << "Could not get proxy to ALTextToSpeech" << std::endl;
    }
}

void kinematics_module::say_hello()
{
    say_text("Hello");
}

int kinematics_module::say_text_and_return_length(const std::string &to_say)
{
    say_text(to_say);
    return to_say.length();
}

std::vector<float> kinematics_module::get_joints()
{

    std::vector<float> joints(KDeviceLists::NUMOFJOINTS);
    double pi = KMath::KMat::transformations::PI;
    //Left Hand
    joints[L_ARM+SHOULDER_PITCH]=0.2;
    joints[L_ARM+SHOULDER_ROLL]=0.1;
    joints[L_ARM+ELBOW_YAW]=0;
    joints[L_ARM+ELBOW_ROLL]=0;
    joints[L_ARM+WRIST_YAW]=0.0;
    //Right Hand
    joints[R_ARM+SHOULDER_PITCH]=M_PI_2;
    joints[R_ARM+SHOULDER_ROLL]=-M_PI_4;
    joints[R_ARM+ELBOW_YAW]=0;
    joints[R_ARM+ELBOW_ROLL]=0;
    joints[R_ARM+WRIST_YAW]=M_PI/3535;
    //Left Leg
    joints[L_LEG+HIP_YAW_PITCH]=0;
    joints[L_LEG+HIP_ROLL]=0;
    joints[L_LEG+HIP_PITCH]=0;
    joints[L_LEG+KNEE_PITCH]=M_PI/4;
    joints[L_LEG+ANKLE_PITCH]=-M_PI/4;
    joints[L_LEG+ANKLE_ROLL]=0;
    //Right Leg
    joints[R_LEG+HIP_YAW_PITCH]=0;
    joints[R_LEG+HIP_ROLL]=0;
    joints[R_LEG+HIP_PITCH]=0;
    joints[R_LEG+KNEE_PITCH]=0;
    joints[R_LEG+ANKLE_PITCH]=0;
    joints[R_LEG+ANKLE_ROLL]=0;
    //Head
    joints[HEAD+YAW]=M_PI_2;
    joints[HEAD+PITCH]=0;

    //Now we can set the joints to the class
    return joints;
}

AL::ALValue kinematics_module::get_com()
{
    ALMotionProxy motion(getParentBroker());
    std::vector<float> sensorAngles = motion.getAngles("Joints", false);
    std::cout << "Sensor angles: " << std::endl << sensorAngles << std::endl;
    std::cout << "KDeviceLists::NUMOFJOINTS: " << KDeviceLists::NUMOFJOINTS << std::endl;

    NAOKinematics nkin;
    std::cout << "Set joints successful: " << nkin.setJoints(sensorAngles) << std::endl;
    KVecDouble3 com = nkin.calculateCenterOfMass();
    return AL::ALValue::array(com.get(0, 0), com.get(1, 0), com.get(2, 0));
}
