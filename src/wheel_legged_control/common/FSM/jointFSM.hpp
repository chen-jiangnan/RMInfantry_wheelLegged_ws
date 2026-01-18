#ifndef JointFSM_H
#define JointFSM_H

#include <csignal>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <unistd.h>

#define C_RESET   "\033[0m"
#define C_RED     "\033[31m"
#define C_GREEN   "\033[32m"
#define C_YELLOW  "\033[33m"
#define C_BLUE    "\033[34m"
#define C_PURPLE  "\033[35m"
#define C_CYAN    "\033[36m"

enum class JointMode {
    ZEROTAU,
    CALI,
    READY,
    DAMPING,
    RESET,
};

struct FSMEvent {
    bool cali = false;
    bool error = false;
};

class JFSMStateBase{
public:
    JFSMStateBase(FSMEvent* e);
    virtual ~JFSMStateBase() = default;
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual JointMode checkTransition(JointMode next_state) {return JointMode::ZEROTAU;}
    JointMode stateName;
protected:
    void printTransition(const std::string& from,
        const std::string& to)
    {
        std::cout
        << C_BLUE << "[FSM]" << C_RESET << " "
        << C_GREEN << from << C_RESET
        << C_YELLOW << " ====> " << C_RESET
        << C_CYAN << to << C_RESET
        << std::endl;
    }
    FSMEvent* event;
};


class JFSMStateZeroTau: public JFSMStateBase {
public:
    JFSMStateZeroTau(FSMEvent* event);
    void enter();
    void exit();
    JointMode checkTransition(JointMode next_state);  
};

class JFSMStateCali: public JFSMStateBase 
{
public:
    JFSMStateCali(FSMEvent* event);
    void enter();
    void exit();
    JointMode checkTransition(JointMode next_state);  
private:
    std::chrono::time_point<std::chrono::system_clock> startTime;
    float caliTime = 5.0;
    bool finishCali = false;
};

class JFSMStateReady: public JFSMStateBase {
public:
    JFSMStateReady(FSMEvent* event);
    void enter();
    void exit();
    JointMode checkTransition(JointMode next_state);  
private:
    bool firstCali = false;
};

class JFSMStateDamping: public JFSMStateBase {
public:
    JFSMStateDamping(FSMEvent* event);
    void enter();
    void exit();
    JointMode checkTransition(JointMode next_state);  
};

class JFSMStateReset: public JFSMStateBase {
public:
    JFSMStateReset(FSMEvent* event);
    void enter();
    void exit();
    JointMode checkTransition(JointMode next_state);  
};

struct JFSMStateList {
    JFSMStateZeroTau *zeroTau;
    JFSMStateCali *cali;
    JFSMStateReady *ready;
    JFSMStateDamping *damping;
    JFSMStateReset *reset;

    void deletePtr(){
        delete zeroTau;
        delete cali;
        delete ready;
        delete damping;
        delete reset;
    }  
};

class JFSM{
public:
    JFSM(FSMEvent* event);
    ~JFSM();
    void JFSMUpdate();
    JointMode setZeroTauState(){return nextState = JointMode::ZEROTAU;}
    JointMode setCaliState(){return nextState = JointMode::CALI;}
    JointMode setReadyState(){return nextState = JointMode::READY;}
    JointMode setDampingState(){return nextState = JointMode::DAMPING;}
    JointMode setResetState(){return nextState = JointMode::RESET;}
    JointMode getCurrentState(){return currentStatePtr->stateName;}
    bool ifReady(){return getCurrentState() == JointMode::READY;}
private:
    JFSMStateBase* getNextState(JointMode stateName);
    JFSMStateBase* currentStatePtr;
    JFSMStateList stateList;
    JointMode nextState;
};
#endif //JointFSM