#include "jointFSM.hpp"

JFSMStateBase::JFSMStateBase(FSMEvent* e): event(e) {}

JFSMStateZeroTau::JFSMStateZeroTau(FSMEvent* event):JFSMStateBase(event){
    stateName = JointMode::ZEROTAU;
}
void JFSMStateZeroTau::enter(){
    std::cout
        << C_GREEN << "[FSM][ENTER] ZEROTAU"
        << C_RESET << std::endl;    
}
void JFSMStateZeroTau::exit(){
    std::cout
        << C_GREEN << "[FSM][EXIT] ZEROTAU"
        << C_RESET << std::endl;    
}
JointMode JFSMStateZeroTau::checkTransition(JointMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Keeping Enter ZEROTAU"
        << C_RESET << std::endl;
        return JointMode::ZEROTAU;
    }
    if(next_state == JointMode::READY){
        if(event->cali){
            printTransition("ZEROTAU","READY");
            return JointMode::READY;
        }
        else{
            std::cout
                << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
                << C_RESET << std::endl;   
            printTransition("ZEROTAU","CALI");
            return JointMode::CALI;
        }
    }
    else if(next_state == JointMode::RESET){
        if(event->cali){
            printTransition("ZEROTAU","RESET");
            return JointMode::RESET;
        }
        else{
            std::cout
            << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
            << C_RESET << std::endl;
            // 不直接跳转到Cali
            return this->stateName;
        }
    }
    else if(next_state == JointMode::DAMPING){
        printTransition("ZEROTAU","DAMPING");
        return JointMode::DAMPING;
    }
    else{
        return this->stateName;
    }
}

JFSMStateCali::JFSMStateCali(FSMEvent* event):JFSMStateBase(event){
    stateName = JointMode::CALI;
}
void JFSMStateCali::enter(){
    std::cout
        << C_GREEN << "[FSM][ENTER] CALI"
        << C_RESET << std::endl;    
}
void JFSMStateCali::exit(){
    std::cout
        << C_GREEN << "[FSM][EXIT] CALI"
        << C_RESET << std::endl;    
}
JointMode JFSMStateCali::checkTransition(JointMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl;
        printTransition("CALI","ZEROTAU");
        return JointMode::ZEROTAU;
    }    
    if(event->cali){
        printTransition("CALI","READY");
        return JointMode::READY;
    }
    else{
        std::cout
        << C_YELLOW << "[FSM][WARNING]: Waiting Cail Finish..."
        << C_RESET << std::endl;         
        return this->stateName;
    }
}

JFSMStateReady::JFSMStateReady(FSMEvent* event):JFSMStateBase(event){
    stateName = JointMode::READY;
}
void JFSMStateReady::enter(){
    std::cout
        << C_GREEN << "[FSM][ENTER] READY"
        << C_RESET << std::endl;    
}
void JFSMStateReady::exit(){
    std::cout
        << C_GREEN << "[FSM][EXIT] READY"
        << C_RESET << std::endl;    
}
JointMode JFSMStateReady::checkTransition(JointMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl; 
        printTransition("READY","ZEROTAU");
        return JointMode::ZEROTAU;
    }

    if(next_state == JointMode::ZEROTAU){
        printTransition("READY","ZEROTAU");
        return JointMode::ZEROTAU;
    }
    else if(next_state == JointMode::DAMPING){
        printTransition("READY","DAMPING");
        return JointMode::DAMPING;
    }
    else{
        return this->stateName;
    }
}

JFSMStateDamping::JFSMStateDamping(FSMEvent* event):JFSMStateBase(event){
    stateName = JointMode::DAMPING;
}
void JFSMStateDamping::enter(){
    std::cout
        << C_GREEN << "[FSM][ENTER] DAMPING"
        << C_RESET << std::endl;    
}
void JFSMStateDamping::exit(){
    std::cout
        << C_GREEN << "[FSM][EXIT] DAMPING"
        << C_RESET << std::endl;    
}
JointMode JFSMStateDamping::checkTransition(JointMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl;
        printTransition("DAMPING","ZEROTAU"); 
        return JointMode::ZEROTAU;
    }

    if(next_state == JointMode::ZEROTAU){
        printTransition("DAMPING","ZEROTAU");
        return JointMode::ZEROTAU;
    }
    else if(next_state == JointMode::RESET){
        if(event->cali){
            printTransition("DAMPING","RESET");
            return JointMode::RESET;
        }
        else{
            std::cout
                << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
                << C_RESET << std::endl;   
            printTransition("DAMPING","CALI");
            return JointMode::CALI;
        }        
    }
    else{
        return this->stateName;
    }
}

JFSMStateReset::JFSMStateReset(FSMEvent* event):JFSMStateBase(event){
    stateName = JointMode::RESET;
}
void JFSMStateReset::enter(){
    std::cout
        << C_GREEN << "[FSM][ENTER] RESET"
        << C_RESET << std::endl;    
}
void JFSMStateReset::exit(){
    std::cout
        << C_GREEN << "[FSM][EXIT] RESET"
        << C_RESET << std::endl;    
}
JointMode JFSMStateReset::checkTransition(JointMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl; 
        printTransition("RESET","ZEROTAU");
        return JointMode::ZEROTAU;
    }

    if(next_state == JointMode::DAMPING){
        printTransition("RESET","DAMPING");
        return JointMode::DAMPING;
    }
    else if(next_state == JointMode::READY){
        if(event->cali){
            printTransition("RESET","READY");
            return JointMode::READY;
        }
        else{
            std::cout
                << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
                << C_RESET << std::endl;   
            printTransition("RESET","CALI");
            return JointMode::CALI;
        }
    }
    else if(next_state == JointMode::ZEROTAU){   
        printTransition("RESET","ZEROTAU");
        return JointMode::CALI;
    }
    else{
        return this->stateName;
    }
}




JFSM::JFSM(FSMEvent* event) {
    stateList.zeroTau = new JFSMStateZeroTau(event);
    stateList.cali = new JFSMStateCali(event);
    stateList.ready = new JFSMStateReady(event);
    stateList.damping = new JFSMStateDamping(event);
    stateList.reset = new JFSMStateReset(event);
    nextState = JointMode::ZEROTAU;
    currentStatePtr = stateList.zeroTau;
    currentStatePtr-> enter();
}
JFSM::~JFSM(){
    stateList.deletePtr();
}

void JFSM::JFSMUpdate(){  
    // printf("JFSMRunerrMode=%d\n",errMode);  
    // if(errMode == ErrorMode::DISCONN) {
    //     printf("JFSMRun shutdown!!!!!!!!!!!\n");
    //     return;
    // }
    auto transtionState = currentStatePtr->checkTransition(nextState);
    if(transtionState != currentStatePtr->stateName){
        currentStatePtr->exit();
        currentStatePtr = getNextState(transtionState);
        currentStatePtr->enter();
    }
}

JFSMStateBase* JFSM::getNextState(JointMode stateName) {
    switch(stateName) {
        case JointMode::ZEROTAU:
            return stateList.zeroTau;
        break;
        case JointMode::CALI:
            return stateList.cali;
        break;
        case JointMode::READY:
            return stateList.ready;
        break;
        case JointMode::DAMPING:
            return stateList.damping;
        break;
        case JointMode::RESET:
            return stateList.reset;
        break;
        default:
            return stateList.damping;
        break;
    }
}