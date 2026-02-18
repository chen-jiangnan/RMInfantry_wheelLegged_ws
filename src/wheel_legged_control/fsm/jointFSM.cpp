#include "jointFSM.hpp"

JFSMStateBase::JFSMStateBase(FSMEvent* e): event(e) {}

JFSMStateZeroTau::JFSMStateZeroTau(FSMEvent* event):JFSMStateBase(event){
    stateName = JFSMode::ZEROTAU;
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
JFSMode JFSMStateZeroTau::checkTransition(JFSMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Keeping Enter ZEROTAU"
        << C_RESET << std::endl;
        return JFSMode::ZEROTAU;
    }
    if(next_state == JFSMode::READY){
        if(event->cali){
            printTransition("ZEROTAU","READY");
            return JFSMode::READY;
        }
        else{
            std::cout
                << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
                << C_RESET << std::endl;   
            printTransition("ZEROTAU","CALI");
            return JFSMode::CALI;
        }
    }
    else if(next_state == JFSMode::RESET){
        if(event->cali){
            printTransition("ZEROTAU","RESET");
            return JFSMode::RESET;
        }
        else{
            std::cout
            << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
            << C_RESET << std::endl;
            // 不直接跳转到Cali
            return this->stateName;
        }
    }
    else if(next_state == JFSMode::DAMPING){
        printTransition("ZEROTAU","DAMPING");
        return JFSMode::DAMPING;
    }
    else{
        return this->stateName;
    }
}

JFSMStateCali::JFSMStateCali(FSMEvent* event):JFSMStateBase(event){
    stateName = JFSMode::CALI;
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
JFSMode JFSMStateCali::checkTransition(JFSMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl;
        printTransition("CALI","ZEROTAU");
        return JFSMode::ZEROTAU;
    }    
    if(event->cali){
        printTransition("CALI","READY");
        return JFSMode::READY;
    }
    else{
        std::cout
        << C_YELLOW << "[FSM][WARNING]: Waiting Cail Finish..."
        << C_RESET << std::endl;         
        return this->stateName;
    }
}

JFSMStateReady::JFSMStateReady(FSMEvent* event):JFSMStateBase(event){
    stateName = JFSMode::READY;
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
JFSMode JFSMStateReady::checkTransition(JFSMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl; 
        printTransition("READY","ZEROTAU");
        return JFSMode::ZEROTAU;
    }

    if(next_state == JFSMode::ZEROTAU){
        printTransition("READY","ZEROTAU");
        return JFSMode::ZEROTAU;
    }
    else if(next_state == JFSMode::DAMPING){
        printTransition("READY","DAMPING");
        return JFSMode::DAMPING;
    }
    else{
        return this->stateName;
    }
}

JFSMStateDamping::JFSMStateDamping(FSMEvent* event):JFSMStateBase(event){
    stateName = JFSMode::DAMPING;
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
JFSMode JFSMStateDamping::checkTransition(JFSMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl;
        printTransition("DAMPING","ZEROTAU"); 
        return JFSMode::ZEROTAU;
    }

    if(next_state == JFSMode::ZEROTAU){
        printTransition("DAMPING","ZEROTAU");
        return JFSMode::ZEROTAU;
    }
    else if(next_state == JFSMode::RESET){
        if(event->cali){
            printTransition("DAMPING","RESET");
            return JFSMode::RESET;
        }
        else{
            std::cout
                << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
                << C_RESET << std::endl;   
            printTransition("DAMPING","CALI");
            return JFSMode::CALI;
        }        
    }
    else{
        return this->stateName;
    }
}

JFSMStateReset::JFSMStateReset(FSMEvent* event):JFSMStateBase(event){
    stateName = JFSMode::RESET;
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
JFSMode JFSMStateReset::checkTransition(JFSMode next_state){
    if(event->error){
        std::cout
        << C_RED << "[FSM][ERROR]: Something Wrong,Try to Enter ZEROTAU"
        << C_RESET << std::endl; 
        printTransition("RESET","ZEROTAU");
        return JFSMode::ZEROTAU;
    }

    if(next_state == JFSMode::DAMPING){
        printTransition("RESET","DAMPING");
        return JFSMode::DAMPING;
    }
    else if(next_state == JFSMode::READY){
        if(event->cali){
            printTransition("RESET","READY");
            return JFSMode::READY;
        }
        else{
            std::cout
                << C_YELLOW << "[FSM][WARNING]: Please Cail First!!!"
                << C_RESET << std::endl;   
            printTransition("RESET","CALI");
            return JFSMode::CALI;
        }
    }
    else if(next_state == JFSMode::ZEROTAU){   
        printTransition("RESET","ZEROTAU");
        return JFSMode::CALI;
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
    nextState = JFSMode::ZEROTAU;
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

JFSMStateBase* JFSM::getNextState(JFSMode stateName) {
    switch(stateName) {
        case JFSMode::ZEROTAU:
            return stateList.zeroTau;
        break;
        case JFSMode::CALI:
            return stateList.cali;
        break;
        case JFSMode::READY:
            return stateList.ready;
        break;
        case JFSMode::DAMPING:
            return stateList.damping;
        break;
        case JFSMode::RESET:
            return stateList.reset;
        break;
        default:
            return stateList.damping;
        break;
    }
}