#line 1 "/home/ahsan/code/ArduinoTVC/States.h"
enum States {
    PAD,
    POWERED_FLIGHT,
    COAST,
    DEPLOY,
};

States next(States s); 
