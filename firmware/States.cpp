#include "States.h"
States next(States s) {
    switch (s) {
        case States::PAD:            return States::POWERED_FLIGHT;
        case States::POWERED_FLIGHT: return States::COAST;
        case States::COAST:          return States::DEPLOY;
        case States::DEPLOY:         return States::DEPLOY;
    }
    return States::DEPLOY; // fallback safety
}
