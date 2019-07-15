#include <iostream>

enum{
    PLEASE_WAIT = 1,
    READY = 2,
    ANSWER = 3,
    APPLY_FORCE_TO_PERSON = 4,
    APPLY_FORCE_TO_DEVICE = 5,
};

struct Signals
{
    std::vector<std::string> name = {"", "Please Wait", "Ready", "Answer", "Apply Force To Person", "Apply Force To Device"};
};
