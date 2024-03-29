#include <iostream>

enum{
    NO_EMOTION = 0,
    ANGER      = 1,
    FEAR       = 2,
    DISGUST    = 3,
    SAD        = 4,
    HAPPY      = 5,
    UNKNOWN    = 6,
    //GRATITUDE  = 6,
    //SYMPATHY   = 7,
    //LOVE       = 8,
};

struct EmotionInfo
{
    std::string name;
    int id;
    std::string estimated_emotion1;
    std::string estimated_emotion2;
    std::string estimated_emotion3;
};

struct Emotions
{
    std::vector<std::string> name = {"",
                                     "anger", 
                                     "fear", 
                                     "disgust", 
                                     "sad", 
                                     "happy", 
                                     "unknown", 
                                     //"gratitude", 
                                     //"sympathy", 
                                     //"love"
                                     };
    std::vector<std::string> nameJP = {"",
                                     "怒る", 
                                     "怖がる", 
                                     "嫌悪する", 
                                     "悲しむ", 
                                     "楽しい",
                                     };
    std::vector<EmotionInfo> info;
};

