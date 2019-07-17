#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "emotions.h"
#include "experiment_signals.h"
#include "master_thesis_program/Answers.h"
#endif

#include <QWidget>
#include <QStackedWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QSpinBox>
#include <iostream>
#include <fstream>

class Control : public QWidget
{
    Q_OBJECT
    public:
        Control(QWidget *parent=0);
        std::ofstream ex_result;
        std::ofstream ex_info;
        std::string path = ros::package::getPath("master_thesis_program") + "/data/";

    protected:
        ros::NodeHandle nh;
        ros::Publisher pub_subjectA;
        ros::Publisher pub_subjectB;
        ros::Publisher pub_record;
        ros::Subscriber sub_subjectA;
        ros::Subscriber sub_subjectB;
        Emotions emotion_data;

    private Q_SLOTS:
        void nextSignal();
        void applyToPersonSignal();
        void applyToDeviceSignal();
        void retryDeviceSignal();
        void newSubjectsSignal();

    private:
        bool subjectA_finished;
        bool subjectB_finished;

        int current_emotion;
        int current_emotion_index;
        std::vector<int> emotion_list = {1,2,3,4,5,6,7,8};
       
        QSpinBox *ex_no;
        QLabel *info_label;
        QLabel *current_emotion_label;
        QLineEdit *subjectA_name_edit;
        QLineEdit *subjectB_name_edit;
        QPushButton *newButton;
        QPushButton *nextButton;
        QPushButton *applyToPersonButton;
        QPushButton *applyToDeviceButton;
        QPushButton *retryDeviceButton;
        
        void answerCallback(const master_thesis_program::Answers::ConstPtr &msg);
        void sensorCallback(const std_msgs::Bool::ConstPtr &msg);
        void recordData();
};

