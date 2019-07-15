#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/WrenchStamped.h>
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
#include <QGroupBox>
#include <QButtonGroup>
#include <iostream>

class Control : public QWidget
{
    Q_OBJECT
    public:
        Control(QWidget *parent=0);

    protected:
        ros::NodeHandle nh;
        ros::Publisher pub_subjectA;
        ros::Publisher pub_subjectB;
        ros::Subscriber sub_subjectB;
        Emotions emotion_data;

    private Q_SLOTS:
        void nextSignal();
        void applyToPersonSignal();
        void applyToDeviceSignal();
        void newSubjectsSignal();

    private:
        int current_emotion;
        int current_emotion_index;
        std::vector<int> emotion_list = {1,2,3,4,5,6,7,8};
        geometry_msgs::WrenchStamped sensor_data;
        
        QLabel *info_label;
        QLabel *current_emotion_label;
        QLineEdit *subjectA_name_edit;
        QLineEdit *subjectB_name_edit;
        QPushButton *newButton;
        QPushButton *nextButton;
        QPushButton *applyToPersonButton;
        QPushButton *applyToDeviceButton;
        
        void answerCallback(const master_thesis_program::Answers::ConstPtr &msg);
        void sensorCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void recordData();
};

