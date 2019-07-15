#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <QApplication>
#include <QWidget>
#include <QStackedWidget>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QButtonGroup>

#include "master_thesis_program/Info.h"
#include "control.h"

Control::Control(QWidget *parent) : QWidget(parent), nh(), current_emotion(0), current_emotion_index(0)
{

    QVBoxLayout *main_layer = new QVBoxLayout;
   
    QHBoxLayout *subjectA_layer = new QHBoxLayout;
    QLabel *subjectA_label = new QLabel("subjectA name");
    subjectA_name_edit = new QLineEdit;
    subjectA_layer->addWidget(subjectA_label);
    subjectA_layer->addWidget(subjectA_name_edit);

    QHBoxLayout *subjectB_layer = new QHBoxLayout;
    QLabel *subjectB_label = new QLabel("subjectB name");
    subjectB_name_edit = new QLineEdit;
    subjectB_layer->addWidget(subjectB_label);
    subjectB_layer->addWidget(subjectB_name_edit);

    newButton = new QPushButton("new subject");
    main_layer->addLayout(subjectA_layer);
    main_layer->addLayout(subjectB_layer);
    main_layer->addWidget(newButton);

    info_label = new QLabel("Please Wait");
    info_label->setAlignment(Qt::AlignCenter);
    main_layer->addWidget(info_label);

    current_emotion_label = new QLabel("");
    current_emotion_label->setAlignment(Qt::AlignCenter);
    main_layer->addWidget(current_emotion_label);

    applyToPersonButton = new QPushButton("apply person");
    applyToDeviceButton = new QPushButton("apply device");
    nextButton = new QPushButton("next");
    applyToPersonButton->setEnabled(false);
    applyToDeviceButton->setEnabled(false);
    nextButton->setEnabled(false);
    main_layer->addWidget(applyToPersonButton);
    main_layer->addWidget(applyToDeviceButton);
    main_layer->addWidget(nextButton);

    connect(newButton, SIGNAL(clicked()), this, SLOT(newSubjectsSignal()));
    connect(nextButton, SIGNAL(clicked()), this, SLOT(nextSignal()));
    connect(applyToPersonButton, SIGNAL(clicked()), this, SLOT(applyToPersonSignal()));
    connect(applyToDeviceButton, SIGNAL(clicked()), this, SLOT(applyToDeviceSignal()));
    
    setLayout(main_layer);

    pub_subjectA = nh.advertise<master_thesis_program::Info>("/subjectA/info", 1);
    pub_subjectB = nh.advertise<std_msgs::Int32>("/subjectB/info", 1);
    sub_subjectB = nh.subscribe("/subjectB/answers", 1, &Control::answerCallback, this);
}

void
Control::newSubjectsSignal()
{
    std::string subjectA_name = subjectA_name_edit->text().toStdString();
    std::string subjectB_name = subjectB_name_edit->text().toStdString();
    if((subjectA_name != "") && (subjectB_name != "")){
        current_emotion_index = 0;
        applyToPersonButton->setEnabled(true);
        newButton->setEnabled(false);
        std::random_shuffle(emotion_list.begin(), emotion_list.end());
        current_emotion = emotion_list[current_emotion_index];
        current_emotion_label->setText(QString::fromStdString(emotion_data.name[current_emotion]));
        current_emotion_index++;
        //TODO: save name to file
    }else{
        info_label->setText("add subject name");
        current_emotion_label->setText("");
    }
}

void
Control::nextSignal()
{
    std_msgs::Int32 dataB;
    dataB.data = PLEASE_WAIT;
    pub_subjectB.publish(dataB);

    master_thesis_program::Info dataA;
    dataA.state = PLEASE_WAIT;
    dataA.emotion = NO_EMOTION;
    pub_subjectA.publish(dataA);

    nextButton->setEnabled(false);
    if(current_emotion_index >= emotion_list.size()){
        newButton->setEnabled(true);
        applyToPersonButton->setEnabled(false);
        applyToDeviceButton->setEnabled(false);
    }else{
        applyToPersonButton->setEnabled(true);
        applyToDeviceButton->setEnabled(false);
    }
    
    current_emotion = emotion_list[current_emotion_index];
    current_emotion_label->setText(QString::fromStdString(emotion_data.name[current_emotion]));
    current_emotion_index++;

    //TODO save target emotion to file
}

void
Control::applyToPersonSignal()
{
    info_label->setText("apply force to person");
    std_msgs::Int32 dataB;
    dataB.data = READY;
    pub_subjectB.publish(dataB);

    master_thesis_program::Info dataA;
    dataA.state = APPLY_FORCE_TO_PERSON;
    dataA.emotion = current_emotion;
    pub_subjectA.publish(dataA);

    applyToPersonButton->setEnabled(false);
    applyToDeviceButton->setEnabled(true);
}

void
Control::applyToDeviceSignal()
{
    info_label->setText("wait for info");
    std_msgs::Int32 dataB;
    dataB.data = ANSWER;
    pub_subjectB.publish(dataB);

    master_thesis_program::Info dataA;
    dataA.state = APPLY_FORCE_TO_DEVICE;
    dataA.emotion = current_emotion;
    pub_subjectA.publish(dataA);

    applyToDeviceButton->setEnabled(false);

    //TODO check if device open
    //ros::Rate rate(10);
    //ros::Time begin = ros::Time::now();
    //while((ros::Time::now().toSec() - begin.toSec()) < 10){
    //    recordData();
    //    rate.sleep();
    //}
    info_label->setText("wait for info");
}

void
Control::recordData()
{
    int seq = sensor_data.header.seq;
    float force_x = sensor_data.wrench.force.x;
    float force_y = sensor_data.wrench.force.y;
    float force_z = sensor_data.wrench.force.z;
    float torque_x = sensor_data.wrench.torque.x;
    float torque_y = sensor_data.wrench.torque.y;
    float torque_z = sensor_data.wrench.torque.z;
    //TODO write to file
}

void
Control::sensorCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    sensor_data.header = msg->header;
    sensor_data.wrench = msg->wrench;
}

void
Control::answerCallback(const master_thesis_program::Answers::ConstPtr &msg)
{
    nextButton->setEnabled(true);
    info_label->setText("receive answers");
    int first_guess = msg->first_guess;
    int second_guess = msg->second_guess;
    int third_guess = msg->third_guess;

    //TODO: save result to file, target, 1st estimate, 2nd estimate, 3rd estimate
    //make sure answer is obtained before any going to next experiment
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    QApplication app(argc, argv);
    QWidget *window = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout;
    
    Control *control_app = new Control(window);
    layout->addWidget(control_app);
    window->setLayout(layout);
    window->show();

    ros::Rate rate(20);
    while(ros::ok()){
        ros::spinOnce();
        app.processEvents();
        rate.sleep();
    }
}

