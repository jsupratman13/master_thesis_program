#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
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

Control::Control(QWidget *parent) : QWidget(parent), nh(), current_emotion(0), current_emotion_index(0), subjectA_finished(false), subjectB_finished(false)
{

    QVBoxLayout *main_layer = new QVBoxLayout;

    QHBoxLayout *experiment_number_layer = new QHBoxLayout;
    QLabel *ex_no_label = new QLabel("experiment no");
    ex_no = new QSpinBox;
    ex_no->setRange(1, 100);
    experiment_number_layer->addWidget(ex_no_label);
    experiment_number_layer->addWidget(ex_no);
    main_layer->addLayout(experiment_number_layer);

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
    main_layer->addWidget(applyToPersonButton);
    applyToPersonButton->setEnabled(false);
    
    QHBoxLayout *device_layer = new QHBoxLayout;
    applyToDeviceButton = new QPushButton("apply device");
    applyToDeviceButton->setEnabled(false);
    device_layer->addWidget(applyToDeviceButton);
    retryDeviceButton = new QPushButton("retry device");
    retryDeviceButton->setEnabled(false);
    device_layer->addWidget(retryDeviceButton);
    main_layer->addLayout(device_layer);

    nextButton = new QPushButton("next");
    nextButton->setEnabled(false);
    main_layer->addWidget(nextButton);

    connect(newButton, SIGNAL(clicked()), this, SLOT(newSubjectsSignal()));
    connect(nextButton, SIGNAL(clicked()), this, SLOT(nextSignal()));
    connect(applyToPersonButton, SIGNAL(clicked()), this, SLOT(applyToPersonSignal()));
    connect(applyToDeviceButton, SIGNAL(clicked()), this, SLOT(applyToDeviceSignal()));
    connect(retryDeviceButton, SIGNAL(clicked()), this, SLOT(retryDeviceSignal()));
    
    setLayout(main_layer);

    pub_subjectA = nh.advertise<master_thesis_program::Info>("/subjectA/info", 1);
    pub_record   = nh.advertise<std_msgs::String>("/subjectA/record_device", 1);
    pub_subjectB = nh.advertise<std_msgs::Int32>("/subjectB/info", 1);
    sub_subjectA = nh.subscribe("/subjectA/response_device", 1, &Control::sensorCallback, this);
    sub_subjectB = nh.subscribe("/subjectB/answers", 1, &Control::answerCallback, this);
}

void
Control::newSubjectsSignal()
{
    std::string subjectA_name = subjectA_name_edit->text().toStdString();
    std::string subjectB_name = subjectB_name_edit->text().toStdString();
    if((subjectA_name != "") && (subjectB_name != "")){
        current_emotion_index = 0;

        ex_no->setEnabled(false);
        applyToPersonButton->setEnabled(true);
        newButton->setEnabled(false);

        std::random_shuffle(emotion_list.begin(), emotion_list.end());
        current_emotion = emotion_list[current_emotion_index];
        current_emotion_label->setText(QString::fromStdString(emotion_data.name[current_emotion]));
        current_emotion_index++;

        ex_result.open(path + "ex" + std::to_string(ex_no->value()) + "_result.csv");
        ex_result << "target emotion, 1st guess, 2nd guess, 3rd guess" << std::endl;

        ex_info.open(path + "ex" + std::to_string(ex_no->value()) + "_info.csv");
        ex_info << "experiment id, subjectA, subjectB" << std::endl;
        ex_info << std::to_string(ex_no->value()) << ",";
        ex_info << subjectA_name << "," << subjectB_name << std::endl;
        ex_info.close();

    }else{
        info_label->setText("add subject name");
        current_emotion_label->setText("");
    }
}

void
Control::nextSignal()
{
    subjectA_finished = false;
    subjectB_finished = false;

    std_msgs::Int32 dataB;
    dataB.data = PLEASE_WAIT;
    pub_subjectB.publish(dataB);

    master_thesis_program::Info dataA;
    dataA.state = PLEASE_WAIT;
    dataA.emotion = NO_EMOTION;
    pub_subjectA.publish(dataA);

    nextButton->setEnabled(false);
    if(current_emotion_index >= emotion_list.size()){
        try{
            ex_result.close();
        } catch(std::exception &e){
            std::cout << e.what() << std::endl;
        }

        ex_no->setValue(ex_no->value() + 1);
        ex_no->setEnabled(true);
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

    std_msgs::String data;
    data.data = "ex" + std::to_string(ex_no->value()) + "_" + emotion_data.name[current_emotion] + ".csv";
    pub_record.publish(data);
}

void
Control::retryDeviceSignal()
{
    info_label->setText("retry");
    master_thesis_program::Info dataA;
    dataA.state = APPLY_FORCE_TO_DEVICE;
    dataA.emotion = current_emotion;
    pub_subjectA.publish(dataA);

    retryDeviceButton->setEnabled(false);

    std_msgs::String data;
    data.data = "ex" + std::to_string(ex_no->value()) + "_" + emotion_data.name[current_emotion] + ".csv";
    pub_record.publish(data);
}

void
Control::sensorCallback(const std_msgs::Bool::ConstPtr &msg)
{
    subjectA_finished = msg->data;
    if(subjectA_finished && subjectB_finished){
        info_label->setText("receive data");
        nextButton->setEnabled(true);
    }
    else if(!subjectA_finished){
        info_label->setText("device failed retry");
        retryDeviceButton->setEnabled(true);
    }
}

void
Control::answerCallback(const master_thesis_program::Answers::ConstPtr &msg)
{
    subjectB_finished = true;
    if(subjectA_finished && subjectB_finished){
        nextButton->setEnabled(true);
    }

    info_label->setText("receive answers");
    int first_guess = msg->first_guess;
    int second_guess = msg->second_guess;
    int third_guess = msg->third_guess;

    ex_result << emotion_data.name[current_emotion] << ",";
    ex_result << emotion_data.name[first_guess] << ","; 
    ex_result << emotion_data.name[second_guess] << ",";
    ex_result << emotion_data.name[third_guess] << std::endl;
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

