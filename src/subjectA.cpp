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
#include "subjectA.h"

SubjectA::SubjectA(QWidget *parent) : QWidget(parent), nh()
{
    QFont fonts("Arial", 40);

    QWidget *info_widget = new QWidget;
    QVBoxLayout *info_layer = new QVBoxLayout;
    //info_label = new QLabel("Please Wait");
    info_label = new QLabel("少し待ってください");
    info_label->setAlignment(Qt::AlignCenter);
    info_label->setFont(fonts);
    emotion_label = new QLabel("");
    emotion_label->setAlignment(Qt::AlignCenter);
    emotion_label->setFont(fonts);

    info_layer->addWidget(info_label);
    info_layer->addWidget(emotion_label);
    info_widget->setLayout(info_layer);
    
    setLayout(info_layer);

    sub_info = nh.subscribe("/subjectA/info", 1, &SubjectA::infoCallback, this);
}

void
SubjectA::infoCallback(const master_thesis_program::Info::ConstPtr &msg)
{
    int state = msg->state;
    int emotion = msg->emotion;
    switch(state){
        case APPLY_FORCE_TO_PERSON:
            //info_label->setText("Apply Force To Person");
            info_label->setText("人の肩に力を加えてください");
            break;
        case APPLY_FORCE_TO_DEVICE:
            info_label->setText("半球に力を加えてください");
            break;
        default:
            info_label->setText("少し待ってください");
            break;
    }
    if(emotion){
        emotion_label->setText(QString::fromStdString(emotion_data.nameJP[emotion]));
    }else{
        emotion_label->setText("");
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subjectA");
    QApplication app(argc, argv);
    QWidget *window = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout;
    
    SubjectA *subjectA_app = new SubjectA(window);
    layout->addWidget(subjectA_app);
    window->setLayout(layout);
    window->show();

    ros::Rate rate(20);
    while(ros::ok()){
        ros::spinOnce();
        app.processEvents();
        rate.sleep();
    }
}

