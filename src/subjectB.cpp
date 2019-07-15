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

#include "subjectB.h"
#include "emotions.h"
#include "experiment_signals.h"
#include "master_thesis_program/Answers.h"

SubjectB::SubjectB(QWidget *parent) : QWidget(parent), nh(), num_answers(0)
{
    QWidget *questionnaire_widget = new QWidget;
    QVBoxLayout *questionnaire_layer = new QVBoxLayout;
    questionnaire_label = new QLabel("1 guess");
    questionnaire_label->setAlignment(Qt::AlignCenter);
    questionnaire_layer->addWidget(questionnaire_label);
    anger_radio = new QRadioButton("anger");
    fear_radio = new QRadioButton("fear");
    disgust_radio = new QRadioButton("disgust");
    sad_radio = new QRadioButton("sad");
    sympathy_radio = new QRadioButton("sympathy");
    happy_radio = new QRadioButton("happy");
    gratitude_radio = new QRadioButton("gratitude");
    love_radio = new QRadioButton("love");
    questionnaire_layer->addWidget(anger_radio);
    questionnaire_layer->addWidget(fear_radio);
    questionnaire_layer->addWidget(disgust_radio);
    questionnaire_layer->addWidget(sad_radio);
    questionnaire_layer->addWidget(sympathy_radio);
    questionnaire_layer->addWidget(happy_radio);
    questionnaire_layer->addWidget(gratitude_radio);
    questionnaire_layer->addWidget(love_radio);
    button_group = new QButtonGroup;
    button_group->addButton(anger_radio);
    button_group->addButton(fear_radio);
    button_group->addButton(disgust_radio);
    button_group->addButton(sad_radio);
    button_group->addButton(sympathy_radio);
    button_group->addButton(happy_radio);
    button_group->addButton(gratitude_radio);
    button_group->addButton(love_radio);
    button_group->setId(anger_radio, ANGER);
    button_group->setId(fear_radio, FEAR);
    button_group->setId(disgust_radio, DISGUST);
    button_group->setId(sad_radio, SAD);
    button_group->setId(sympathy_radio, SYMPATHY);
    button_group->setId(happy_radio, HAPPY);
    button_group->setId(gratitude_radio, GRATITUDE);
    button_group->setId(love_radio, LOVE);
    confirm_button = new QPushButton("confirm");
    confirm_button->setEnabled(false);
    questionnaire_layer->addWidget(confirm_button);
    connect(confirm_button, SIGNAL(clicked()), this, SLOT(sendReady()));
    connect(button_group, SIGNAL(buttonClicked(int)), this, SLOT(enableConfirm()));
    questionnaire_widget->setLayout(questionnaire_layer);

    QWidget *info_widget = new QWidget;
    QVBoxLayout *info_layer = new QVBoxLayout;
    info_label = new QLabel("Please Wait");
    info_label->setAlignment(Qt::AlignCenter);
    info_layer->addWidget(info_label);
    info_widget->setLayout(info_layer);
    
    stacked_widget = new QStackedWidget;
    questionnaire_index = stacked_widget->addWidget(questionnaire_widget);    
    info_index = stacked_widget->addWidget(info_widget);
    stacked_widget->setCurrentIndex(info_index);
    QVBoxLayout *main_layer = new QVBoxLayout;
    main_layer->addWidget(stacked_widget);

    setLayout(main_layer);

    pub_answers = nh.advertise<master_thesis_program::Answers>("/subjectB/answers", 1);
    sub_response = nh.subscribe("/subjectB/info", 1, &SubjectB::responseCallback, this);
}

void
SubjectB::responseCallback(const std_msgs::Int32::ConstPtr &msg)
{
    if(msg->data){
        if(msg->data == ANSWER){
            stacked_widget->setCurrentIndex(questionnaire_index);
            info_label->setText("Please Wait");
        }else{
            stacked_widget->setCurrentIndex(info_index);
            if(msg->data == PLEASE_WAIT)
                info_label->setText("Please Wait");
            else if(msg->data == READY)
                info_label->setText("Ready");
        }
    }
}

void 
SubjectB::sendReady(){
    if(button_group->checkedId() > 0){
        answers.push_back(button_group->checkedId());
        num_answers++;
        if(num_answers >= 3){
            master_thesis_program::Answers data;
            data.first_guess = answers[0];
            data.second_guess = answers[1];
            data.third_guess = answers[2];
            pub_answers.publish(data);
            answers.clear();
            num_answers = 0;
            stacked_widget->setCurrentIndex(info_index);
        }
        QAbstractButton *checked = button_group->checkedButton();
        button_group->setExclusive(false);
        checked->setChecked(false);
        button_group->setExclusive(true);
        confirm_button->setEnabled(false);
        questionnaire_label->setText(QString::fromStdString(std::to_string(num_answers+1) + " guess"));
    }
}

void
SubjectB::enableConfirm(){
    confirm_button->setEnabled(true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subjectB");
    QApplication app(argc, argv);
    QWidget *window = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout;
    
    SubjectB *subjectB_app = new SubjectB(window);
    layout->addWidget(subjectB_app);
    window->setLayout(layout);
    window->show();

    ros::Rate rate(20);
    while(ros::ok()){
        ros::spinOnce();
        app.processEvents();
        rate.sleep();
    }
}

