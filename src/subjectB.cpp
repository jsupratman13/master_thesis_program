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
    QFont fonts("Arial", 40);

    QWidget *questionnaire_widget = new QWidget;
    QVBoxLayout *questionnaire_layer = new QVBoxLayout;
    questionnaire_label = new QLabel("1 guess");
    questionnaire_label->setFont(QFont("Arial", 20));
    questionnaire_label->setAlignment(Qt::AlignCenter);
    questionnaire_layer->addWidget(questionnaire_label);
    anger_radio = new QRadioButton("怒っている");
    anger_radio->setFont(fonts);
    fear_radio = new QRadioButton("怖がっている");
    fear_radio->setFont(fonts);
    disgust_radio = new QRadioButton("嫌悪している");
    disgust_radio->setFont(fonts);
    sad_radio = new QRadioButton("悲しい");
    sad_radio->setFont(fonts);
    happy_radio = new QRadioButton("楽しい");
    happy_radio->setFont(fonts);
    none_of_above_radio = new QRadioButton("どれでもない");
    none_of_above_radio->setFont(fonts);
    questionnaire_layer->addWidget(anger_radio);
    questionnaire_layer->addWidget(fear_radio);
    questionnaire_layer->addWidget(disgust_radio);
    questionnaire_layer->addWidget(sad_radio);
    questionnaire_layer->addWidget(happy_radio);
    questionnaire_layer->addWidget(none_of_above_radio);
    button_group = new QButtonGroup;
    button_group->addButton(anger_radio);
    button_group->addButton(fear_radio);
    button_group->addButton(disgust_radio);
    button_group->addButton(sad_radio);
    button_group->addButton(happy_radio);
    button_group->addButton(none_of_above_radio);
    button_group->setId(anger_radio, ANGER);
    button_group->setId(fear_radio, FEAR);
    button_group->setId(disgust_radio, DISGUST);
    button_group->setId(sad_radio, SAD);
    button_group->setId(happy_radio, HAPPY);
    button_group->setId(none_of_above_radio, UNKNOWN);
    confirm_button = new QPushButton("confirm");
    confirm_button->setEnabled(false);
    questionnaire_layer->addWidget(confirm_button);
    connect(confirm_button, SIGNAL(clicked()), this, SLOT(sendReady()));
    connect(button_group, SIGNAL(buttonClicked(int)), this, SLOT(enableConfirm()));
    questionnaire_widget->setLayout(questionnaire_layer);

    QWidget *info_widget = new QWidget;
    QVBoxLayout *info_layer = new QVBoxLayout;
    //info_label = new QLabel("Please Wait");
    info_label = new QLabel("少し待ってください");
    info_label->setAlignment(Qt::AlignCenter);
    info_label->setFont(fonts);
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
            //info_label->setText("Please Wait");
            info_label->setText("少し待ってください");
        }else{
            stacked_widget->setCurrentIndex(info_index);
            if(msg->data == PLEASE_WAIT)
                //info_label->setText("Please Wait");
                info_label->setText("少し待ってください");
            else if(msg->data == READY)
                //info_label->setText("Ready");
                info_label->setText("力を加えます");
        }
    }
}

void 
SubjectB::sendReady(){
    if(button_group->checkedId() > 0){
        int guess = button_group->checkedId();
        answers.push_back(guess);
        num_answers++;
        if(guess == UNKNOWN){
            for(int i=num_answers; i<3; i++){
                answers.push_back(UNKNOWN);
            }
            num_answers = 4;
        }

        if(num_answers >= 3){
            master_thesis_program::Answers data;
            data.first_guess = answers[0];
            data.second_guess = answers[1];
            data.third_guess = answers[2];
            pub_answers.publish(data);

            stacked_widget->setCurrentIndex(info_index);

            QAbstractButton *answers0 = button_group->button(answers[0]);
            QAbstractButton *answers1 = button_group->button(answers[1]);
            QAbstractButton *checked = button_group->checkedButton();
            button_group->setExclusive(false);
            answers0->setEnabled(true);
            answers1->setEnabled(true);
            checked->setChecked(false);
            button_group->setExclusive(true);

            questionnaire_label->setText(QString::fromStdString(std::to_string(num_answers+1) + " guess"));
            num_answers = 0;
            answers.clear();
        }
        else{

            QAbstractButton *checked = button_group->checkedButton();
            button_group->setExclusive(false);
            checked->setChecked(false);
            checked->setEnabled(false);
            button_group->setExclusive(true);
            confirm_button->setEnabled(false);
            questionnaire_label->setText(QString::fromStdString(std::to_string(num_answers+1) + " guess"));
        }
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

