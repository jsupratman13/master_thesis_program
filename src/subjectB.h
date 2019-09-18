#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Int32.h>
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

class SubjectB : public QWidget
{
    Q_OBJECT
    public:
        SubjectB(QWidget *parent=0);

    protected:
        ros::NodeHandle nh;
        ros::Publisher pub_answers;
        ros::Subscriber sub_response;

    private Q_SLOTS:
        void sendReady();
        void enableConfirm();

    private:
        QStackedWidget *stacked_widget;
        int questionnaire_index;
        int info_index;
        QLabel *questionnaire_label;
        QLabel *info_label;
        
        QButtonGroup *button_group;
        QRadioButton *anger_radio;
        QRadioButton *fear_radio;
        QRadioButton *disgust_radio;
        QRadioButton *sad_radio;
        //QRadioButton *sympathy_radio;
        QRadioButton *happy_radio;
        //QRadioButton *gratitude_radio;
        //QRadioButton *love_radio;
        QRadioButton *none_of_above_radio;

        QPushButton *confirm_button;
        void responseCallback(const std_msgs::Int32::ConstPtr &msg);
        int num_answers;
        std::vector<int> answers;
};
