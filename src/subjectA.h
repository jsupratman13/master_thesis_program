#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "master_thesis_program/Info.h"
#include "emotions.h"
#include "experiment_signals.h"
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

class SubjectA : public QWidget
{
    Q_OBJECT
    public:
        SubjectA(QWidget *parent=0);

    protected:
        ros::NodeHandle nh;
        ros::Subscriber sub_info;

    private:
        Emotions emotion_data;
        QLabel *emotion_label;
        QLabel *info_label;
        
        void infoCallback(const master_thesis_program::Info::ConstPtr &msg);
};
