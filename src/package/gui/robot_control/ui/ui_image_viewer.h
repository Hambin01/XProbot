/********************************************************************************
** Form generated from reading UI file 'image_viewer.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGE_VIEWER_H
#define UI_IMAGE_VIEWER_H

#include <QtCore/QVariant>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QWidget>

QT_BEGIN_NAMESPACE

class Ui_image_viewer_form
{
public:
    QGridLayout *gridLayout_3;
    QGroupBox *groupBox_11;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_11;
    QDoubleSpinBox *curTask;
    QLabel *label_12;
    QProgressBar *battery;
    QLabel *label_18;
    QDoubleSpinBox *chargeState;
    QLabel *label_17;
    QDoubleSpinBox *ultrasonic;
    QHBoxLayout *horizontalLayout;
    QLabel *label_13;
    QDoubleSpinBox *speed;
    QLabel *label_14;
    QDoubleSpinBox *radius;
    QLabel *label_16;
    QDoubleSpinBox *erroCode;
    QLabel *label_15;
    QDoubleSpinBox *warnning;
    QGroupBox *groupBox_6;
    QGridLayout *gridLayout_4;
    QHBoxLayout *horizontalLayout_5;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_7;
    QDoubleSpinBox *goal_pose_x;
    QLabel *label_8;
    QDoubleSpinBox *goal_pose_y;
    QLabel *label_5;
    QDoubleSpinBox *robot_pose_x;
    QLabel *label_6;
    QDoubleSpinBox *robot_pose_y;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *nodeClear;
    QPushButton *importNode;
    QPushButton *saveNode;
    QPushButton *ConnectSet;
    QPushButton *deleteSet;
    QGridLayout *gridLayout_6;
    QPushButton *setGoal;
    QLabel *label_20;
    QComboBox *comboBox;
    QLabel *label_4;
    QLabel *label_21;
    QComboBox *crossBox;
    QDoubleSpinBox *putterTaskHeight;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_9;
    QLineEdit *newNodeName;
    QPushButton *insertNode;
    QPushButton *setDirection;
    QPushButton *setCharge;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *startNavigation;
    QPushButton *stopNavigation;
    QPushButton *clearPath;
    QPushButton *startCharge;
    QPushButton *stopCharge;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QPushButton *robot_back;
    QLabel *label_10;
    QPushButton *robot_go;
    QPushButton *robot_left;
    QDoubleSpinBox *crossSpeed;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label;
    QDoubleSpinBox *robot_speed;
    QLabel *label_2;
    QDoubleSpinBox *robot_radius;
    QPushButton *robot_right;
    QPushButton *crossSpeedSet;
    QPushButton *crossDown;
    QPushButton *crossUp;
    QPushButton *agvInit;

    void setupUi(QWidget *image_viewer_form)
    {
        if (image_viewer_form->objectName().isEmpty())
            image_viewer_form->setObjectName(QString::fromUtf8("image_viewer_form"));
        image_viewer_form->resize(572, 500);
        image_viewer_form->setMinimumSize(QSize(572, 500));
        image_viewer_form->setStyleSheet(QString::fromUtf8("border-color: rgb(122, 122, 122);"));
        gridLayout_3 = new QGridLayout(image_viewer_form);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        groupBox_11 = new QGroupBox(image_viewer_form);
        groupBox_11->setObjectName(QString::fromUtf8("groupBox_11"));
        groupBox_11->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"            border-color: rgb(156, 156, 156);\n"
"            border-width: 2px;\n"
"            border-style: solid;\n"
"            margin-top:1ex;   \n"
"            margin-bottom: 2ex;\n"
"        }\n"
"        QGroupBox::title {\n"
"            subcontrol-origin: margin;\n"
"            subcontrol-position: top left;\n"
"            left: 10px;\n"
"            margin-left: 1px;\n"
"            padding:2px;\n"
"        }"));
        gridLayout_2 = new QGridLayout(groupBox_11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_11 = new QLabel(groupBox_11);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        horizontalLayout_2->addWidget(label_11);

        curTask = new QDoubleSpinBox(groupBox_11);
        curTask->setObjectName(QString::fromUtf8("curTask"));
        curTask->setMouseTracking(false);
        curTask->setReadOnly(true);
        curTask->setDecimals(0);
        curTask->setMaximum(20.000000000000000);

        horizontalLayout_2->addWidget(curTask);

        label_12 = new QLabel(groupBox_11);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setTextFormat(Qt::AutoText);

        horizontalLayout_2->addWidget(label_12);

        battery = new QProgressBar(groupBox_11);
        battery->setObjectName(QString::fromUtf8("battery"));
        battery->setValue(0);

        horizontalLayout_2->addWidget(battery);

        label_18 = new QLabel(groupBox_11);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_2->addWidget(label_18);

        chargeState = new QDoubleSpinBox(groupBox_11);
        chargeState->setObjectName(QString::fromUtf8("chargeState"));
        chargeState->setReadOnly(true);
        chargeState->setDecimals(0);
        chargeState->setMaximum(3.000000000000000);

        horizontalLayout_2->addWidget(chargeState);

        label_17 = new QLabel(groupBox_11);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_2->addWidget(label_17);

        ultrasonic = new QDoubleSpinBox(groupBox_11);
        ultrasonic->setObjectName(QString::fromUtf8("ultrasonic"));
        ultrasonic->setReadOnly(true);
        ultrasonic->setDecimals(0);
        ultrasonic->setMaximum(5200.000000000000000);

        horizontalLayout_2->addWidget(ultrasonic);


        gridLayout_2->addLayout(horizontalLayout_2, 0, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_13 = new QLabel(groupBox_11);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout->addWidget(label_13);

        speed = new QDoubleSpinBox(groupBox_11);
        speed->setObjectName(QString::fromUtf8("speed"));
        speed->setReadOnly(true);
        speed->setDecimals(3);
        speed->setMinimum(-1.000000000000000);
        speed->setMaximum(1.000000000000000);

        horizontalLayout->addWidget(speed);

        label_14 = new QLabel(groupBox_11);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout->addWidget(label_14);

        radius = new QDoubleSpinBox(groupBox_11);
        radius->setObjectName(QString::fromUtf8("radius"));
        radius->setReadOnly(true);
        radius->setDecimals(3);
        radius->setMinimum(-1.000000000000000);
        radius->setMaximum(1.000000000000000);

        horizontalLayout->addWidget(radius);

        label_16 = new QLabel(groupBox_11);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        horizontalLayout->addWidget(label_16);

        erroCode = new QDoubleSpinBox(groupBox_11);
        erroCode->setObjectName(QString::fromUtf8("erroCode"));
        erroCode->setReadOnly(true);
        erroCode->setDecimals(0);
        erroCode->setMaximum(20.000000000000000);

        horizontalLayout->addWidget(erroCode);

        label_15 = new QLabel(groupBox_11);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        horizontalLayout->addWidget(label_15);

        warnning = new QDoubleSpinBox(groupBox_11);
        warnning->setObjectName(QString::fromUtf8("warnning"));
        warnning->setReadOnly(true);
        warnning->setDecimals(0);
        warnning->setMaximum(20.000000000000000);

        horizontalLayout->addWidget(warnning);


        gridLayout_2->addLayout(horizontalLayout, 1, 0, 1, 1);


        gridLayout_3->addWidget(groupBox_11, 0, 0, 1, 1);

        groupBox_6 = new QGroupBox(image_viewer_form);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"            border-color: rgb(156, 156, 156);\n"
"            border-width: 2px;\n"
"            border-style: solid;\n"
"            margin-top:1ex;   \n"
"            margin-bottom: 2ex;\n"
"        }\n"
"        QGroupBox::title {\n"
"            subcontrol-origin: margin;\n"
"            subcontrol-position: top left;\n"
"            left: 10px;\n"
"            margin-left: 1px;\n"
"            padding:2px;\n"
"        }"));
        gridLayout_4 = new QGridLayout(groupBox_6);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));

        gridLayout_4->addLayout(horizontalLayout_5, 0, 0, 1, 1);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_7 = new QLabel(groupBox_6);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_8->addWidget(label_7);

        goal_pose_x = new QDoubleSpinBox(groupBox_6);
        goal_pose_x->setObjectName(QString::fromUtf8("goal_pose_x"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(goal_pose_x->sizePolicy().hasHeightForWidth());
        goal_pose_x->setSizePolicy(sizePolicy);
        goal_pose_x->setReadOnly(true);
        goal_pose_x->setDecimals(3);
        goal_pose_x->setMinimum(-1000.000000000000000);
        goal_pose_x->setMaximum(1000.000000000000000);

        horizontalLayout_8->addWidget(goal_pose_x);

        label_8 = new QLabel(groupBox_6);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_8->addWidget(label_8);

        goal_pose_y = new QDoubleSpinBox(groupBox_6);
        goal_pose_y->setObjectName(QString::fromUtf8("goal_pose_y"));
        goal_pose_y->setDecimals(3);
        goal_pose_y->setMinimum(-1000.000000000000000);
        goal_pose_y->setMaximum(1000.000000000000000);

        horizontalLayout_8->addWidget(goal_pose_y);

        label_5 = new QLabel(groupBox_6);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_8->addWidget(label_5);

        robot_pose_x = new QDoubleSpinBox(groupBox_6);
        robot_pose_x->setObjectName(QString::fromUtf8("robot_pose_x"));
        robot_pose_x->setReadOnly(true);
        robot_pose_x->setDecimals(3);
        robot_pose_x->setMinimum(-1000.000000000000000);
        robot_pose_x->setMaximum(1000.000000000000000);

        horizontalLayout_8->addWidget(robot_pose_x);

        label_6 = new QLabel(groupBox_6);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_8->addWidget(label_6);

        robot_pose_y = new QDoubleSpinBox(groupBox_6);
        robot_pose_y->setObjectName(QString::fromUtf8("robot_pose_y"));
        robot_pose_y->setReadOnly(true);
        robot_pose_y->setDecimals(3);
        robot_pose_y->setMinimum(-1000.000000000000000);
        robot_pose_y->setMaximum(1000.000000000000000);

        horizontalLayout_8->addWidget(robot_pose_y);


        gridLayout_4->addLayout(horizontalLayout_8, 1, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        nodeClear = new QPushButton(groupBox_6);
        nodeClear->setObjectName(QString::fromUtf8("nodeClear"));

        horizontalLayout_3->addWidget(nodeClear);

        importNode = new QPushButton(groupBox_6);
        importNode->setObjectName(QString::fromUtf8("importNode"));

        horizontalLayout_3->addWidget(importNode);

        saveNode = new QPushButton(groupBox_6);
        saveNode->setObjectName(QString::fromUtf8("saveNode"));

        horizontalLayout_3->addWidget(saveNode);

        ConnectSet = new QPushButton(groupBox_6);
        ConnectSet->setObjectName(QString::fromUtf8("ConnectSet"));
        ConnectSet->setMouseTracking(false);

        horizontalLayout_3->addWidget(ConnectSet);

        deleteSet = new QPushButton(groupBox_6);
        deleteSet->setObjectName(QString::fromUtf8("deleteSet"));

        horizontalLayout_3->addWidget(deleteSet);


        gridLayout_4->addLayout(horizontalLayout_3, 2, 0, 1, 1);

        gridLayout_6 = new QGridLayout();
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        setGoal = new QPushButton(groupBox_6);
        setGoal->setObjectName(QString::fromUtf8("setGoal"));

        gridLayout_6->addWidget(setGoal, 0, 6, 1, 1);

        label_20 = new QLabel(groupBox_6);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        gridLayout_6->addWidget(label_20, 0, 4, 1, 1);

        comboBox = new QComboBox(groupBox_6);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout_6->addWidget(comboBox, 0, 1, 1, 1);

        label_4 = new QLabel(groupBox_6);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_6->addWidget(label_4, 0, 0, 1, 1);

        label_21 = new QLabel(groupBox_6);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        gridLayout_6->addWidget(label_21, 0, 2, 1, 1);

        crossBox = new QComboBox(groupBox_6);
        crossBox->setObjectName(QString::fromUtf8("crossBox"));

        gridLayout_6->addWidget(crossBox, 0, 5, 1, 1);

        putterTaskHeight = new QDoubleSpinBox(groupBox_6);
        putterTaskHeight->setObjectName(QString::fromUtf8("putterTaskHeight"));
        putterTaskHeight->setReadOnly(true);
        putterTaskHeight->setDecimals(1);
        putterTaskHeight->setMaximum(1000.000000000000000);

        gridLayout_6->addWidget(putterTaskHeight, 0, 3, 1, 1);


        gridLayout_4->addLayout(gridLayout_6, 3, 0, 1, 1);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_9 = new QLabel(groupBox_6);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_6->addWidget(label_9);

        newNodeName = new QLineEdit(groupBox_6);
        newNodeName->setObjectName(QString::fromUtf8("newNodeName"));

        horizontalLayout_6->addWidget(newNodeName);

        insertNode = new QPushButton(groupBox_6);
        insertNode->setObjectName(QString::fromUtf8("insertNode"));

        horizontalLayout_6->addWidget(insertNode);

        setDirection = new QPushButton(groupBox_6);
        setDirection->setObjectName(QString::fromUtf8("setDirection"));

        horizontalLayout_6->addWidget(setDirection);

        setCharge = new QPushButton(groupBox_6);
        setCharge->setObjectName(QString::fromUtf8("setCharge"));

        horizontalLayout_6->addWidget(setCharge);


        gridLayout_4->addLayout(horizontalLayout_6, 4, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        startNavigation = new QPushButton(groupBox_6);
        startNavigation->setObjectName(QString::fromUtf8("startNavigation"));

        horizontalLayout_4->addWidget(startNavigation);

        stopNavigation = new QPushButton(groupBox_6);
        stopNavigation->setObjectName(QString::fromUtf8("stopNavigation"));

        horizontalLayout_4->addWidget(stopNavigation);

        clearPath = new QPushButton(groupBox_6);
        clearPath->setObjectName(QString::fromUtf8("clearPath"));

        horizontalLayout_4->addWidget(clearPath);

        startCharge = new QPushButton(groupBox_6);
        startCharge->setObjectName(QString::fromUtf8("startCharge"));

        horizontalLayout_4->addWidget(startCharge);

        stopCharge = new QPushButton(groupBox_6);
        stopCharge->setObjectName(QString::fromUtf8("stopCharge"));

        horizontalLayout_4->addWidget(stopCharge);


        gridLayout_4->addLayout(horizontalLayout_4, 5, 0, 1, 1);


        gridLayout_3->addWidget(groupBox_6, 2, 0, 1, 1);

        groupBox = new QGroupBox(image_viewer_form);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setAutoFillBackground(false);
        groupBox->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"            border-color: rgb(156, 156, 156);\n"
"            border-width: 2px;\n"
"            border-style: solid;\n"
"            margin-top:1ex;   \n"
"            margin-bottom: 2ex;\n"
"        }\n"
"        QGroupBox::title {\n"
"            subcontrol-origin: margin;\n"
"            subcontrol-position: top left;\n"
"            left: 10px;\n"
"            margin-left: 1px;\n"
"            padding:2px;\n"
"        }"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        robot_back = new QPushButton(groupBox);
        robot_back->setObjectName(QString::fromUtf8("robot_back"));

        gridLayout->addWidget(robot_back, 1, 1, 1, 1);

        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 2, 4, 1, 1);

        robot_go = new QPushButton(groupBox);
        robot_go->setObjectName(QString::fromUtf8("robot_go"));

        gridLayout->addWidget(robot_go, 0, 1, 1, 1);

        robot_left = new QPushButton(groupBox);
        robot_left->setObjectName(QString::fromUtf8("robot_left"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(robot_left->sizePolicy().hasHeightForWidth());
        robot_left->setSizePolicy(sizePolicy1);
        robot_left->setStyleSheet(QString::fromUtf8(""));

        gridLayout->addWidget(robot_left, 0, 0, 2, 1);

        crossSpeed = new QDoubleSpinBox(groupBox);
        crossSpeed->setObjectName(QString::fromUtf8("crossSpeed"));
        crossSpeed->setDecimals(0);
        crossSpeed->setMinimum(0.000000000000000);
        crossSpeed->setMaximum(300.000000000000000);

        gridLayout->addWidget(crossSpeed, 2, 5, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_7->addWidget(label);

        robot_speed = new QDoubleSpinBox(groupBox);
        robot_speed->setObjectName(QString::fromUtf8("robot_speed"));
        robot_speed->setDecimals(3);
        robot_speed->setMinimum(0.000000000000000);
        robot_speed->setMaximum(1.000000000000000);
        robot_speed->setSingleStep(0.010000000000000);
        robot_speed->setValue(0.100000000000000);

        horizontalLayout_7->addWidget(robot_speed);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_7->addWidget(label_2);

        robot_radius = new QDoubleSpinBox(groupBox);
        robot_radius->setObjectName(QString::fromUtf8("robot_radius"));
        robot_radius->setMaximum(1.000000000000000);
        robot_radius->setSingleStep(0.010000000000000);
        robot_radius->setValue(0.050000000000000);

        horizontalLayout_7->addWidget(robot_radius);


        gridLayout->addLayout(horizontalLayout_7, 2, 0, 1, 4);

        robot_right = new QPushButton(groupBox);
        robot_right->setObjectName(QString::fromUtf8("robot_right"));

        gridLayout->addWidget(robot_right, 0, 2, 2, 1);

        crossSpeedSet = new QPushButton(groupBox);
        crossSpeedSet->setObjectName(QString::fromUtf8("crossSpeedSet"));

        gridLayout->addWidget(crossSpeedSet, 2, 6, 1, 1);

        crossDown = new QPushButton(groupBox);
        crossDown->setObjectName(QString::fromUtf8("crossDown"));

        gridLayout->addWidget(crossDown, 1, 6, 1, 1);

        crossUp = new QPushButton(groupBox);
        crossUp->setObjectName(QString::fromUtf8("crossUp"));

        gridLayout->addWidget(crossUp, 0, 6, 1, 1);

        agvInit = new QPushButton(groupBox);
        agvInit->setObjectName(QString::fromUtf8("agvInit"));

        gridLayout->addWidget(agvInit, 1, 5, 1, 1);


        gridLayout_3->addWidget(groupBox, 1, 0, 1, 1);


        retranslateUi(image_viewer_form);

        QMetaObject::connectSlotsByName(image_viewer_form);
    } // setupUi

    void retranslateUi(QWidget *image_viewer_form)
    {
        image_viewer_form->setWindowTitle(QApplication::translate("image_viewer_form", "Form", nullptr));
        groupBox_11->setTitle(QApplication::translate("image_viewer_form", "RobotState", nullptr));
        label_11->setText(QApplication::translate("image_viewer_form", "CurTask:", nullptr));
#ifndef QT_NO_TOOLTIP
        curTask->setToolTip(QApplication::translate("image_viewer_form", "<html><head/><body><p>RobotIdle	= 0,   </p><p>RobotRadiusRunning =1,     </p><p>RobotAngleRotating =2,    </p><p>RobotIniting =3,    </p><p>RobotCharging=4,     </p><p>RobotPausing=5,     </p><p>RobotFaulting=6,    </p><p>RobotSleeping=7,     </p><p>RobotPuttering=8,    </p><p>RobotCrossing=9, 		</p><p>RobotForcePutterReseting=10</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        curTask->setWhatsThis(QString());
#endif // QT_NO_WHATSTHIS
        label_12->setText(QApplication::translate("image_viewer_form", "battery", nullptr));
        label_18->setText(QApplication::translate("image_viewer_form", "Charge", nullptr));
        label_17->setText(QApplication::translate("image_viewer_form", "Ultrasonic:", nullptr));
        label_13->setText(QApplication::translate("image_viewer_form", "Speed:", nullptr));
        label_14->setText(QApplication::translate("image_viewer_form", "Radius:", nullptr));
        label_16->setText(QApplication::translate("image_viewer_form", "Error:", nullptr));
        label_15->setText(QApplication::translate("image_viewer_form", "Warnning:", nullptr));
#ifndef QT_NO_TOOLTIP
        warnning->setToolTip(QApplication::translate("image_viewer_form", "<html><head/><body><p>NoWarnning	= 0,</p><p>LowBattery	 = 1,</p><p>BatteryCommError   = 2,</p><p>MpuError = 3,</p><p>UltrasonicDetected = 4,</p><p>TemperatureWarn  = 5,</p><p>Temperaturecomm    = 6,</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        groupBox_6->setTitle(QApplication::translate("image_viewer_form", "Navigation", nullptr));
        label_7->setText(QApplication::translate("image_viewer_form", "goal_x:", nullptr));
        label_8->setText(QApplication::translate("image_viewer_form", "goal_y:", nullptr));
        label_5->setText(QApplication::translate("image_viewer_form", "x:", nullptr));
        label_6->setText(QApplication::translate("image_viewer_form", "y:", nullptr));
        nodeClear->setText(QApplication::translate("image_viewer_form", "New Node List", nullptr));
        importNode->setText(QApplication::translate("image_viewer_form", "Import From", nullptr));
        saveNode->setText(QApplication::translate("image_viewer_form", "Save", nullptr));
        ConnectSet->setText(QApplication::translate("image_viewer_form", "ConnectStart", nullptr));
        deleteSet->setText(QApplication::translate("image_viewer_form", "DeleteStart", nullptr));
        setGoal->setText(QApplication::translate("image_viewer_form", "Set Path Node", nullptr));
        label_20->setText(QApplication::translate("image_viewer_form", "Cross", nullptr));
        label_4->setText(QApplication::translate("image_viewer_form", "Node", nullptr));
        label_21->setText(QApplication::translate("image_viewer_form", "Putter", nullptr));
        label_9->setText(QApplication::translate("image_viewer_form", "NodeName", nullptr));
        insertNode->setText(QApplication::translate("image_viewer_form", "Insert", nullptr));
        setDirection->setText(QApplication::translate("image_viewer_form", "setDirection", nullptr));
        setCharge->setText(QApplication::translate("image_viewer_form", "setCharge", nullptr));
        startNavigation->setText(QApplication::translate("image_viewer_form", "Start Navigation", nullptr));
        stopNavigation->setText(QApplication::translate("image_viewer_form", "stop", nullptr));
        clearPath->setText(QApplication::translate("image_viewer_form", "Clear Path", nullptr));
        startCharge->setText(QApplication::translate("image_viewer_form", "Start Charge", nullptr));
        stopCharge->setText(QApplication::translate("image_viewer_form", "Stop Charge", nullptr));
        groupBox->setTitle(QApplication::translate("image_viewer_form", "Robot Control", nullptr));
        robot_back->setText(QApplication::translate("image_viewer_form", "Back", nullptr));
        label_10->setText(QApplication::translate("image_viewer_form", "putter", nullptr));
        robot_go->setText(QApplication::translate("image_viewer_form", "Go", nullptr));
        robot_left->setText(QApplication::translate("image_viewer_form", "Left", nullptr));
        label->setText(QApplication::translate("image_viewer_form", "Speed", nullptr));
        label_2->setText(QApplication::translate("image_viewer_form", "Radius", nullptr));
        robot_right->setText(QApplication::translate("image_viewer_form", "Right", nullptr));
        crossSpeedSet->setText(QApplication::translate("image_viewer_form", "Set", nullptr));
        crossDown->setText(QApplication::translate("image_viewer_form", "CrossDown", nullptr));
        crossUp->setText(QApplication::translate("image_viewer_form", "CrossUp", nullptr));
        agvInit->setText(QApplication::translate("image_viewer_form", "AgvInit", nullptr));
    } // retranslateUi

};

namespace Ui {
    class image_viewer_form: public Ui_image_viewer_form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGE_VIEWER_H
