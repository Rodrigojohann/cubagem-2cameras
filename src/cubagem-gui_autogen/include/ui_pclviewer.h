/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.11.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QVTKWidget *qvtkWidget;
    QGroupBox *groupBox_ColorMode;
    QGridLayout *gridLayout_2;
    QLabel *label_2;
    QLabel *label_1;
    QPushButton *pushButton_2;
    QPushButton *pushButton_1;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->setWindowModality(Qt::NonModal);
        PCLViewer->resize(920, 678);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy);
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(50);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy1);
        qvtkWidget->setMinimumSize(QSize(640, 480));
        qvtkWidget->setLayoutDirection(Qt::LeftToRight);

        gridLayout->addWidget(qvtkWidget, 0, 0, 1, 1);

        groupBox_ColorMode = new QGroupBox(centralwidget);
        groupBox_ColorMode->setObjectName(QStringLiteral("groupBox_ColorMode"));
        sizePolicy.setHeightForWidth(groupBox_ColorMode->sizePolicy().hasHeightForWidth());
        groupBox_ColorMode->setSizePolicy(sizePolicy);
        groupBox_ColorMode->setMinimumSize(QSize(230, 180));
        QFont font;
        font.setPointSize(16);
        font.setBold(true);
        font.setWeight(75);
        groupBox_ColorMode->setFont(font);
        groupBox_ColorMode->setLayoutDirection(Qt::LeftToRight);
        groupBox_ColorMode->setAlignment(Qt::AlignCenter);
        gridLayout_2 = new QGridLayout(groupBox_ColorMode);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label_2 = new QLabel(groupBox_ColorMode);
        label_2->setObjectName(QStringLiteral("label_2"));
        QFont font1;
        font1.setPointSize(37);
        font1.setBold(false);
        font1.setWeight(50);
        label_2->setFont(font1);
        label_2->setLayoutDirection(Qt::LeftToRight);
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_2, 1, 1, 1, 1);

        label_1 = new QLabel(groupBox_ColorMode);
        label_1->setObjectName(QStringLiteral("label_1"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(label_1->sizePolicy().hasHeightForWidth());
        label_1->setSizePolicy(sizePolicy2);
        QFont font2;
        font2.setPointSize(20);
        label_1->setFont(font2);
        label_1->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_1, 0, 1, 1, 1);

        pushButton_2 = new QPushButton(groupBox_ColorMode);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy3);
        QFont font3;
        font3.setPointSize(35);
        pushButton_2->setFont(font3);

        gridLayout_2->addWidget(pushButton_2, 0, 2, 2, 1);

        pushButton_1 = new QPushButton(groupBox_ColorMode);
        pushButton_1->setObjectName(QStringLiteral("pushButton_1"));
        sizePolicy3.setHeightForWidth(pushButton_1->sizePolicy().hasHeightForWidth());
        pushButton_1->setSizePolicy(sizePolicy3);
        pushButton_1->setFont(font3);

        gridLayout_2->addWidget(pushButton_1, 0, 0, 2, 1);


        gridLayout->addWidget(groupBox_ColorMode, 1, 0, 1, 1);

        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", nullptr));
        groupBox_ColorMode->setTitle(QString());
        label_2->setText(QApplication::translate("PCLViewer", "<html><head/><body><p><span style=\" font-weight:600;\">0.00 kg</span></p></body></html>", nullptr));
        label_1->setText(QApplication::translate("PCLViewer", "Cubagem", nullptr));
        pushButton_2->setText(QApplication::translate("PCLViewer", "Palete", nullptr));
        pushButton_1->setText(QApplication::translate("PCLViewer", "Caixa", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
