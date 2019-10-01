#include "particlefiltermaingui.h"
#include "ui_particlefiltermaingui.h"

particleFilterMainGUI::particleFilterMainGUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::particleFilterMainGUI)
{
    ui->setupUi(this);
}

particleFilterMainGUI::~particleFilterMainGUI()
{
    delete ui;
}
