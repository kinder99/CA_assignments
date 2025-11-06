#include "widgetsph.h"
#include "ui_widgetsph.h"

WidgetSPH::WidgetSPH(QWidget *parent) : QWidget(parent), ui(new Ui::WidgetSPH)
{
    ui->setupUi(this);

    connect(ui->btnUpdate, &QPushButton::clicked, this,
            [=] (void) { emit updatedParameters(); });
}

WidgetSPH::~WidgetSPH()
{
    delete ui;
}

double WidgetSPH::getGravity() const {
    return ui->gravity->value();
}

double WidgetSPH::getSpeed() const {
    return ui->speed->value();
}

double WidgetSPH::getDensity() const {
    return ui->density->value();
}

double WidgetSPH::getWidth() const {
    return ui->width->value();
}

double WidgetSPH::getHeight() const {
    return ui->height->value();
}

double WidgetSPH::getDepth() const {
    return ui->depth->value();
}

double WidgetSPH::getSizeX() const {
    return ui->sizeX->value();
}

double WidgetSPH::getSizeY() const {
    return ui->sizeY->value();
}

double WidgetSPH::getSizeZ() const {
    return ui->sizeZ->value();
}

