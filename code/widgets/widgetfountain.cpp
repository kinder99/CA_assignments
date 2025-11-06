#include "widgetfountain.h"
#include "ui_widgetfountain.h"

WidgetFountain::WidgetFountain(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetFountain)
{
    ui->setupUi(this);

    connect(ui->btnUpdate, &QPushButton::clicked, this,
            [=] (void) { emit updatedParameters(); });
}

WidgetFountain::~WidgetFountain()
{
    delete ui;
}

double WidgetFountain::getGravity() const {
    return ui->gravity->value();
}

double WidgetFountain::getLifetime() const {
    return ui->lifetime->value();
}

double WidgetFountain::getEmitRate() const {
    return ui->rate->value();
}

bool WidgetFountain::getCollisions() const {
    return ui->p_p_col->isChecked();
}
