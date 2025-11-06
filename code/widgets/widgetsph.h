#ifndef WIDGETSPH_H
#define WIDGETSPH_H

#include <QWidget>

namespace Ui {
class WidgetSPH;
}

class WidgetSPH : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetSPH(QWidget *parent = nullptr);
    ~WidgetSPH();

    double getGravity() const;
    double getSpeed() const;
    double getDensity() const;

    double getWidth() const;
    double getHeight() const;
    double getDepth() const;
    double getSizeX() const;
    double getSizeY() const;
    double getSizeZ() const;

signals:
    void updatedParameters();

private:
    Ui::WidgetSPH *ui;
};

#endif // WIDGETSPH_H
