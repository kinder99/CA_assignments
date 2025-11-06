#ifndef WIDGETFOUNTAIN_H
#define WIDGETFOUNTAIN_H

#include <QWidget>

namespace Ui {
class WidgetFountain;
}

class WidgetFountain : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetFountain(QWidget *parent = nullptr);
    ~WidgetFountain();

    double getGravity()    const;
    double getLifetime() const;
    double getEmitRate() const;
    bool getCollisions() const;

signals:
    void updatedParameters();

private:
    Ui::WidgetFountain *ui;
};

#endif // WIDGETFOUNTAIN_H
