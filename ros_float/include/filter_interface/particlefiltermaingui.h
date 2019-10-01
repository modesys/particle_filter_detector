#ifndef PARTICLEFILTERMAINGUI_H
#define PARTICLEFILTERMAINGUI_H

#include <QWidget>

namespace Ui {
class particleFilterMainGUI;
}

class particleFilterMainGUI : public QWidget
{
    Q_OBJECT

public:
    explicit particleFilterMainGUI(QWidget *parent = nullptr);
    ~particleFilterMainGUI();

private:
    Ui::particleFilterMainGUI *ui;
};

#endif // PARTICLEFILTERMAINGUI_H
