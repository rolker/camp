#ifndef AUTONOMOUSVEHICLEPROJECT_H
#define AUTONOMOUSVEHICLEPROJECT_H

#include <QObject>

class QStandardItemModel;
class QStandardItem;

class AutonomousVehicleProject : public QObject
{
    Q_OBJECT
public:
    explicit AutonomousVehicleProject(QObject *parent = 0);
    ~AutonomousVehicleProject();

    QStandardItemModel *getModel() const;
    void openBackground(QString const &fname);
signals:

public slots:

private:
    QStandardItemModel* model;
    std::map<QString,QStandardItem*> topLevelItems;
};

#endif // AUTONOMOUSVEHICLEPROJECT_H
