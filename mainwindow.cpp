#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <gdal_priv.h>
#include <cstdint>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GDALAllRegister();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_action_Open_triggered()
{
    QString fname = QFileDialog::getOpenFileName(this,tr("Open"),"/home/roland/data/BSB_ROOT/");

    GDALDataset * dataset = reinterpret_cast<GDALDataset*>(GDALOpen(fname.toStdString().c_str(),GA_ReadOnly));
    if (dataset)
    {
        int width = dataset->GetRasterXSize();
        int height = dataset->GetRasterYSize();


        GDALRasterBand * band = dataset->GetRasterBand(1);

        GDALColorTable *colorTable = band->GetColorTable();

        std::vector<uint32_t> buffer(width);

        QImage image(width,height,QImage::Format_ARGB32);

        for(int j = 0; j<height; ++j)
        {
            band->RasterIO(GF_Read,0,j,width,1,&buffer.front(),width,1,GDT_UInt32,0,0);
            uchar *scanline = image.scanLine(j);
            for(int i = 0; i < width; ++i)
            {
                GDALColorEntry const *ce = colorTable->GetColorEntry(buffer[i]);
                scanline[i*4] = ce->c3;
                scanline[i*4+1] = ce->c2;
                scanline[i*4+2] = ce->c1;
                scanline[i*4+3] = ce->c4;
            }
        }
        ui->missionCanvas->setBackgroundImage(image);
    }
}
