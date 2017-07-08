#include "MainWindow.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));


	setCentralWidget(&canvas);
}

MainWindow::~MainWindow() {
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Design file..."), "", tr("Design Files (*.xml)"));
	if (filename.isEmpty()) return;

	this->setWindowTitle("ChaCra - " + filename);

	try {
		canvas.open(filename);
	}
	catch (char* ex) {
		QMessageBox::warning(this, "Error message", ex);
	}
}

void MainWindow::onRun() {
	canvas.run();
}

void MainWindow::onStop() {
	canvas.stop();
}

void MainWindow::onStepForward() {
	canvas.stepForward();
}

void MainWindow::onStepBackward() {
	canvas.stepBackward();
}