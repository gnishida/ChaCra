#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <kinematics.h>
#include <QTimer>

class Canvas : public QWidget {
Q_OBJECT

public:
	bool ctrlPressed;
	bool shiftPressed;

	kinematics::Kinematics kinematics;
	QTimer* animation_timer;
	float simulation_speed;
	QPoint prev_mouse_pt;
	QPoint origin;
	double scale;
	boost::shared_ptr<kinematics::Joint> selectedJoint;
	std::vector<std::vector<glm::dvec2>> body_pts;
	std::vector<std::vector<glm::dmat3x3>> poses;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void open(const QString& filename);
	void run();
	void stop();
	void speedUp();
	void speedDown();
	void invertSpeed();
	void stepForward();
	void stepBackward();
	void showAssemblies(bool flag);
	void showLinks(bool flag);
	void showBodies(bool flag);

	std::vector<std::vector<glm::dvec2>> sampleBodyPoints(const std::vector<std::vector<glm::dvec2>>& body_pts, int N);
	std::tuple<glm::dvec2, glm::dvec2, glm::dvec2, glm::dvec2> findConnectors(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<std::vector<glm::dvec2>>& body_pts);
	std::tuple<glm::dvec2, glm::dvec2, glm::dvec2> findTrimmer(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<std::vector<glm::dvec2>>& body_pts, const glm::dvec2& a1, const glm::dvec2& a2, const glm::dvec2& b1, const glm::dvec2& b2);
	double genRand();
	double genRand(double a, double b);
	double genNormal();
	double genNormal(double m, double s);

public slots:
	void animation_update();

protected:
	void paintEvent(QPaintEvent* e);
	void mousePressEvent(QMouseEvent* e);
	void mouseMoveEvent(QMouseEvent* e);
	void mouseReleaseEvent(QMouseEvent* e);
	void mouseDoubleClickEvent(QMouseEvent* e);
	void wheelEvent(QWheelEvent* e);
	void resizeEvent(QResizeEvent *e);

public:
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
};

#endif // CANVAS_H
