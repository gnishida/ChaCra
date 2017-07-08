#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QDomDocument>
#include <QResizeEvent>
#include <QtWidgets/QApplication>
#include <glm/gtx/string_cast.hpp>
#include <random>

static std::default_random_engine generator;

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	origin = QPoint(0, height());
	scale = 10.0;
	animation_timer = NULL;
	simulation_speed = 0.01;
	
	/*
	// poses
	poses.resize(2, std::vector<glm::dmat3x3>(2));
	poses[0][0] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	poses[0][1] = { 1, 0, 0, 0, 1, 0, 2, -0.5, 1 };
	poses[1][0] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	poses[1][1] = { 0, 1, 0, -1, 0, 0, 0, 1.5, 1 };

	// body geometry
	body_pts.resize(poses[0].size());
	body_pts[0].push_back(glm::dvec2(-1, -0.5));
	body_pts[0].push_back(glm::dvec2(-0.5, -0.5));
	body_pts[0].push_back(glm::dvec2(0, -0.5));
	body_pts[0].push_back(glm::dvec2(0.5, -0.5));
	body_pts[0].push_back(glm::dvec2(1, -0.5));
	body_pts[0].push_back(glm::dvec2(1, 0));
	body_pts[0].push_back(glm::dvec2(1, 0.5));
	body_pts[0].push_back(glm::dvec2(0.5, 0.5));
	body_pts[0].push_back(glm::dvec2(0, 0.5));
	body_pts[0].push_back(glm::dvec2(-0.5, 0.5));
	body_pts[0].push_back(glm::dvec2(-1, 0.5));
	body_pts[0].push_back(glm::dvec2(-1, 0));

	body_pts[1].push_back(glm::dvec2(-0.5, -1));
	body_pts[1].push_back(glm::dvec2(0, -1));
	body_pts[1].push_back(glm::dvec2(0.5, -1));
	body_pts[1].push_back(glm::dvec2(0.5, -0.5));
	body_pts[1].push_back(glm::dvec2(0.5, 0));
	body_pts[1].push_back(glm::dvec2(0.5, 0.5));
	body_pts[1].push_back(glm::dvec2(0.5, 1));
	body_pts[1].push_back(glm::dvec2(0, 1));
	body_pts[1].push_back(glm::dvec2(-0.5, 1));
	body_pts[1].push_back(glm::dvec2(-0.5, 0.5));
	body_pts[1].push_back(glm::dvec2(-0.5, 0));
	body_pts[1].push_back(glm::dvec2(-0.5, -0.5));

	// body geometry in the world coordinate system
	std::vector<std::vector<std::vector<glm::dvec2>>> body_pts_world(poses.size(), std::vector<std::vector<glm::dvec2>>(body_pts.size()));
	for (int i = 0; i < poses.size(); i++) {
		for (int j = 0; j < body_pts.size(); j++) {
			for (int k = 0; k < body_pts[j].size(); k++) {
				body_pts_world[i][j].push_back(glm::dvec2(poses[i][j] * glm::dvec3(body_pts[j][k], 1)));
			}
		}
	}

	auto connectors = findConnectors(poses, body_pts);
	auto trimmer = findTrimmer(poses, body_pts, std::get<0>(connectors), std::get<1>(connectors), std::get<2>(connectors), std::get<3>(connectors));
	
	// initialize kinematics structure
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, glm::dvec2(poses[0][0] * glm::dvec3(std::get<0>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, false, glm::dvec2(poses[0][1] * glm::dvec3(std::get<1>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, true, glm::dvec2(poses[0][0] * glm::dvec3(std::get<2>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, glm::dvec2(poses[0][1] * glm::dvec3(std::get<3>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(4, true, glm::dvec2(poses[0][0] * glm::dvec3(std::get<0>(trimmer), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(5, false, glm::dvec2(poses[0][0] * glm::dvec3(std::get<1>(trimmer), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(6, false, glm::dvec2(poses[0][0] * glm::dvec3(std::get<2>(trimmer), 1)))));
	kinematics.diagram.addLink(false, kinematics.diagram.joints[0], kinematics.diagram.joints[1]);
	kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);
	kinematics.diagram.addLink(false, { kinematics.diagram.joints[1], kinematics.diagram.joints[3], kinematics.diagram.joints[6] });
	kinematics.diagram.addLink(true, kinematics.diagram.joints[4], kinematics.diagram.joints[5]);
	kinematics.diagram.addLink(false, kinematics.diagram.joints[5], kinematics.diagram.joints[6]);

	// update the geometry
	kinematics.diagram.addBody(kinematics.diagram.joints[0], kinematics.diagram.joints[2], body_pts_world[0][0]);
	kinematics.diagram.addBody(kinematics.diagram.joints[1], kinematics.diagram.joints[3], body_pts_world[0][1]);

	// setup the kinematic system
	kinematics.diagram.initialize();
	*/
}

Canvas::~Canvas() {
}

void Canvas::open(const QString& filename) {
	QFile file(filename);
	if(!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

	QDomDocument doc;
	doc.setContent(&file);
	QDomElement root = doc.documentElement();
	if (root.tagName() != "design")	throw "Invalid file format.";

	kinematics.diagram.clear();
	poses.clear();
	int pose_count = 0;

	QDomNode node = root.firstChild();
	while (!node.isNull()) {
		if (node.toElement().tagName() == "pose") {
			QDomNode body_node = node.firstChild();
			std::vector<std::vector<glm::dvec2>> bodies;

			while (!body_node.isNull()) {
				if (body_node.toElement().tagName() == "body") {
					QDomNode point_node = body_node.firstChild();
					std::vector<glm::dvec2> pts;

					while (!point_node.isNull()) {
						if (point_node.toElement().tagName() == "point") {
							double x = point_node.toElement().attribute("x").toDouble();
							double y = point_node.toElement().attribute("y").toDouble();
							pts.push_back(glm::dvec2(x, y));
						}

						point_node = point_node.nextSibling();
					}

					bodies.push_back(pts);
				}

				body_node = body_node.nextSibling();
			}

			if (pose_count == 0) {
				body_pts = bodies;
				std::vector<glm::dmat3x3> Ts;
				for (int i = 0; i < bodies.size(); i++) {
					Ts.push_back(glm::dmat3x3());
				}
				poses.push_back(Ts);
			}
			else {
				std::vector<glm::dmat3x3> Ts;
				for (int i = 0; i < bodies.size(); i++) {
					glm::dmat3x3 T = kinematics::affineTransform(body_pts[i][0], bodies[i][0], body_pts[i][1], bodies[i][1]);
					Ts.push_back(T);
				}
				poses.push_back(Ts);
			}
			pose_count++;
		}

		node = node.nextSibling();
	}

	// body geometry of the 1st pose in the world coordinate system
	std::vector<std::vector<glm::dvec2>> body_pts_world(body_pts.size());
	for (int i = 0; i < body_pts.size(); i++) {
		for (int j = 0; j < body_pts[i].size(); j++) {
			body_pts_world[i].push_back(glm::dvec2(poses[0][i] * glm::dvec3(body_pts[i][j], 1)));
		}
	}

	// uniformly sample the points along the border of the bodies
	std::vector<std::vector<glm::dvec2>> sampled_body_pts = sampleBodyPoints(body_pts, 20);

	auto connectors = findConnectors(poses, sampled_body_pts);
	auto trimmer = findTrimmer(poses, sampled_body_pts, std::get<0>(connectors), std::get<1>(connectors), std::get<2>(connectors), std::get<3>(connectors));

	// initialize kinematics structure
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, glm::dvec2(poses[0][0] * glm::dvec3(std::get<0>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, false, glm::dvec2(poses[0][1] * glm::dvec3(std::get<1>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, true, glm::dvec2(poses[0][0] * glm::dvec3(std::get<2>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, glm::dvec2(poses[0][1] * glm::dvec3(std::get<3>(connectors), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(4, true, glm::dvec2(poses[0][0] * glm::dvec3(std::get<0>(trimmer), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(5, false, glm::dvec2(poses[0][0] * glm::dvec3(std::get<1>(trimmer), 1)))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(6, false, glm::dvec2(poses[0][0] * glm::dvec3(std::get<2>(trimmer), 1)))));
	kinematics.diagram.addLink(false, kinematics.diagram.joints[0], kinematics.diagram.joints[1]);
	kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);
	kinematics.diagram.addLink(false, { kinematics.diagram.joints[1], kinematics.diagram.joints[3], kinematics.diagram.joints[6] });
	kinematics.diagram.addLink(true, kinematics.diagram.joints[4], kinematics.diagram.joints[5]);
	kinematics.diagram.addLink(false, kinematics.diagram.joints[5], kinematics.diagram.joints[6]);

	// update the geometry
	kinematics.diagram.bodies.clear();
	kinematics.diagram.addBody(kinematics.diagram.joints[0], kinematics.diagram.joints[2], body_pts_world[0]);
	kinematics.diagram.addBody(kinematics.diagram.joints[1], kinematics.diagram.joints[3], body_pts_world[1]);

	// setup the kinematic system
	kinematics.diagram.initialize();

	update();
}

void Canvas::run() {
	if (animation_timer == NULL) {
		animation_timer = new QTimer(this);
		connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
		animation_timer->start(10);
	}
}

void Canvas::stop() {
	if (animation_timer != NULL) {
		animation_timer->stop();
		delete animation_timer;
		animation_timer = NULL;
	}
}

void Canvas::speedUp() {
	simulation_speed *= 2;
}

void Canvas::speedDown() {
	simulation_speed *= 0.5;
}

void Canvas::invertSpeed() {
	simulation_speed = -simulation_speed;
}

void Canvas::stepForward() {
	if (animation_timer == NULL) {
		try {
			kinematics.stepForward(simulation_speed);
		}
		catch (char* ex) {
			simulation_speed = -simulation_speed;
			std::cerr << "Animation is stopped by error:" << std::endl;
			std::cerr << ex << std::endl;
		}
		update();
	}
}

void Canvas::stepBackward() {
	if (animation_timer == NULL) {
		try {
			kinematics.stepForward(-simulation_speed);
		}
		catch (char* ex) {
			simulation_speed = -simulation_speed;
			std::cerr << "Animation is stopped by error:" << std::endl;
			std::cerr << ex << std::endl;
		}
		update();
	}
}

void Canvas::showAssemblies(bool flag) {
	kinematics.showAssemblies(flag);
	update();
}

void Canvas::showLinks(bool flag) {
	kinematics.showLinks(flag);
	update();
}

void Canvas::showBodies(bool flag) {
	kinematics.showBodies(flag);
	update();
}

/**
 * Sample N points along the boder of the bodies.
 */
std::vector<std::vector<glm::dvec2>> Canvas::sampleBodyPoints(const std::vector<std::vector<glm::dvec2>>& body_pts, int N) {
	std::vector<std::vector<glm::dvec2>> ret(body_pts.size());

	for (int i = 0; i < body_pts.size(); i++) {
		double border_length = 0.0;
		for (int j = 0; j < body_pts[i].size(); j++) {
			int next = (j + 1) % body_pts[i].size();
			border_length += glm::length(body_pts[i][j] - body_pts[i][next]);
		}
		
		double step_length = border_length / N;
		double remaining_length = step_length;
		for (int j = 0; j < body_pts[i].size(); j++) {
			int next = (j + 1) % body_pts[i].size();

			glm::dvec2 v = body_pts[i][next] - body_pts[i][j];
			double length = glm::length(v);
			v /= length;

			glm::dvec2 p = body_pts[i][j];
			while (length > remaining_length) {
				p += v * remaining_length;
				ret[i].push_back(p);
				length -= remaining_length;
				remaining_length = step_length;
			}

			remaining_length -= length;
		}

		if (remaining_length < step_length * 0.3) {
			ret[i].push_back(body_pts[i][0]);
		}
	}

	return ret;
}

/**
 * Return the coordinates of the end points of two connectors, 
 * the end point of 1st connector on the 1st body, the end point of 1st connector on the 2nd body,
 * the end point of 2nd connector on the 1st body, and the end point of 2nd connector on the 2nd body.
 * Note: The coordinates are based on the local coordinate system.
 */
std::tuple<glm::dvec2, glm::dvec2, glm::dvec2, glm::dvec2> Canvas::findConnectors(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<std::vector<glm::dvec2>>& body_pts) {
	// randomly select two points on C_1 and C_2, calculate the distance variation, and store the pair into the map
	std::map<double, std::pair<int, int>> table;
	std::map<std::pair<int, int>, bool> duplication_check;
	int cnt = 0;
	while (cnt < 50) {
		int id1 = rand() % body_pts[0].size();
		int id2 = rand() % body_pts[1].size();
		if (duplication_check.find(std::make_pair(id1, id2)) != duplication_check.end()) continue;

		duplication_check[std::make_pair(id1, id2)] = true;
		cnt++;

		glm::dvec2 p1a = glm::dvec2(poses[0][0] * glm::dvec3(body_pts[0][id1], 1));
		glm::dvec2 p1b = glm::dvec2(poses[1][0] * glm::dvec3(body_pts[0][id1], 1));
		glm::dvec2 p2a = glm::dvec2(poses[0][1] * glm::dvec3(body_pts[1][id2], 1));
		glm::dvec2 p2b = glm::dvec2(poses[1][1] * glm::dvec3(body_pts[1][id2], 1));

		double da = glm::length(p1a - p2a);
		double db = glm::length(p1b - p2b);
		double var = abs(da - db);
		while (table.find(var) != table.end()) {
			var += 0.000001;
		}
		table[var] = std::make_pair(id1, id2);
	}

	auto it = table.begin();
	int id1 = it->second.first;
	int id2 = it->second.second;
	glm::dvec2 p1 = glm::dvec2(poses[0][0] * glm::dvec3(body_pts[0][id1], 1));
	glm::dvec2 p2 = glm::dvec2(poses[0][1] * glm::dvec3(body_pts[1][id2], 1));
	it++;

	int id3;
	int id4;
	glm::dvec2 p3;
	glm::dvec2 p4;
	for (; it != table.end(); it++) {
		id3 = it->second.first;
		id4 = it->second.second;
		if (id3 == id1) continue;

		p3 = glm::dvec2(poses[0][0] * glm::dvec3(body_pts[0][id3], 1));
		p4 = glm::dvec2(poses[0][1] * glm::dvec3(body_pts[1][id4], 1));

		if (glm::dot((p2 - p1) / glm::length(p2 - p1), (p4 - p3) / glm::length(p4 - p3)) > 0.98) continue;

		// DEBUG
		std::cout << "Connectors are found." << std::endl;
		std::cout << "The distance variance is at most " << it->first << "." << std::endl;

		break;
	}

	return std::make_tuple(body_pts[0][id1], body_pts[1][id2], body_pts[0][id3], body_pts[1][id4]);
}

/**
 * Calculate the trimmer links, and return the coordinates of p_1, q_12, and r2, in the local system of C_1.
 *
 * @param poses
 * @param body_pts
 * @param a1		the end point on C_1 of the first connector in the local system of C_1
 * @param a2		the end point on C_2 of the first connector in the local system of C_2
 * @param b1		the end point on C_1 of the second connector in the local system of C_1
 * @param b2		the end point on C_2 of the second connector in the local system of C_2
 */
std::tuple<glm::dvec2, glm::dvec2, glm::dvec2> Canvas::findTrimmer(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<std::vector<glm::dvec2>>& body_pts, const glm::dvec2& a1, const glm::dvec2& a2, const glm::dvec2& b1, const glm::dvec2& b2) {
	// calculate the bounding box of C_1
	glm::dvec2 min_pt(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	glm::dvec2 max_pt(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
	for (int i = 0; i < body_pts[0].size(); i++) {
		min_pt.x = std::min(min_pt.x, body_pts[0][i].x);
		max_pt.x = std::max(max_pt.x, body_pts[0][i].x);
		min_pt.y = std::min(min_pt.y, body_pts[0][i].y);
		max_pt.y = std::max(max_pt.y, body_pts[0][i].y);
	}

	std::map<double, std::pair<glm::dvec2, glm::dvec2>, std::greater<double>> table;
	int cnt = 0;
	while (cnt < 100) {
		// radomly sample a point p_1 in C_1
		glm::dvec2 p1(genNormal((min_pt.x + max_pt.x) * 0.5, (max_pt.x - min_pt.x) * 0.5), genNormal((min_pt.y + max_pt.y) * 0.5, (max_pt.y - min_pt.y) * 0.5));
		if (!kinematics::withinPolygon(body_pts[0], p1)) continue;

		// if p_1 is too close to the end point of the first connect, reject it.
		if (glm::length(p1 - a1) < 0.1) continue;

		cnt++;
		glm::dvec2 p1a(poses[0][0] * glm::dvec3(p1, 1));
		glm::dvec2 p1b(poses[1][0] * glm::dvec3(p1, 1));
		
		// randomly sample a point r_2 on the border of C_2
		int id2 = rand() % body_pts[1].size();
		glm::dvec2 r2a(poses[0][1] * glm::dvec3(body_pts[1][id2], 1));
		glm::dvec2 r2b(poses[1][1] * glm::dvec3(body_pts[1][id2], 1));
		/*
		glm::dvec2 r2a(poses[0][1] * glm::dvec3(a2, 1));
		glm::dvec2 r2b(poses[1][1] * glm::dvec3(a2, 1));
		*/

		double da = glm::length(p1a - r2a);
		double db = glm::length(p1b - r2b);
		double var = abs(da - db);
		while (table.find(var) != table.end()) {
			var += 0.000001;
		}
		table[var] = std::make_pair(p1, body_pts[1][id2]);
		//table[var] = std::make_pair(p1, a2);
	}

	// calculate the angle of the first connector
	glm::dvec2 a2a(glm::inverse(poses[0][0]) * poses[0][1] * glm::dvec3(a2, 1));
	glm::dvec2 a2b(glm::inverse(poses[1][0]) * poses[1][1] * glm::dvec3(a2, 1));
	double theta_a = atan2(a2a.y - a1.y, a2a.x - a1.x);
	double theta_b = atan2(a2b.y - a1.y, a2b.x - a1.x);
	double l1 = glm::length(a2a - a1);

	glm::dvec2 b2a(glm::inverse(poses[0][0]) * poses[0][1] * glm::dvec3(b2, 1));
	glm::dvec2 b2b(glm::inverse(poses[1][0]) * poses[1][1] * glm::dvec3(b2, 1));
	double l2 = glm::length(b2a - b1);
	double l3 = glm::length(a2a - b2a);

	for (auto it = table.begin(); it != table.end(); it++) {
		// the coordinate of p1 in a local system of C_1
		glm::dvec2 p1 = it->second.first;

		// the coordinate of r2 in a local system of C_2
		glm::dvec2 r2 = it->second.second;
		
		// convert the coordinate of r2 in a local system of C_2 to that in a local system of C_1
		glm::dvec2 r2_C1 = glm::dvec2(glm::inverse(poses[0][0]) * poses[0][1] * glm::dvec3(r2, 1));

		// calculate the distance between p1 and r2 for each theta
		glm::dvec2 prev_b2 = b2a;
		std::vector<double> dists;
		for (int i = 0; i <= 20; i++) {
			double theta = theta_a + (theta_b - theta_a) / 20.0 * i;

			glm::dvec2 current_a2(a1.x + l1 * cos(theta), a1.y + l1 * sin(theta));
			glm::dvec2 current_b2 = kinematics::circleCircleIntersection(current_a2, l3, b1, l2, prev_b2);

			// calculate the transformation matrix of C_2 in the local system of C_1
			glm::dmat3x3 T = glm::inverse(poses[0][0]) * kinematics::affineTransform(a2a, current_a2, b2a, current_b2) * poses[0][1];

			// calculate the coordinates of r2 in the local system of C_1
			glm::dvec2 current_r2 = glm::dvec2(T * glm::dvec3(r2, 1));

			// calculate the distance between p1 and r2
			double dist = glm::length(current_r2 - p1);
			dists.push_back(dist);

			prev_b2 = current_b2;
		}

		// check the monotonicity
		bool monotonic = true;
		if (dists.front() < dists.back()) {
			for (int i = 0; i < dists.size() - 1; i++) {
				if (dists[i + 1] < dists[i]) {
					monotonic = false;
					break;
				}
			}
		}
		else {
			for (int i = 0; i < dists.size() - 1; i++) {
				if (dists[i + 1] > dists[i]) {
					monotonic = false;
					break;
				}
			}
		}

		if (monotonic) {
			// calculate the lengths of K_12 and L_12
			double L12 = (dists.front() + dists.back()) / 2.0;
			double K12 = abs(dists.front() - dists.back()) / 2.0;

			// calculate the coordinates of q_12
			glm::dvec2 q12;
			if (dists.front() > dists.back()) {
				q12 = p1 + (r2_C1 - p1) / (L12 + K12) * K12;
			}
			else {
				q12 = p1 + (p1 - r2_C1) / (L12 - K12) * K12;
			}

			return std::make_tuple(p1, q12, r2_C1);
		}
	}

	throw "No trimmer was found.";
}

double Canvas::genRand() {
	return genRand(0, 1);
}

double Canvas::genRand(double a, double b) {
	std::uniform_real_distribution<double> distribution(a, b);
	return distribution(generator);
}

double Canvas::genNormal() {	
	return genNormal(0, 1);
}

double Canvas::genNormal(double m, double s) {
	std::normal_distribution<double> distribution(m, s);
	return distribution(generator);
}


void Canvas::animation_update() {
	try {
		kinematics.stepForward(simulation_speed);
	}
	catch (char* ex) {
		simulation_speed = -simulation_speed;
		//stop();
		std::cerr << "Animation is stopped by error:" << std::endl;
		std::cerr << ex << std::endl;
	}

	update();

}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	// draw axes
	painter.save();
	painter.setPen(QPen(QColor(128, 128, 128), 1, Qt::DashLine));
	painter.drawLine(-10000 * scale + origin.x(), origin.y(), 10000 * scale + origin.y(), origin.y());
	painter.drawLine(origin.x(), 10000 * scale + origin.y(), origin.x(), -10000 * scale + origin.y());
	painter.restore();

	// draw bodies
	painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
	painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
	for (int i = 0; i < poses.size(); i++) {
		for (int j = 0; j < body_pts.size(); j++) {
			QPolygonF pts;
			for (int k = 0; k < body_pts[j].size(); k++) {
				glm::dvec2 p = glm::dvec2(poses[i][j] * glm::dvec3(body_pts[j][k], 1));
				pts.push_back(QPointF(origin.x() + p.x * scale, origin.y() - p.y * scale));
			}
			painter.drawPolygon(pts);
		}
	}

	kinematics.draw(painter, origin, scale);
}

void Canvas::mousePressEvent(QMouseEvent* e) {
	if (e->buttons() & Qt::LeftButton) {
		// convert the mouse position to the world coordinate system
		glm::dvec2 pt((e->x() - origin.x()) / scale, -(e->y() - origin.y()) / scale);

		// select a joint to move
		selectedJoint.reset();
		double min_dist = 6;
		for (int i = 0; i < kinematics.diagram.joints.size(); i++) {
			double dist = glm::length(kinematics.diagram.joints[i]->pos - pt);
			if (dist < min_dist) {
				min_dist = dist;
				selectedJoint = kinematics.diagram.joints[i];
			}
		}
	}

	prev_mouse_pt = e->pos();
}

void Canvas::mouseMoveEvent(QMouseEvent* e) {
	if (e->buttons() & Qt::LeftButton && selectedJoint) {
		// move the selected joint
		selectedJoint->pos.x = (e->x() - origin.x()) / scale;
		selectedJoint->pos.y = -(e->y() - origin.y()) / scale;

		// update the geometry
		//kinematics.diagram.bodies.clear();
		//kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts[0]);

		// setup the kinematic system
		kinematics.diagram.initialize();
		update();
	}
	else {
		// move the camera
		if (e->buttons() & Qt::RightButton) {
			// translate the Origin
			origin += e->pos() - prev_mouse_pt;
			update();
		}
	}

	prev_mouse_pt = e->pos();
}

void Canvas::mouseReleaseEvent(QMouseEvent* e) {
}

void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
}

void Canvas::wheelEvent(QWheelEvent* e) {
	scale += e->delta() * 0.01;
	scale = std::min(std::max(0.1, scale), 1000.0);
	update();
}

void Canvas::resizeEvent(QResizeEvent *e) {
}

void Canvas::keyPressEvent(QKeyEvent* e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Escape:
		break;
	case Qt::Key_Space:
		break;
	case Qt::Key_Delete:
		break;
	}

	update();
}

void Canvas::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

