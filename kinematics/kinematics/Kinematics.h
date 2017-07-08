#pragma once

#include <QPainter>
#include <QPoint>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <boost/shared_ptr.hpp>
#include <QMap>
#include "Joint.h"
#include "Link.h"
#include "BodyGeometry.h"
#include "KinematicDiagram.h"

namespace kinematics {
	
	class Kinematics {
	public:
		KinematicDiagram diagram;
		std::vector<std::vector<glm::vec2>> trace_end_effector;

		bool show_assemblies;
		bool show_links;
		bool show_bodies;

	public:
		Kinematics();

		void load(const QString& filename);
		void save(const QString& filename);
		void forwardKinematics();
		void stepForward(double time_step);
		bool isCollided();
		void draw(QPainter& painter, const QPoint& origin, float scale) const ;
		void showAssemblies(bool flag);
		void showLinks(bool flag);
		void showBodies(bool flag);
	};

}
