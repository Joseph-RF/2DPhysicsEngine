#include "Engine.h"

int windowWidth = 1200;
int windowHeight = 800;
float f_windowWidth = 1200.f;
float f_windowHeight = 800.f;

int cell_size = 50;
int cell_number_x = windowWidth / cell_size;
int cell_number_y = windowHeight / cell_size;

extern sf::Vector2f lowerBarrier_position = { 0.f , f_windowHeight * 0.95f };
extern sf::Vector2f upperBarrier_position = { 0.f, 0.f };
extern sf::Vector2f rightBarrier_position = { f_windowWidth - 20.f ,0.f };
extern sf::Vector2f leftBarrier_position = { 0.f, 0.f };

Engine::Engine()
{
	initVariables();
	initWindow();
	initFont();
	initText();
	initScene();
}

Engine::~Engine()
{
	for (size_t i = 0; i < Entities.size(); i++) {
		delete Entities[i];
	}

	delete window;
}

void Engine::initVariables()
{
	window = nullptr;
	timeStep = 0.0f;
	timeElapsed = 0.0f;
	subSteps = 8;
	entitiesSpawned = 0;

	grid = std::vector<std::vector<Entity*>>(cell_number_x * cell_number_y);
}

void Engine::initFont()
{
	if (!font.loadFromFile("Fonts/RobotoMono-Semibold.ttf")) {
		std::cout << "ERROR: Failed to load font." << std::endl;
	}
}

void Engine::initText()
{
	fpstext.setFont(font);
	fpstext.setCharacterSize(24);
	fpstext.setFillColor(sf::Color::Red);

	entitiesSpawnedText.setFont(font);
	entitiesSpawnedText.setCharacterSize(24);
	entitiesSpawnedText.setFillColor(sf::Color::Red);
	entitiesSpawnedText.setPosition(f_windowWidth * 0.75f, 0.0f);
}

void Engine::initScene()
{
	upperBarrier.setBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, 0));
	sf::Vector2f upperBarrier_position = { 0.f ,0.f };

	rightBarrier.setBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(f_windowWidth - 20, 0));
	sf::Vector2f rightBarrier_position = { f_windowWidth - 20.f ,0.f };

	lowerBarrier.setBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, f_windowHeight * 0.95f));
	sf::Vector2f lowerBarrier_position = { 0.f , f_windowHeight * 0.95f };

	leftBarrier.setBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(0, 0));
	sf::Vector2f leftBarrier_position = { 0.f , 0.f };
}

void Engine::initWindow()
{
	window = new sf::RenderWindow(sf::VideoMode(windowWidth, windowHeight), "Physics Engine");
	window->setFramerateLimit(0);
}

void Engine::addCircle()
{
	Entities.emplace_back(new Circle(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 10.f, sf::Color::Red));
	entitiesSpawned++;
}

void Engine::addSquare()
{
	Entities.emplace_back(new Square(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 20.f, sf::Color::Green));
	entitiesSpawned++;
}

void Engine::addSpring()
{
	addCircle();
	addCircle();

	Springs.emplace_back(Spring(Entities[Entities.size() - 2], Entities[Entities.size() - 1], 10.f, 0.2f, 300.f));
}

void Engine::addSponge()
{
	Entities.emplace_back(new Circle(sf::Vector2f(50.f, 50.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Circle(sf::Vector2f(120.f, 50.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Circle(sf::Vector2f(50.f, 120.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Circle(sf::Vector2f(120.f, 120.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Circle(sf::Vector2f(85.f, 85.f), 1.f, 10.f, sf::Color::Green));

	Springs.emplace_back(Spring(Entities[Entities.size() - 1], Entities[Entities.size() - 5], 500.f, 1.f, 49.5f));
	Springs.emplace_back(Spring(Entities[Entities.size() - 1], Entities[Entities.size() - 4], 500.f, 1.f, 49.5f));
	Springs.emplace_back(Spring(Entities[Entities.size() - 1], Entities[Entities.size() - 3], 500.f, 1.f, 49.5f));
	Springs.emplace_back(Spring(Entities[Entities.size() - 1], Entities[Entities.size() - 2], 500.f, 1.f, 49.5f));

	Springs.emplace_back(Spring(Entities[Entities.size() - 5], Entities[Entities.size() - 4], 500.f, 1.f, 70.f));
	Springs.emplace_back(Spring(Entities[Entities.size() - 4], Entities[Entities.size() - 2], 500.f, 1.f, 70.f));
	Springs.emplace_back(Spring(Entities[Entities.size() - 2], Entities[Entities.size() - 3], 500.f, 1.f, 70.f));
	Springs.emplace_back(Spring(Entities[Entities.size() - 3], Entities[Entities.size() - 5], 500.f, 1.f, 70.f));
}

void Engine::pollEvents()
{
	while (window->pollEvent(e)) {
		switch (e.type) {
		case sf::Event::Closed:
			window->close();
			break;
		case sf::Event::KeyPressed:
			if (e.key.code == sf::Keyboard::Escape) {
				window->close();
			} else if (e.key.code == sf::Keyboard::C) {
				addCircle();
			} else if (e.key.code == sf::Keyboard::S) {
				addSquare();
			} else if (e.key.code == sf::Keyboard::R) {
				addSpring();
			} else if (e.key.code == sf::Keyboard::T) {
				addSponge();
			}
			break;
		}
	}
}

void Engine::solver(Entity& E, float dt)
{
	//Verlet integration being used because this simulation involves spheres being dependent on each other's motion.
	//Verlet offers numerical stability as well as being fast and accurate enough for the purposes of this software.
	//Will use verlet WITHOUT velocities


	timeElapsed += timeStep;


	sf::Vector2f displacement = E.currentPosition - E.oldPosition;
	float angularDisplacement = E.currentAngle - E.oldAngle;

	E.oldPosition = E.currentPosition;
	E.oldAngle = E.currentAngle;

	applyGravity(E);
	E.currentAcceleration = E.force / E.mass;

	E.currentPosition += displacement + (E.currentAcceleration * dt * dt);
	E.currentAngle += angularDisplacement;

	E.force = { 0.f, 0.f };
}

inline void Engine::applyGravity(Entity& E)
{
	//Different forces will be added here
	E.force += {0.f * E.mass, (1000.f * E.mass)}; //Gravity
}

void Engine::detectEntityBarrierCollision()
{
	size_t entityNumber = Entities.size();
	for (size_t i = 0; i < entityNumber; ++i) {

		Entities[i]->entityBarrierCollision();
	}
}

void Engine::wideSweep()
{
	int number_of_cells = cell_number_x * cell_number_y;
	for (size_t i = 0; i < number_of_cells; ++i) {
		for (size_t j = 0; j < grid[i].size(); ++j) {
			detectEntityEntityCollision(i, grid[i][j]);
			detectEntityEntityCollision(i - 1, grid[i][j]);
			detectEntityEntityCollision(i + 1, grid[i][j]);

			detectEntityEntityCollision(i + cell_number_x, grid[i][j]);
			detectEntityEntityCollision(i + cell_number_x - 1, grid[i][j]);
			detectEntityEntityCollision(i + cell_number_x + 1, grid[i][j]);

			detectEntityEntityCollision(i - cell_number_x, grid[i][j]);
			detectEntityEntityCollision(i - cell_number_x - 1, grid[i][j]);
			detectEntityEntityCollision(i - cell_number_x + 1, grid[i][j]);
		}
	}
}

void Engine::detectEntityEntityCollision(int grid_cell, Entity* E)
{
	if ((grid_cell >= cell_number_x * cell_number_y) || grid_cell < 0) { return; }

	for (size_t k = 0; k < grid[grid_cell].size(); ++k) {

		if (E == grid[grid_cell][k]) {
			break;
		}

		E->detectEntityCollision(*grid[grid_cell][k]);
	}
}

void Engine::sortEntities()
{
	for (int i = 0; i < cell_number_x * cell_number_y; ++i) {
		grid[i].clear();
	}

	for (Entity* entity : Entities) {
		grid[getCellNumber(entity->currentPosition)].emplace_back(entity);
	}
}

int Engine::getCellNumber(sf::Vector2f pos)
{
	return ((std::floor(pos.x / cell_size)) + std::floor(pos.y / cell_size) * cell_number_x);
}

void Engine::circleCircleDetection(Circle& c1, Circle& c2)
{
	sf::Vector2f separation = c2.currentPosition - c1.currentPosition;
	float separation_magnitude = pow((separation.x * separation.x) + (separation.y * separation.y), 0.5f);

	if (separation_magnitude < c1.size + c2.size) {
		circleCircleResolution(c1, c2, separation_magnitude, (separation / separation_magnitude));
	}
}

void Engine::circleCircleResolution(Circle& c1, Circle& c2, float depth, sf::Vector2f axis)
{
	//Resolve collision between the circles
	const sf::Vector2f vel_c2 = c2.currentPosition - c2.oldPosition;
	const sf::Vector2f vel_c1 = c1.currentPosition - c1.oldPosition;

	const float norm_vel_relative = Engine::dotProduct(vel_c2 - vel_c1, axis);

	sf::Vector2f overLapCorrection = 0.5f * (c1.size + c2.size - depth) * axis;

	c1.currentPosition -= overLapCorrection;
	c2.currentPosition += overLapCorrection;

	float j;

	//Only take impulse into account if objects are moving towards each other.
	if (norm_vel_relative > 0.f) {
		j = 0.f;
	}
	else {
		j = -(1.f + (c1.resCoeff > c2.resCoeff ? c1.resCoeff : c2.resCoeff)) *
			norm_vel_relative / ((1.f / c1.mass) + (1.f / c2.mass));
	}

	c1.oldPosition += ((j / (1.f / c1.mass)) * axis - overLapCorrection);
	c2.oldPosition -= ((j / (1.f / c2.mass)) * axis - overLapCorrection);
}

void Engine::circlePolygonDetection(Circle& c, ConvexPolygon& polygon)
{
	sf::Vector2f closestPoint;
	int polygonVertexCount = polygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f tempVector = Engine::closestPointOnLineSegment(c.currentPosition, polygon.getVertexPosition(i),
																	polygon.getVertexPosition((i + 1) % polygonVertexCount));
		float tempDistance = Engine::findDistance(c.currentPosition, tempVector);

		if (tempDistance < c.size) {
			sf::Vector2f axis = tempVector - c.currentPosition;
			axis = Engine::normalise(axis);

			//Check the normal is facing in the correct direction
			if (dotProduct(polygon.currentPosition - c.currentPosition, axis) < 0.f) {
				axis = {-axis.x, -axis.y};
			}
			//std::cout << "Distance being passed: " << tempDistance << std::endl;
			Engine::circlePolygonResolution(c, polygon, (c.size - tempDistance), axis, tempVector - polygon.currentPosition);
		}
	}
}

/*
void Engine::circlePolygonDetection(Circle& c, ConvexPolygon& convexPolygon)
{	
	//SAT implementation to find out whether a polygon and a circle have collided
	//Draw an axis which is the normal of the polygon.
	//Project every vertex onto that axis and two points on the circle.
	//If there is any gap, the polygons are NOT touching

	//Define normal and depth. These will be the axis and depth associated with the smallest 
	//displacement possible in order to push the two objects aside.

	float minimumDepth = std::numeric_limits<float>::max();
	sf::Vector2f minimumAxis;

	int polygonVertexCount = convexPolygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {

		sf::Vector2f axis;
		float depth;

		sf::Vector2f vertex1Position = convexPolygon.getVertexPosition(i);
		sf::Vector2f vertex2Position = convexPolygon.getVertexPosition((i + 1) % polygonVertexCount);

		axis = sf::Vector2f(vertex2Position.y - vertex1Position.y, -(vertex2Position.x - vertex1Position.x));
		axis = Engine::normalise(axis);

		//Project all the vertices of both polygons onto the axis. Find the max and min for both polygons
		float maxCircle = std::numeric_limits<float>::min();
		float minCircle = std::numeric_limits<float>::max();

		float maxPolygon = maxCircle;
		float minPolygon = minCircle;

		sf::Vector2f circlePoint1 = c.currentPosition - (axis * c.size);
		sf::Vector2f circlePoint2 = c.currentPosition + (axis * c.size);

		maxCircle = Engine::dotProduct(circlePoint1, axis);
		minCircle = Engine::dotProduct(circlePoint2, axis);

		if (maxCircle < minCircle) {
			float temp = maxCircle;
			maxCircle = minCircle;
			minCircle = temp;
		}

		for (int j = 0; j < polygonVertexCount; ++j) {
			sf::Vector2f vertexPosition = convexPolygon.getVertexPosition(j);
			float projection = Engine::dotProduct(vertexPosition, axis);

			if (projection > maxPolygon) {
				maxPolygon = projection;
			}
			if (projection < minPolygon) {
				minPolygon = projection;
			}
		}

		if (maxCircle < minPolygon || maxPolygon < minCircle) {
			//Not colliding.
			return;
		}
		//Projections are overlapping
		if (maxCircle - minPolygon < maxPolygon - minCircle) {
			depth = maxCircle - minPolygon;
		}
		else {
			depth = maxPolygon - minCircle;
		}

		if (depth < minimumDepth) {
			minimumDepth = depth;
			minimumAxis = axis;
		}
	}

	//Find the axis point from the center of the circle to the closest point on the polygon.

	sf::Vector2f axis;
	float depth;

	
	//for (int i = 0; i < polygonVertexCount; ++i) {
		//float distance = Engine::findDistance(convexPolygon.getVertexPosition(i), c.currentPosition);
		//if (distance < shortestDistance) {
			//shortestDistance = distance;
			//axis = c.currentPosition - convexPolygon.getVertexPosition(i); //Should this be reversed?
		//}
	//}
	
	sf::Vector2f closestPoint = Engine::findContactPoint(c, convexPolygon);
	axis = c.currentPosition - closestPoint;

	axis = Engine::normalise(axis);

	//Project all the vertices of both polygons onto the axis. Find the max and min for both polygons
	float maxCircle = std::numeric_limits<float>::min();
	float minCircle = std::numeric_limits<float>::max();

	float maxPolygon = maxCircle;
	float minPolygon = minCircle;

	sf::Vector2f circlePoint1 = c.currentPosition - (axis * c.size);
	sf::Vector2f circlePoint2 = c.currentPosition + (axis * c.size);

	maxCircle = Engine::dotProduct(circlePoint1, axis);
	minCircle = Engine::dotProduct(circlePoint2, axis);

	if (maxCircle < minCircle) {
		float temp = maxCircle;
		maxCircle = minCircle;
		minCircle = temp;
	}

	for (int j = 0; j < polygonVertexCount; ++j) {
		sf::Vector2f vertexPosition = convexPolygon.getVertexPosition(j);
		float projection = Engine::dotProduct(vertexPosition, axis);

		if (projection > maxPolygon) {
			maxPolygon = projection;
		}
		if (projection < minPolygon) {
			minPolygon = projection;
		}
	}

	if (maxCircle < minPolygon || maxPolygon < minCircle) {
		//Not colliding.
		return;
	}
	//Projections are overlapping
	if (maxCircle - minPolygon < maxPolygon - minCircle) {
		depth = maxCircle - minPolygon;
	}
	else {
		depth = maxPolygon - minCircle;
	}

	if (depth < minimumDepth) {
		minimumDepth = depth;
		minimumAxis = axis;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.
	float distance = Engine::findDistance(Engine::findContactPoint(c, convexPolygon), c.currentPosition);
	if (distance > 10.1f) {
		std::cout << "Over the maximum expected" << std::endl;
		std::cout << distance << std::endl;
	}
	//Check the normal is facing in the correct direction
	if (dotProduct(convexPolygon.currentPosition - c.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	circlePolygonResolution(c, convexPolygon, minimumDepth, minimumAxis);
}
*/
void Engine::circlePolygonResolution(Circle& c, ConvexPolygon& polygon, float depth, sf::Vector2f axis, const sf::Vector2f& contactPointOnPolygon)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	const sf::Vector2f circleVelocity = (c.currentPosition - c.oldPosition);
	const sf::Vector2f polygonVelocity = (polygon.currentPosition - polygon.oldPosition);

	float normRelativeVelocity = Engine::dotProduct(polygonVelocity - circleVelocity, axis);

	float j;
	if (normRelativeVelocity > 0.f) {
		j = 0.f;
	}
	else {
		j = -(1 + (c.resCoeff > polygon.resCoeff ? c.resCoeff : polygon.resCoeff))
			* normRelativeVelocity / ((1.f / c.mass) + (1.f / polygon.mass));
	}

	//Separate the two shapes. It should be noted axis MUST be normalised here.
	c.currentPosition -= (0.5f * depth * axis);
	polygon.currentPosition += (0.5f * depth * axis);

	const sf::Vector2f contactPoint = contactPointOnPolygon + polygon.currentPosition;
	
	float distance = Engine::findDistance(contactPoint, c.currentPosition);
	//std::cout << "Distance between contact point and circle centre: " << distance << std::endl;
	//std::cout << "Distance between contact point and polygon centre: " << Engine::findDistance(contactPoint, polygon.currentPosition) << std::endl;


	c.oldPosition += (((j / (1.f / c.mass)) * axis) - 0.5f * depth * axis);
	polygon.oldPosition -= (((j / (1.f / polygon.mass)) * axis) - 0.5f * depth * axis);

	return;
}

sf::Vector2f Engine::closestPointOnSegmentToCircle(Circle& c, sf::Vector2f& a, sf::Vector2f& b)
{
	sf::Vector2f ab = b - a;
	sf::Vector2f ac = c.currentPosition - a;

	float abMagnitudeSquared = ((ab.x * ab.x) + (ab.y * ab.y));

	if (abMagnitudeSquared == 0) {
		//a and b are the same point
		return a;
	}

	float t = ((ac.x * ab.x) + (ac.y * ab.y)) / abMagnitudeSquared;

	if (t < 0) {
		return a;
	}
	else if (t > 1) {
		return b;
	}
	else {
		return { a.x + (t * ab.x), a.y + (t * ab.y) };
	}
}

void Engine::polygonPolygonDetection(ConvexPolygon& convexPolygon1, ConvexPolygon& convexPolygon2)
{	
	float minimumDepth = std::numeric_limits<float>::max();
	sf::Vector2f minimumAxis;
	
	if ((projectionSAT(convexPolygon1, convexPolygon2, minimumDepth, minimumAxis)) ||
		(projectionSAT(convexPolygon2, convexPolygon1, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (dotProduct(convexPolygon2.currentPosition - convexPolygon1.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}

	polygonPolygonResolution(convexPolygon1, convexPolygon2, minimumDepth, minimumAxis);
}

bool Engine::projectionSAT(ConvexPolygon& convexPolygon1, ConvexPolygon& convexPolygon2, float& minDepth, sf::Vector2f& minAxis)
{
	//SAT implementation to find out whether two polygons have collided
	//Similar to circle-polygon collision detection

	int polygon1VertexCount = convexPolygon1.body.getPointCount();
	int polygon2VertexCount = convexPolygon2.body.getPointCount();

	for (int i = 0; i < polygon1VertexCount; ++i) {

		sf::Vector2f axis;
		float depth;

		sf::Vector2f vertex1Position = convexPolygon1.getVertexPosition(i);
		sf::Vector2f vertex2Position = convexPolygon1.getVertexPosition((i + 1) % polygon1VertexCount);

		axis = sf::Vector2f(vertex2Position.y - vertex1Position.y, -(vertex2Position.x - vertex1Position.x));
		axis = Engine::normalise(axis);

		//Project all the vertices of both polygons onto the axis. Find the max and min for both polygons
		float max1 = std::numeric_limits<float>::min();
		float min1 = std::numeric_limits<float>::max();

		float max2 = max1;
		float min2 = min1;

		for (int j = 0; j < polygon1VertexCount; ++j) {
			sf::Vector2f vertexPosition = convexPolygon1.getVertexPosition(j);
			float projection = Engine::dotProduct(vertexPosition, axis);

			if (projection > max1) {
				max1 = projection;
			}
			if (projection < min1) {
				min1 = projection;
			}
		}
		for (int j = 0; j < polygon2VertexCount; ++j) {
			sf::Vector2f vertexPosition = convexPolygon2.getVertexPosition(j);
			float projection = Engine::dotProduct(vertexPosition, axis);

			if (projection > max2) {
				max2 = projection;
			}
			if (projection < min2) {
				min2 = projection;
			}
		}

		if (max1 < min2 || max2 < min1) {
			//Not colliding.
			return true;
		}
		//Projections are overlapping
		if (max1 - min2 < max2 - min1) {
			depth = max1 - min2;
		}
		else {
			depth = max2 - min1;
		}

		if (depth < minDepth) {
			minDepth = depth;
			minAxis = axis;
		}
	}
	return false;
}

void Engine::polygonPolygonResolution(ConvexPolygon& convexPolygon1, ConvexPolygon& convexPolygon2, float depth, sf::Vector2f axis)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	const sf::Vector2f polygon1Velocity = (convexPolygon1.currentPosition - convexPolygon1.oldPosition);
	const sf::Vector2f polygon2Velocity = (convexPolygon2.currentPosition - convexPolygon2.oldPosition);

	float normRelativeVelocity = Engine::dotProduct(polygon2Velocity - polygon1Velocity, axis);

	float j;
	if (normRelativeVelocity > 0.f) {
		j = 0.f;
	}
	else {
		j = -(1 + (convexPolygon1.resCoeff > convexPolygon2.resCoeff ? convexPolygon1.resCoeff : convexPolygon2.resCoeff))
			* normRelativeVelocity / ((1.f / convexPolygon1.mass) + (1.f / convexPolygon2.mass));
	}

	//Separate the two polygons. It should be noted axis MUST be normalised here.
	convexPolygon1.currentPosition -= (0.5f * depth * axis);
	convexPolygon2.currentPosition += (0.5f * depth * axis);
	
	convexPolygon1.oldPosition += (((j / (1.f / convexPolygon1.mass)) * axis) - 0.5f * depth * axis);
	convexPolygon2.oldPosition -= (((j / (1.f / convexPolygon2.mass)) * axis) - 0.5f * depth * axis);

	return;
}

sf::Vector2f Engine::closestPointOnLineSegment(const sf::Vector2f& circlePos, const sf::Vector2f& vertex1Pos, const sf::Vector2f& vertex2Pos)
{
	sf::Vector2f edge = vertex2Pos - vertex1Pos;
	float denominator = Engine::dotProduct(edge, edge);
	if (denominator == 0.0f) {
		return vertex1Pos;
	}

	float t = Engine::dotProduct((circlePos - vertex1Pos), edge) / denominator;

	//Clamp t between 0 and 1.
	if (t > 1.f) { t = 1.f; }
	else if (t < 0.f) { t = 0.f; }

	return (vertex1Pos + (t * (vertex2Pos - vertex1Pos)));
}

sf::Vector2f Engine::findContactPoint(Circle& c, ConvexPolygon& convexPolygon)
{
	sf::Vector2f contactPoint;
	float distanceToContactPoint = std::numeric_limits<float>::max();

	int polygonVertexCount = convexPolygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f tempVector = Engine::closestPointOnLineSegment(c.currentPosition, convexPolygon.getVertexPosition(i),
																	convexPolygon.getVertexPosition((i + 1) % polygonVertexCount));
		float tempDistance = Engine::findDistance(c.currentPosition, tempVector);

		if (tempDistance < distanceToContactPoint) {
			distanceToContactPoint = tempDistance;
			contactPoint = tempVector;
		}
	}
	return contactPoint;
}

float Engine::dotProduct(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return ((v1.x * v2.x) + (v1.y * v2.y));
}

sf::Vector2f Engine::normalise(const sf::Vector2f& v)
{
	return sf::Vector2f(v / pow(v.x * v.x + v.y * v.y , 0.5f));
}

float Engine::findDistance(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return pow(((v2.x - v1.x) * (v2.x - v1.x)) + ((v2.y - v1.y) * (v2.y - v1.y)), 0.5f);
}

void Engine::update(float dt)
{
	timeStep = dt;

	pollEvents();

	framerate = 1.f / clock.getElapsedTime().asSeconds();
	clock.restart();

	//addCircle();

	for (int n = 0; n < subSteps; ++n) {

		sortEntities();

		updateSprings();
		updateEntities();
		detectEntityBarrierCollision();
		wideSweep();
	}
	updateText();
}

void Engine::updateEntities()
{
	std::vector<Entity*>::iterator it;
	for(it = Entities.begin(); it != Entities.end();) {
		sf::Vector2f pos = (*it)->currentPosition;
		if (pos.y > f_windowHeight || pos.x > f_windowWidth || pos.x < 0 || pos.y < 0) {
			it = Entities.erase(it);
			std::cout << "Entity deleted" << std::endl;
		} else {
			solver(*(*it), timeStep / subSteps);
			(*it)->updatePosition();
			it++;
		}
	}
}

void Engine::updateSprings()
{
	std::vector<Spring>::iterator it;
	
	for (it = Springs.begin(); it != Springs.end();) {
		sf::Vector2f pos = it->position;
		if (pos.y > f_windowHeight || pos.x > f_windowWidth || pos.x < 0 || pos.y < 0) {
			it = Springs.erase(it);
			std::cout << "Spring deleted" << std::endl;
		}
		else {
			it->update();
			it->applyForces(timeStep);
			it++;
		}
	}
}

void Engine::updateText() 
{
	fpstext.setString("FPS: " + std::to_string(framerate));
	entitiesSpawnedText.setString("Entities spawned: " + std::to_string(entitiesSpawned));
}

void Engine::render()
{
	window->clear();

	for (Entity* entity : Entities) {
		entity->renderEntity(*window);
	}

	for (Spring& spring : Springs) {
		spring.render(*window);
	}

	upperBarrier.renderBarrier(*window);
	rightBarrier.renderBarrier(*window);
	lowerBarrier.renderBarrier(*window);
	leftBarrier.renderBarrier(*window);

	window->draw(fpstext);
	window->draw(entitiesSpawnedText);

	window->display();
}

const bool Engine::isWindowOpen() const
{
	if (window->isOpen()) {
		return true;
	} else {
		return false;
	}
}

Circle::Circle()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.f;

	mass = 1.f;
	size = 5.f;
	momentOfInertia = mass * size * size * 0.5f;
	resCoeff = 0.8f;

	body.setRadius(size);
	color = sf::Color::White;
	body.setFillColor(color);

	body.setOrigin(size, size);

	force = { 0.f, 0.f };
}

Circle::Circle(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.f;

	mass = inputMass;
	size = inputSize;
	momentOfInertia = mass * size * size * 0.5f;
	resCoeff = 0.8f;

	body.setRadius(size);
	color = inputColor;
	body.setFillColor(color);

	body.setOrigin(size, size);
	body.setRadius(size);

	force = { 0.f, 0.f };
}

Circle::~Circle()
{

}

void Circle::updatePosition()
{
	body.setPosition(currentPosition);
	body.setRotation(currentAngle);
}

void Circle::entityBarrierCollision()
{
	const sf::Vector2f displacement = this->currentPosition - this->oldPosition;

	if (this->currentPosition.y > lowerBarrier_position.y - this->size) {
		//Move the entity to the very edge where it would be acceptable for it to be in

		this->currentPosition.y = lowerBarrier_position.y - this->size;

		//Change the old position such that the entitiy receieves the right impulse. Term in brackets accounts for the coefficient 
		// of restitution since displacement is a stand-in for velocity.
		this->oldPosition.y = this->currentPosition.y + (this->resCoeff * displacement.y);
	}

	if (this->currentPosition.x > rightBarrier_position.x - this->size) {

		this->currentPosition.x = rightBarrier_position.x - this->size;

		this->oldPosition.x = this->currentPosition.x + (this->resCoeff * displacement.x);
	}

	if (this->currentPosition.y < upperBarrier_position.y + 20) {

		this->currentPosition.y = upperBarrier_position.y + 20;

		this->oldPosition.y = this->currentPosition.y + (this->resCoeff * displacement.y);
	}

	if (this->currentPosition.x < leftBarrier_position.x + 20 + this->size) {

		this->currentPosition.x = leftBarrier_position.x + 20 + this->size;

		this->oldPosition.x = this->currentPosition.x + (this->resCoeff * displacement.x);
	}
}

void Circle::detectEntityCollision(Entity& e)
{
	e.detectCircleCollision(*this);
}

void Circle::detectCircleCollision(Circle& c)
{
	Engine::circleCircleDetection(*this, c);
}

void Circle::detectSquareCollision(Square& s)
{
	Engine::circlePolygonDetection(*this, s);
}

void Circle::renderEntity(sf::RenderWindow& target)
{
	target.draw(body);
}

Square::Square()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.f;

	mass = 1.f;
	size = 10.f;
	momentOfInertia = mass * size * size / 6.f;

	body.setPointCount(4);
	body.setPoint(0, { -(size / 2.f), -(size / 2.f) });
	body.setPoint(1, { (size / 2.f), -(size / 2.f) });
	body.setPoint(2, { (size / 2.f), (size / 2.f) });
	body.setPoint(3, { -(size / 2.f), (size / 2.f) });

	color = sf::Color::Green;
	body.setFillColor(color);
	resCoeff = 0.8f;

	force = { 0.f, 0.f };
}

Square::Square(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.4f;

	mass = inputMass;
	size = inputSize;
	momentOfInertia = mass * size * size / 6.f;

	//Points for a square are defined starting in top left corner and moving clockwise.
	//Co-ordinates of points are relative to the position of the body

	body.setPointCount(4);
	body.setPoint(0, { -(size / 2.f), -(size / 2.f)});
	body.setPoint(1, { (size / 2.f), -(size / 2.f) });
	body.setPoint(2, { (size / 2.f), (size / 2.f) });
	body.setPoint(3, { -(size / 2.f), (size / 2.f) });

	color = inputColor;
	body.setFillColor(color);
	resCoeff = 0.8f;

	force = { 0.f, 0.f };
	/*
	body.setOutlineThickness(1);
	body.setOutlineColor(sf::Color(250, 150, 100));
	*/
}

Square::~Square()
{
}

void Square::updatePosition()
{
	body.setPosition(currentPosition);
	body.setRotation(currentAngle);
	//body.rotate(0.4f);
}

void Square::entityBarrierCollision()
{
	const sf::Vector2f displacement = this->currentPosition - this->oldPosition;

	if (this->currentPosition.y > lowerBarrier_position.y - 0.5f * this->size) {
		//Move the entity to the very edge where it would be acceptable for it to be in

		this->currentPosition.y = lowerBarrier_position.y - 0.5f * this->size;

		//Change the old position such that the entitiy receieves the right impulse. Term in brackets accounts for the coefficient 
		// of restitution since displacement is a stand-in for velocity.
		this->oldPosition.y = this->currentPosition.y + (this->resCoeff * displacement.y);
	}

	if (this->currentPosition.x > rightBarrier_position.x - 0.5f * this->size) {

		this->currentPosition.x = rightBarrier_position.x - 0.5f * this->size;

		this->oldPosition.x = this->currentPosition.x + (this->resCoeff * displacement.x);
	}
	//WHAT DOES THIS 20 REPRESENT?????
	if (this->currentPosition.y < upperBarrier_position.y + 20 + 0.5f * this->size) {

		this->currentPosition.y = upperBarrier_position.y + 20 + 0.5f * this->size;

		this->oldPosition.y = this->currentPosition.y + (this->resCoeff * displacement.y);
	}

	if (this->currentPosition.x < leftBarrier_position.x + 20 + 0.5f * this->size) {

		this->currentPosition.x = leftBarrier_position.x + 20 + 0.5f * this->size;

		this->oldPosition.x = this->currentPosition.x + (this->resCoeff * displacement.x);
	}
}

void Square::detectEntityCollision(Entity& e)
{
	e.detectSquareCollision(*this);
}

void Square::detectCircleCollision(Circle& c)
{
	Engine::circlePolygonDetection(c, *this);
}

void Square::detectSquareCollision(Square& s)
{
	Engine::polygonPolygonDetection(*this, s);
}

sf::Vector2f Square::getVertexPosition(int vertex)
{
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex);
	float angle = (3.14159 / 180.f) * currentAngle;

	float x = (nonRotatedPosition.x * std::cos(angle)) - (nonRotatedPosition.y * std::sin(angle));
	float y = (nonRotatedPosition.x * std::sin(angle)) + (nonRotatedPosition.y * std::cos(angle));

	return { x + currentPosition.x, y + currentPosition.y };
}

void Square::renderEntity(sf::RenderWindow& target)
{
	target.draw(body);
}

rectBarrier::rectBarrier()
{
	color = sf::Color::White;
	body.setFillColor(color);
	size = sf::Vector2f(10.f, 10.f);
	body.setSize(size);
	position = sf::Vector2f(0.f, 0.f);
	body.setPosition(position);
}

rectBarrier::rectBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition)
{
	color = inputColor;
	body.setFillColor(color);
	size = inputSize;
	body.setSize(size);
	position = inputPosition;
	body.setPosition(position);
}

rectBarrier::~rectBarrier() 
{
	
}

void rectBarrier::setBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition)
{
	color = inputColor;
	body.setFillColor(color);
	size = inputSize;
	body.setSize(size);
	position = inputPosition;
	body.setPosition(position);
}

void rectBarrier::renderBarrier(sf::RenderWindow& target)
{
	target.draw(body);
}

Spring::Spring(Entity* inputEntity1, Entity* inputEntity2, float inputSpringConstant, float inputDampingConstant, float inputRestLength)
{
	entity1 = inputEntity1;
	entity2 = inputEntity2;
	springWidth = 10.f;
	springConstant = inputSpringConstant;
	dampingConstant = inputDampingConstant;
	restLength = inputRestLength;

	springBody.setFillColor(sf::Color::White);
	springBody.setSize({ abs(entity1->currentPosition.x - entity2->currentPosition.x), springWidth });
	springBody.setPosition({ entity1->currentPosition.x, entity1->currentPosition.y });
	springBody.setOrigin({ 0, springWidth / 2});
	position = entity1->currentPosition;

	e1RestDistance = restLength * entity2->mass / (entity1->mass + entity2->mass);
	e2RestDistance = restLength - e1RestDistance;
}

Spring::~Spring()
{

}

void Spring::update()
{
	const sf::Vector2f v = entity2->currentPosition - entity1->currentPosition;

	springBody.setPosition({ entity1->currentPosition.x, entity1->currentPosition.y});
	position = entity1->currentPosition;

	springBody.setSize({ pow(v.x * v.x + v.y * v.y, 0.5f), springWidth });

	springBody.setRotation((180.f / 3.14f) * (atan2(10.f, 0.f) - atan2(v.x ,v.y)));

	centreOfMass = (entity1->mass * entity1->currentPosition + entity2->mass * entity2->currentPosition) / (entity1->mass + entity2->mass);
}

void Spring::applyForces(float dt)
{
	//Will first find where both entities should be a.k.a their rest positions
	const sf::Vector2f v = entity2->currentPosition - entity1->currentPosition;

	//Find the velocity of each entity from the positions
	const sf::Vector2f entity1velocity = (entity1->currentPosition - entity1->oldPosition) / dt;
	const sf::Vector2f entity2velocity = (entity2->currentPosition - entity2->oldPosition) / dt;

	const sf::Vector2f e1RestPos = centreOfMass - (v / pow(v.x * v.x + v.y * v.y, 0.5f) * e1RestDistance);
	const sf::Vector2f e2RestPos = centreOfMass + (v / pow(v.x * v.x + v.y * v.y, 0.5f) * e2RestDistance);

	//Apply the restoring force to the entities
	entity1->force += -springConstant * (entity1->currentPosition - e1RestPos) - dampingConstant * (entity1velocity - entity2velocity);
	entity2->force += -springConstant * (entity2->currentPosition - e2RestPos) - dampingConstant * (entity2velocity - entity1velocity);
}

void Spring::render(sf::RenderWindow& target)
{
	target.draw(springBody);
}
