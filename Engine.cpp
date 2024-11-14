/*
* Note that very small acceleration doesn't work due to floating point precision.
* Bug? Feature? It looks like circles collide with squares, the circle loses
* all momentum perpendicular to the edge of the circle it collided with.
* Consider updating once every single frame. Should be no problem since barriers
* cannot be moved by other objects.
*/

#include "Engine.h"

int windowWidth = 1200;
int windowHeight = 800;
float f_windowWidth = 1200.f;
float f_windowHeight = 800.f;

int counter = 0;
float float_upperLimit = std::numeric_limits<float>::max();
float float_lowerLimit = std::numeric_limits<float>::lowest();
float root3 = 1.73205;

int cell_size = 40;
int cell_number_x = windowWidth / cell_size;
int cell_number_y = windowHeight / cell_size;

extern sf::Vector2f lowerBarrier_position = { f_windowWidth * 0.5f , f_windowHeight * 0.95f };
extern sf::Vector2f upperBarrier_position = { f_windowWidth * 0.5f , 10.f };
extern sf::Vector2f rightBarrier_position = { f_windowWidth - 10.f , f_windowHeight * 0.5f};
extern sf::Vector2f leftBarrier_position = { 10.f, f_windowHeight * 0.5f };

std::vector<sf::RectangleShape> Engine::objectsToDraw;

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
	subSteps = 4;
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
	rectBarriers.emplace_back(sf::Color::White, sf::Vector2f(f_windowWidth, 20), upperBarrier_position);
	sf::Vector2f upperBarrier_position = { 0.f ,0.f };

	rectBarriers.emplace_back(sf::Color::White, sf::Vector2f(20, f_windowHeight), rightBarrier_position);
	sf::Vector2f rightBarrier_position = { f_windowWidth - 20.f ,0.f };

	rectBarriers.emplace_back(sf::Color::White, sf::Vector2f(f_windowWidth, 20), lowerBarrier_position);
	sf::Vector2f lowerBarrier_position = { 0.f , f_windowHeight * 0.95f };

	rectBarriers.emplace_back(sf::Color::White, sf::Vector2f(20, f_windowHeight), leftBarrier_position);
	sf::Vector2f leftBarrier_position = { 0.f , 0.f };
}

void Engine::initWindow()
{
	window = new sf::RenderWindow(sf::VideoMode(windowWidth, windowHeight), "Physics Engine");
	window->setFramerateLimit(0);
}

void Engine::addCircle()
{
	sf::Color colour = sf::Color(rand() % 100 + 155, rand() % 100 + 155, rand() % 100 + 155);
	Entities.emplace_back(new Circle(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 10.f, colour));
	entitiesSpawned++;
}

void Engine::addSquare()
{
	sf::Color colour = sf::Color(rand() % 100 + 155, rand() % 100 + 155, rand() % 100 + 155);
	Entities.emplace_back(new Square(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 20.f, colour));
	entitiesSpawned++;
}

void Engine::addTriangle()
{
	sf::Color colour = sf::Color(rand() % 100 + 155, rand() % 100 + 155, rand() % 100 + 155);
	Entities.emplace_back(new Triangle(sf::Vector2f(rand() % 1000 + 100, 100.f), 1.f, 20.f, colour));
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
			} else if (e.key.code == sf::Keyboard::L) {
				addTriangle();
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

	//Damping coefficient
	angularDisplacement *= 0.995f;

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
	E.force += {0.f * E.mass, (500.f * E.mass)}; //Gravity
}

void Engine::detectEntityBarrierCollision()
{
	size_t entityNumber = Entities.size();
	for (size_t i = 0; i < entityNumber; ++i) {
		//Consider doing a pseudo wide sweep here. Would need to be able to access entities size
		size_t barrierNumber = rectBarriers.size();
		for (size_t j = 0; j < barrierNumber; ++j) {
			Entities[i]->detectBarrierCollision(rectBarriers[j]);
		}
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
	int cellColumn = std::floor(pos.x / cell_size);
	int cellRow = std::floor(pos.y / cell_size) * cell_number_x;

	if (cellColumn > cell_number_x || cellColumn < 0) {
		return 0;
	}
	if (cellRow > cell_number_y || cellRow < 0) {
		return 0;
	}
	return cellColumn * cellRow;
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
	sf::Vector2f overLapCorrection = 0.5f * (c1.size + c2.size - depth) * axis;

	c1.currentPosition -= overLapCorrection;
	c1.oldPosition -= overLapCorrection;

	c2.currentPosition += overLapCorrection;
	c2.oldPosition += overLapCorrection;

	//Resolve collision between the circles
	const sf::Vector2f vel_c2 = c2.currentPosition - c2.oldPosition;
	const sf::Vector2f vel_c1 = c1.currentPosition - c1.oldPosition;

	const float norm_vel_relative = Engine::dotProduct(vel_c2 - vel_c1, axis);

	//Only take impulse into account if objects are moving towards each other.
	if (norm_vel_relative > 0.f) {
		return;
	}

	float restitutionCoefficient = (c1.resCoeff > c2.resCoeff ? c2.resCoeff : c1.resCoeff);
	
	float j = -(1.f + restitutionCoefficient) *
		norm_vel_relative / ((1.f / c1.mass) + (1.f / c2.mass));


	c1.oldPosition += (j / (1.f / c1.mass)) * axis;
	c2.oldPosition -= (j / (1.f / c2.mass)) * axis;
}

void Engine::circlePolygonDetection(Circle& c, ConvexPolygon& polygon)
{
	if (!Engine::checkBoundingBox(c, polygon)) {
		return;
	}

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

void Engine::circlePolygonResolution(Circle& c, ConvexPolygon& polygon, float depth, sf::Vector2f axis, const sf::Vector2f& contactPointOnPolygon)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	//Denoting circle as A, polygon as B and contact point as P.
	sf::Vector2f overlapCorrection = 0.5f * depth * axis;

	c.currentPosition -= overlapCorrection;
	c.oldPosition -= overlapCorrection;

	polygon.currentPosition += overlapCorrection;
	polygon.oldPosition += overlapCorrection;

	sf::Vector2f r_PA = (polygon.currentPosition + contactPointOnPolygon) - c.currentPosition;

	sf::Vector2f v_A = c.currentPosition - c.oldPosition;
	v_A -= (c.currentAngle - c.oldAngle) * (3.141592f / 180.f) * Engine::perpendicular(r_PA);

	sf::Vector2f v_B = polygon.currentPosition - polygon.oldPosition;
	v_B -= (polygon.currentAngle - polygon.oldAngle) * (3.141592f / 180.f) * Engine::perpendicular(contactPointOnPolygon);

	float contactVelocity_mag = Engine::dotProduct(v_B - v_A, axis);

	if (contactVelocity_mag > 0.f) {
		return;
	}

	float r_PA_PerpDotN = Engine::dotProduct(Engine::perpendicular(r_PA), axis);
	float r_PB_PerpDotN = Engine::dotProduct(Engine::perpendicular(contactPointOnPolygon), axis);

	float denom = (1.f / c.mass) + (1.f / polygon.mass) +
		(r_PA_PerpDotN * r_PA_PerpDotN) * (1.f / c.momentOfInertia) +
		(r_PB_PerpDotN * r_PB_PerpDotN) * (1.f / polygon.momentOfInertia);

	//Modify this to find the lower of the two restitution coefficients
	float restitutionCoefficient = (c.resCoeff > polygon.resCoeff ? polygon.resCoeff : c.resCoeff);

	float j = -(1.f + restitutionCoefficient) * contactVelocity_mag;
	j /= denom;

	sf::Vector2f impulse = j * axis;

	c.oldPosition += impulse * (1.f / c.mass);
	c.oldAngle += Engine::crossProduct(r_PA, impulse) * (1.f / c.momentOfInertia);

	polygon.oldPosition -= impulse * (1.f / polygon.mass);
	polygon.oldAngle -= Engine::crossProduct(contactPointOnPolygon, impulse) * (1.f / polygon.momentOfInertia);
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

void Engine::polygonPolygonDetection(ConvexPolygon& polygonA, ConvexPolygon& polygonB)
{
	if (!Engine::checkBoundingBox(polygonA, polygonB)) {
		return;
	}

	float minimumDepth = float_upperLimit;
	sf::Vector2f minimumAxis = {-1.f, -1.f};
	
	std::vector<sf::Vector2f> polygonAVertices = Engine::getPolygonVertexPositions(polygonA);
	std::vector<sf::Vector2f> polygonBVertices = Engine::getPolygonVertexPositions(polygonB);

	if ((projectionSAT(polygonAVertices, polygonBVertices, minimumDepth, minimumAxis)) ||
		(projectionSAT(polygonBVertices, polygonAVertices, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (dotProduct(polygonB.currentPosition - polygonA.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	//std::cout << "Collision axis: " << minimumAxis.x << " " << minimumAxis.y << std::endl;
	//std::cout << "Collision depth: " << minimumDepth << std::endl;

	//From this, conclude that the vertices are found where they should be. All are 23.09 away from the centre.
	/*
	if (minimumDepth > 1.0f && polygonAVertices.size() < 4 && polygonBVertices.size() < 4) {
		std::cout << "Somthing happened" << std::endl;
		std::cout << "Distance between the two shapes: " << Engine::findDistance(polygonA.currentPosition, polygonB.currentPosition) << std::endl;
		std::cout << "Distance between polygon A and its vertices: " << Engine::findDistance(polygonA.currentPosition, polygonAVertices[0]) << std::endl;
		std::cout << Engine::findDistance(polygonA.currentPosition, polygonAVertices[1]) << std::endl;
		std::cout << Engine::findDistance(polygonA.currentPosition, polygonAVertices[2]) << std::endl;
		std::cout << "Distance between polygon B and its vertices: " << Engine::findDistance(polygonB.currentPosition, polygonBVertices[0]) << std::endl;
		std::cout << Engine::findDistance(polygonB.currentPosition, polygonBVertices[1]) << std::endl;
		std::cout << Engine::findDistance(polygonB.currentPosition, polygonBVertices[2]) << std::endl;
	}
	*/

	/*

	sf::RectangleShape temp({ 5.f, 5.f });
	temp.setPosition(polygonA.currentPosition);
	temp.setFillColor(sf::Color::Red);
	Engine::objectsToDraw.push_back(temp);

	sf::RectangleShape temp2({ 5.f, 5.f });
	temp2.setPosition(polygonB.currentPosition);
	temp2.setFillColor(sf::Color::Red);
	Engine::objectsToDraw.push_back(temp2);

	for (int i = 0; i < polygonAVertices.size(); ++i) {
		temp.setPosition(polygonAVertices[i]);
		Engine::objectsToDraw.push_back(temp);
	}

	for (int i = 0; i < polygonBVertices.size(); ++i) {
		temp.setPosition(polygonBVertices[i]);
		Engine::objectsToDraw.push_back(temp);
	}

	*/

	polygonPolygonResolution(polygonA, polygonB, minimumDepth, minimumAxis);
}

bool Engine::projectionSAT(std::vector<sf::Vector2f>& polygonAVertices, std::vector<sf::Vector2f>& polygonBVertices, float& minDepth, sf::Vector2f& minAxis)
{
	//SAT implementation to find out whether two polygons have collided
	//Similar to circle-polygon collision detection

	int polygonAVertexCount = polygonAVertices.size();
	int polygonBVertexCount = polygonBVertices.size();

	for (int i = 0; i < polygonAVertexCount; ++i) {

		sf::Vector2f axis;
		float depth;

		sf::Vector2f vertex1Position = polygonAVertices[i];
		sf::Vector2f vertex2Position = polygonAVertices[(i + 1) % polygonAVertexCount];

		axis = Engine::perpendicular(vertex2Position - vertex1Position);
		axis = Engine::normalise(axis);

		//Project all the vertices of both polygons onto the axis. Find the max and min for both polygons
		float max1 = float_lowerLimit;
		float min1 = float_upperLimit;

		float max2 = max1;
		float min2 = min1;

		for (int j = 0; j < polygonAVertexCount; ++j) {
			sf::Vector2f vertexPosition = polygonAVertices[j];
			float projection = Engine::dotProduct(vertexPosition, axis);

			if (projection > max1) {
				max1 = projection;
			}
			if (projection < min1) {
				min1 = projection;
			}
		}
		for (int j = 0; j < polygonBVertexCount; ++j) {
			sf::Vector2f vertexPosition = polygonBVertices[j];
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
			//std::cout << "not colliding" << std::endl;
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
	//std::cout << "Colliding" << std::endl;
	return false;
}

void Engine::polygonPolygonResolution(ConvexPolygon& polygonA, ConvexPolygon& polygonB, float depth, sf::Vector2f axis)
{
	sf::Vector2f overlapCorrection = 0.5f * axis * depth;

	//std::cout << "Depth of collision: " << depth << std::endl;

	polygonA.currentPosition -= overlapCorrection;
	polygonA.oldPosition -= overlapCorrection;

	polygonB.currentPosition += overlapCorrection;
	polygonB.oldPosition += overlapCorrection;

	std::vector<sf::Vector2f> contactPoints(2);
	int contactCount = 0;

	Engine::findContactPoints(polygonA, polygonB, contactPoints[0], contactPoints[1], contactCount);

	//TEMPORARY STUFF ADD TO SUPPORT DRAWING CONTACT POINTS.
	
	sf::RectangleShape temp({ 5.f, 5.f });
	temp.setPosition(contactPoints[0]);
	temp.setFillColor(sf::Color::Red);
	Engine::objectsToDraw.push_back(temp);
	
	//std::cout << "Contact point registered." << std::endl;

	std::vector<sf::Vector2f> r_PA_List(2);
	std::vector<sf::Vector2f> r_PB_List(2);
	std::vector<sf::Vector2f> impulseList(2);

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f r_PA = contactPoints[i] - polygonA.currentPosition;
		sf::Vector2f r_PB = contactPoints[i] - polygonB.currentPosition;

		r_PA_List[i] = r_PA;
		r_PB_List[i] = r_PB;

		sf::Vector2f v_A = (polygonA.currentPosition - polygonA.oldPosition);
		v_A -= (polygonA.currentAngle - polygonA.oldAngle) * (3.141592f / 180.f) * Engine::perpendicular(r_PA);

		sf::Vector2f v_B = (polygonB.currentPosition - polygonB.oldPosition);
		v_B -= (polygonB.currentAngle - polygonB.oldAngle) * (3.141592f / 180.f) * Engine::perpendicular(r_PB);

		float contactVelocity_mag = Engine::dotProduct(v_B - v_A, axis);

		if (contactVelocity_mag > 0.f)
		{
			continue;
		}

		float r_PA_PerpDotN = Engine::dotProduct(Engine::perpendicular(r_PA), axis);
		float r_PB_PerpDotN = Engine::dotProduct(Engine::perpendicular(r_PB), axis);

		float denom = (1.f / polygonA.mass) + (1.f / polygonB.mass) +
			(r_PA_PerpDotN * r_PA_PerpDotN) * (1.f / polygonA.momentOfInertia) +
			(r_PB_PerpDotN * r_PB_PerpDotN) * (1.f / polygonB.momentOfInertia);

		float restitutionCoefficient = (polygonA.resCoeff > polygonB.resCoeff ? polygonB.resCoeff : polygonA.resCoeff);

		float j = -(1.f + restitutionCoefficient) * contactVelocity_mag;
		j /= denom;
		j /= contactCount;

		sf::Vector2f impulse = j * axis;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f impulse = impulseList[i];
		sf::Vector2f r_PA = r_PA_List[i];
		sf::Vector2f r_PB = r_PB_List[i];

		polygonA.oldPosition += impulse * (1.f / polygonA.mass);
		polygonA.oldAngle += Engine::crossProduct(r_PA, impulse) * (1.f / polygonA.momentOfInertia) * (180.f / 3.141592f);

		polygonB.oldPosition -= impulse * (1.f / polygonB.mass);
		polygonB.oldAngle -= Engine::crossProduct(r_PB, impulse) * (1.f / polygonB.momentOfInertia) * (180.f / 3.141592f);
	}

	return;
}

bool Engine::checkBoundingBox(Circle& circle, rectBarrier& barrier)
{
	//NOTE: ASSUMES BARRIER IS NOT ROTATED

	float maxX_circle = 0.f;
	float minX_circle = 0.f;
	float maxY_circle = 0.f;
	float minY_circle = 0.f;

	sf::Vector2f barrierSize = barrier.size;

	float maxX_barrier = barrier.position.x + barrierSize.x;
	float minX_barrier = barrier.position.x - barrierSize.x;
	float maxY_barrier = barrier.position.y + barrierSize.y;
	float minY_barrier = barrier.position.y - barrierSize.y;

	circle.getBoundingBox(maxX_circle, minX_circle, maxY_circle, minY_circle);

	return (minX_circle < maxX_barrier && maxX_circle > minX_barrier &&
			minY_circle < maxY_barrier && maxY_circle > minY_barrier);
}

bool Engine::checkBoundingBox(ConvexPolygon& polygon, rectBarrier& barrier)
{
	//NOTE: ASSUMES BARRIER IS NOT ROTATED

	float maxX_polygon = 0.f;
	float minX_polygon = 0.f;
	float maxY_polygon = 0.f;
	float minY_polygon = 0.f;

	sf::Vector2f barrierSize = barrier.size;

	float maxX_barrier = barrier.position.x + barrierSize.x;
	float minX_barrier = barrier.position.x - barrierSize.x;
	float maxY_barrier = barrier.position.y + barrierSize.y;
	float minY_barrier = barrier.position.y - barrierSize.y;

	polygon.getBoundingBox(maxX_polygon, minX_polygon, maxY_polygon, minY_polygon);

	return (minX_polygon < maxX_barrier && maxX_polygon > minX_barrier &&
			minY_polygon < maxY_barrier && maxY_polygon > minY_barrier);
}

bool Engine::checkBoundingBox(Circle& circle, ConvexPolygon& polygon)
{
	float maxX_circle = 0.f;
	float minX_circle = 0.f;
	float maxY_circle = 0.f;
	float minY_circle = 0.f;

	float maxX_polygon = 0.f;
	float minX_polygon = 0.f;
	float maxY_polygon = 0.f;
	float minY_polygon = 0.f;

	circle.getBoundingBox(maxX_circle, minX_circle, maxY_circle, minY_circle);
	polygon.getBoundingBox(maxX_polygon, minX_polygon, maxY_polygon, minY_polygon);

	return (minX_circle < maxX_polygon && maxX_circle > minX_polygon &&
			minY_circle < maxY_polygon && maxY_circle > minY_polygon);
}

bool Engine::checkBoundingBox(ConvexPolygon& polygonA, ConvexPolygon& polygonB)
{
	float maxX_A = 0.f;
	float minX_A = 0.f;
	float maxY_A = 0.f;
	float minY_A = 0.f;

	float maxX_B = 0.f;
	float minX_B = 0.f;
	float maxY_B = 0.f;
	float minY_B = 0.f;

	polygonA.getBoundingBox(maxX_A, minX_A, maxY_A, minY_A);
	polygonB.getBoundingBox(maxX_B, minX_B, maxY_B, minY_B);

	return (minX_A < maxX_B && maxX_A > minX_B &&
			minY_A < maxY_B && maxY_A > minY_B);
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
	float distanceToContactPoint = float_upperLimit;

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

void Engine::findContactPoints(ConvexPolygon& polygonA, ConvexPolygon& polygonB, sf::Vector2f& contactPoint1, sf::Vector2f& contactPoint2, int& contactCount)
{
	contactPoint1 = { -1.f, -1.f };
	contactPoint2 = { -1.f, -1.f };

	float minDist = float_upperLimit;
	int polygonAVertexCount = polygonA.body.getPointCount();
	int polygonBVertexCount = polygonB.body.getPointCount();
	std::vector<sf::Vector2f> polygonAVertices(polygonAVertexCount);

	for (int i = 0; i < polygonAVertexCount; ++i) {
		polygonAVertices[i] = polygonA.getVertexPosition(i);
	}

	for (int i = 0; i < polygonAVertexCount; i++)
	{
		sf::Vector2f point = polygonAVertices[i];

		for (int j = 0; j < polygonBVertexCount; j++)
		{
			sf::Vector2f v1 = polygonB.getVertexPosition(j);
			sf::Vector2f v2 = polygonB.getVertexPosition((j + 1) % polygonBVertexCount);

			sf::Vector2f contactPoint = closestPointOnLineSegment(point, v1, v2);
			float dist = Engine::findDistance(contactPoint, point);
			
			if (Engine::nearlyEqual(dist, minDist))
			{
				if (!Engine::nearlyEqual(contactPoint, contactPoint1)) {
					contactPoint2 = contactPoint;
					contactCount = 2;
				}
			}
			
			else if (dist < minDist)
			{
				minDist = dist;
				contactCount = 1;
				contactPoint1 = contactPoint;
			}
		}
	}

	for (int i = 0; i < polygonBVertexCount; i++)
	{
		sf::Vector2f point = polygonB.getVertexPosition(i);

		for (int j = 0; j < polygonAVertexCount; j++)
		{
			sf::Vector2f v1 = polygonAVertices[j];
			sf::Vector2f v2 = polygonAVertices[(j + 1) % polygonAVertexCount];

			sf::Vector2f contactPoint = Engine::closestPointOnLineSegment(point, v1, v2);
			float dist = Engine::findDistance(contactPoint, point);

			if (Engine::nearlyEqual(dist, minDist))
			{
				if (!Engine::nearlyEqual(contactPoint, contactPoint1)) {
					contactPoint2 = contactPoint;
					contactCount = 2;
				}
			}

			else if (dist < minDist)
			{
				minDist = dist;
				contactCount = 1;
				contactPoint1 = contactPoint;
			}
		}
	}
	return;
}

void Engine::findContactPoints(ConvexPolygon& polygon, rectBarrier& barrier, sf::Vector2f& contactPoint1, sf::Vector2f& contactPoint2, int& contactCount)
{
	contactPoint1 = {-1.f, -1.f};
	contactPoint2 = { -1.f, -1.f };

	float minDist = float_upperLimit;
	int polygonVertexCount = polygon.body.getPointCount();
	int barrierVertexCount = 4;
	std::vector<sf::Vector2f> polygonVertices(polygonVertexCount);

	for (int i = 0; i < polygonVertexCount; ++i) {
		polygonVertices[i] = polygon.getVertexPosition(i);
	}

	for (int i = 0; i < polygonVertexCount; i++)
	{
		sf::Vector2f point = polygonVertices[i];

		for (int j = 0; j < barrierVertexCount; j++)
		{
			sf::Vector2f v1 = barrier.vertexPositions[j];
			sf::Vector2f v2 = barrier.vertexPositions[(j + 1) % barrierVertexCount];

			sf::Vector2f contactPoint = closestPointOnLineSegment(point, v1, v2);
			float dist = Engine::findDistance(contactPoint, point);

			if (Engine::nearlyEqual(dist, minDist))
			{
				if (!Engine::nearlyEqual(contactPoint, contactPoint1)) {
					contactPoint2 = contactPoint;
					contactCount = 2;
				}
			}
			else if (dist < minDist)
			{
				minDist = dist;
				contactCount = 1;
				contactPoint1 = contactPoint;
			}
		}
	}

	for (int i = 0; i < barrierVertexCount; i++)
	{
		sf::Vector2f point = barrier.vertexPositions[i];

		for (int j = 0; j < polygonVertexCount; j++)
		{
			sf::Vector2f v1 = polygonVertices[j];
			sf::Vector2f v2 = polygonVertices[(j + 1) % polygonVertexCount];

			sf::Vector2f contactPoint = Engine::closestPointOnLineSegment(point, v1, v2);
			float dist = Engine::findDistance(contactPoint, point);

			if (Engine::nearlyEqual(dist, minDist))
			{
				if (!Engine::nearlyEqual(contactPoint, contactPoint1)) {
					contactPoint2 = contactPoint;
					contactCount = 2;
				}
			}

			else if (dist < minDist)
			{
				minDist = dist;
				contactCount = 1;
				contactPoint1 = contactPoint;
			}
		}
	}
}

std::pair<sf::Vector2f, sf::Vector2f> Engine::findReferenceEdge(ConvexPolygon& polygonA, ConvexPolygon& polygonB, sf::Vector2f& axis)
{
	float maxProjection = float_lowerLimit;
	std::pair<sf::Vector2f, sf::Vector2f> bestEdge;
	int polygonAVertexCount = polygonA.body.getPointCount();

	for (int i = 0; i < polygonAVertexCount; ++i) {
		sf::Vector2f v1 = polygonA.getVertexPosition(i);
		sf::Vector2f v2 = polygonA.getVertexPosition((i + 1) % polygonAVertexCount);

		float tempProjection = Engine::dotProduct(Engine::normalise(v2 - v1), axis);

		if (tempProjection > maxProjection) {
			maxProjection = tempProjection;
			bestEdge = { v1, v2 };
		}
	}

	int polygonBVertexCount = polygonB.body.getPointCount();

	for (int i = 0; i < polygonBVertexCount; ++i) {
		sf::Vector2f v1 = polygonB.getVertexPosition(i);
		sf::Vector2f v2 = polygonB.getVertexPosition((i + 1) % polygonBVertexCount);

		float tempProjection = Engine::dotProduct(Engine::normalise(v2 - v1), axis);

		if (tempProjection > maxProjection) {
			maxProjection = tempProjection;
			bestEdge = { v1, v2 };
		}
	}

	return bestEdge;
}

std::pair<sf::Vector2f, sf::Vector2f> Engine::findReferenceEdge(ConvexPolygon& polygon, rectBarrier& barrier, sf::Vector2f& axis)
{
	/*
	* Get rid of these two functions by taking in vertices as input. Less repeated code.
	*/
	float maxProjection = float_lowerLimit;
	std::pair<sf::Vector2f, sf::Vector2f> bestEdge;
	int polygonVertexCount = polygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f v1 = polygon.getVertexPosition(i);
		sf::Vector2f v2 = polygon.getVertexPosition((i + 1) % polygonVertexCount);

		float tempProjection = Engine::dotProduct(Engine::normalise(v2 - v1), axis);

		if (tempProjection > maxProjection) {
			maxProjection = tempProjection;
			bestEdge = { v1, v2 };
		}
	}

	int barrierVertexCount = barrier.body.getPointCount();

	for (int i = 0; i < barrierVertexCount; ++i) {
		sf::Vector2f v1 = barrier.vertexPositions[i];
		sf::Vector2f v2 = barrier.vertexPositions[(i + 1) % barrierVertexCount];

		float tempProjection = Engine::dotProduct(Engine::normalise(v2 - v1), axis);

		if (tempProjection > maxProjection) {
			maxProjection = tempProjection;
			bestEdge = { v1, v2 };
		}
	}

	return bestEdge;
}

void Engine::polygonBarrierDetection(ConvexPolygon& polygon, rectBarrier& barrier)
{
	if (!Engine::checkBoundingBox(polygon, barrier)) {
		return;
	}

	//std::cout << "Checking" << std::endl;
	float minimumDepth = float_upperLimit;
	sf::Vector2f minimumAxis;

	sf::Vector2f barrierPosition = barrier.position;

	std::vector<sf::Vector2f> polygonVertices = Engine::getPolygonVertexPositions(polygon);

	if ((projectionSAT(polygonVertices, barrier.vertexPositions, minimumDepth, minimumAxis)) ||
		(projectionSAT(barrier.vertexPositions, polygonVertices, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (dotProduct(barrierPosition - polygon.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	polygonBarrierResolution(polygon, barrier, minimumDepth, minimumAxis);
}

void Engine::polygonBarrierResolution(ConvexPolygon& polygon, rectBarrier& barrier, const float& depth, const sf::Vector2f& axis)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	
	polygon.currentPosition -= (depth * axis);
	polygon.oldPosition -= (depth * axis);

	std::vector<sf::Vector2f> contactPoints(2);
	int contactCount = 0;

	Engine::findContactPoints(polygon, barrier, contactPoints[0], contactPoints[1], contactCount);

	//TEMPORARY STUFF ADD TO SUPPORT DRAWING CONTACT POINTS.
	/*
	sf::RectangleShape temp({ 5.f, 5.f });
	temp.setPosition(contactPoints[0]);
	temp.setFillColor(sf::Color::Red);
	Engine::objectsToDraw.push_back(temp);
	//std::cout << "Contact point registered." << std::endl;
	*/

	std::vector<sf::Vector2f> raList(2);
	std::vector<sf::Vector2f> impulseList(2);

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f polygonToContactPoint = contactPoints[i] - polygon.currentPosition;

		raList[i] = polygonToContactPoint;

		sf::Vector2f polygonVelocity = (polygon.currentPosition - polygon.oldPosition);

		polygonVelocity -= (polygon.currentAngle - polygon.oldAngle) * (3.141592f / 180.f) * Engine::perpendicular(polygonToContactPoint);

		float contactVelocityMag = Engine::dotProduct(-polygonVelocity, axis);

		if (contactVelocityMag > 0.f)
		{
			continue;
		}
		
		float raPerpDotN = Engine::dotProduct(Engine::perpendicular(polygonToContactPoint), axis);

		float denom = (1.f / polygon.mass) + 0.f +
			(raPerpDotN * raPerpDotN) * (1.f / polygon.momentOfInertia) +
			0.f;

		float j = -(1.f + polygon.resCoeff) * contactVelocityMag;
		j /= denom;
		j /= contactCount;

		sf::Vector2f impulse = j * axis;
		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f impulse = impulseList[i];
		sf::Vector2f ra = raList[i];

		polygon.oldPosition += impulse * (1.f / polygon.mass);
		polygon.oldAngle += Engine::crossProduct(ra, impulse) * (1.f / polygon.momentOfInertia) * (180.f / 3.141592f);
	}

	return;
}

void Engine::circleBarrierDetection(Circle& c, rectBarrier& b)
{
	if (!Engine::checkBoundingBox(c, b)) {
		return;
	}

	sf::Vector2f closestPoint;
	int barrierVertexCount = b.body.getPointCount();

	for (int i = 0; i < barrierVertexCount; ++i) {
		sf::Vector2f tempVector = Engine::closestPointOnLineSegment(c.currentPosition, b.vertexPositions[i],
			b.vertexPositions[(i + 1) % barrierVertexCount]);
		float tempDistance = Engine::findDistance(c.currentPosition, tempVector);

		if (tempDistance < c.size) {
			sf::Vector2f axis = tempVector - c.currentPosition;
			axis = Engine::normalise(axis);

			//Check the normal is facing in the correct direction
			if (dotProduct(b.position - c.currentPosition, axis) < 0.f) {
				axis = { -axis.x, -axis.y };
			}
			//std::cout << "Distance being passed: " << tempDistance << std::endl;
			Engine::circleBarrierResolution(c, b, (c.size - tempDistance), axis, tempVector - b.position);
		}
	}
}

void Engine::circleBarrierResolution(Circle& c, rectBarrier& b, const float& depth, const sf::Vector2f& axis, const sf::Vector2f& contactPointOnBarrier)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	//Denoting circle as A and barrier as B.
	c.currentPosition -= (depth * axis);
	c.oldPosition -= (depth * axis);

	sf::Vector2f circleToContactPoint = (contactPointOnBarrier + b.position) - c.currentPosition;

	sf::Vector2f circleVelocity = (c.currentPosition - c.oldPosition);
	circleVelocity -= (c.currentAngle - c.oldAngle) * (3.141592f / 180.f) * Engine::perpendicular(circleToContactPoint);

	float contactVelocityMag = Engine::dotProduct(-circleVelocity, axis);

	if (contactVelocityMag > 0.f) {
		return;
	}

	float raPerpDotN = Engine::dotProduct(Engine::perpendicular(circleToContactPoint), axis);

	float denom = (1.f / c.mass) + 0.f +
		(raPerpDotN * raPerpDotN) * (1.f / c.momentOfInertia) +
		0.f;


	float j = -(1 + c.resCoeff) * contactVelocityMag;
	j /= denom;

	sf::Vector2f impulse = j * axis;

	c.oldPosition += impulse * (1.f / c.mass);
	c.oldAngle += Engine::crossProduct(circleToContactPoint, impulse) * (1.f / c.momentOfInertia) * (180.f / 3.141592f);

	return;
}

float Engine::dotProduct(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return ((v1.x * v2.x) + (v1.y * v2.y));
}

float Engine::crossProduct(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

sf::Vector2f Engine::normalise(const sf::Vector2f& v)
{
	return sf::Vector2f(v / pow(v.x * v.x + v.y * v.y , 0.5f));
}

float Engine::findDistance(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	return pow(((v2.x - v1.x) * (v2.x - v1.x)) + ((v2.y - v1.y) * (v2.y - v1.y)), 0.5f);
}

sf::Vector2f Engine::perpendicular(const sf::Vector2f& v)
{
	return { v.y, -v.x };
}

bool Engine::nearlyEqual(const float a1, const float a2)
{
	float verySmallAmount = 0.1f;
	return abs(a1 - a2) < verySmallAmount;
}

bool Engine::nearlyEqual(const sf::Vector2f& v1, const sf::Vector2f& v2)
{
	float verySmallAmount = 0.1f;
	return Engine::findDistance(v1, v2) < verySmallAmount;
}

std::vector<sf::Vector2f> Engine::getPolygonVertexPositions(ConvexPolygon& polygon)
{
	int vertexCount = polygon.body.getPointCount();
	std::vector<sf::Vector2f> vertexPositions(vertexCount);

	for (int i = 0; i < vertexCount; ++i) {
		vertexPositions[i] = polygon.getVertexPosition(i);
		//std::cout << polygon.getVertexPosition(i).x << " " << polygon.getVertexPosition(i).y << std::endl;
	}
	return vertexPositions;
}

std::vector<sf::Vector2f> Engine::getBarrierVertexPositions(rectBarrier& barrier)
{
	int vertexCount = barrier.body.getPointCount();
	std::vector<sf::Vector2f> vertexPositions(vertexCount);

	for (int i = 0; i < vertexCount; ++i) {
		vertexPositions[i] = barrier.getVertexPosition(i);
	}
	return vertexPositions;
}

void Engine::update(float dt)
{
	timeStep = dt;

	pollEvents();

	/*
	if (counter > 20) {
		if(entitiesSpawned < 40) {
			//addSquare();
		}
		counter = 0;
	}
	counter++;

	*/
	framerate = 1.f / clock.getElapsedTime().asSeconds();
	clock.restart();

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

	for (rectBarrier& barrier : rectBarriers) {
		barrier.renderBarrier(*window);
	}

	for (sf::RectangleShape object : Engine::objectsToDraw) {
		//std::cout << object.getPosition().x << std::endl;
		window->draw(object);
	}

	/*
	if (counter > 120) {
		objectsToDraw.clear();
		counter = 0;
	}
	counter++;
	*/

	window->draw(fpstext);
	window->draw(entitiesSpawnedText);

	window->display();
	objectsToDraw.clear();
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
	diameter = size * 2;
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
	diameter = size * 2;
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

void Circle::getBoundingBox(float& maxX, float& minX, float& maxY, float& minY)
{
	maxX = this->currentPosition.x + this->size;
	minX = this->currentPosition.x - this->size;
	maxY = this->currentPosition.y + this->size;
	minY = this->currentPosition.y - this->size;
}

void Circle::updatePosition()
{
	body.setPosition(currentPosition);
	body.setRotation(currentAngle);
}

void Circle::detectBarrierCollision(rectBarrier& b)
{
	Engine::circleBarrierDetection(*this, b);
}

void Circle::detectEntityCollision(Entity& e)
{
	e.detectCircleCollision(*this);
}

void Circle::detectCircleCollision(Circle& c)
{
	Engine::circleCircleDetection(*this, c);
}

void Circle::detectPolygonCollision(ConvexPolygon& polygon)
{
	Engine::circlePolygonDetection(*this, polygon);
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
	diameter = pow(size * size + size * size, 0.5f);
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
	oldAngle = -0.4f;

	mass = inputMass;
	size = inputSize;
	diameter = pow(size * size + size * size, 0.5f);
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

void Square::getBoundingBox(float& maxX, float& minX, float& maxY, float& minY)
{
	maxX = this->currentPosition.x + diameter * 0.5f;
	minX = this->currentPosition.x - diameter * 0.5f;
	maxY = this->currentPosition.y + diameter * 0.5f;
	minY = this->currentPosition.y - diameter * 0.5f;
}

void Square::updatePosition()
{
	body.setPosition(currentPosition);
	body.setRotation(currentAngle);
}

void Square::detectBarrierCollision(rectBarrier& b)
{
	Engine::polygonBarrierDetection(*this, b);
}

void Square::detectEntityCollision(Entity& e)
{
	e.detectPolygonCollision(*this);
}

void Square::detectCircleCollision(Circle& c)
{
	Engine::circlePolygonDetection(c, *this);
}

void Square::detectPolygonCollision(ConvexPolygon& polygon)
{
	Engine::polygonPolygonDetection(*this, polygon);
}

sf::Vector2f Square::getVertexPosition(int vertex)
{
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex);
	float angle = (3.14159 / 180.f) * currentAngle;

	float sinAngle = std::sin(angle);
	float cosAngle = std::cos(angle);

	float x = (nonRotatedPosition.x * cosAngle) - (nonRotatedPosition.y * sinAngle);
	float y = (nonRotatedPosition.x * sinAngle) + (nonRotatedPosition.y * cosAngle);

	return { x + currentPosition.x, y + currentPosition.y };
}

void Square::renderEntity(sf::RenderWindow& target)
{
	target.draw(body);
}

Triangle::Triangle()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.f;

	mass = 1.f;
	size = 10.f;
	diameter = root3 * 0.5f * size;
	momentOfInertia = mass * size * size / 12.f;

	body.setPointCount(3);
	body.setPoint(0, { 0.f, -(diameter * 2.f / 3.f) });
	body.setPoint(1, { (size / 2.f), (diameter / 3.f) });
	body.setPoint(2, { -(size / 2.f), (diameter / 3.f) });

	color = sf::Color::Green;
	body.setFillColor(color);
	resCoeff = 0.8f;

	force = { 0.f, 0.f };
}

Triangle::Triangle(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = -0.4f;

	mass = inputMass;
	size = inputSize;
	diameter = root3 * 0.5f * size;
	momentOfInertia = mass * size * size / 12.f;

	//Points for a triangle are defined starting in top left corner and moving clockwise.
	//Co-ordinates of points are relative to the position of the body

	body.setPointCount(3);
	body.setPoint(0, { 0.f, -(diameter * 2.f / 3.f) });
	body.setPoint(1, { (size / 2.f), (diameter / 3.f) });
	body.setPoint(2, { -(size / 2.f), (diameter / 3.f) });

	color = inputColor;
	body.setFillColor(color);
	resCoeff = 0.8f;

	force = { 0.f, 0.f };
	/*
	body.setOutlineThickness(1);
	body.setOutlineColor(sf::Color(250, 150, 100));
	*/
}

Triangle::~Triangle()
{
}

void Triangle::getBoundingBox(float& maxX, float& minX, float& maxY, float& minY)
{
	maxX = this->currentPosition.x + diameter * 0.667f;
	minX = this->currentPosition.x - diameter * 0.667f;
	maxY = this->currentPosition.y + diameter * 0.667f;
	minY = this->currentPosition.y - diameter * 0.667f;
}

void Triangle::updatePosition()
{
	body.setPosition(currentPosition);
	body.setRotation(currentAngle);
}

void Triangle::detectBarrierCollision(rectBarrier& b)
{
	Engine::polygonBarrierDetection(*this, b);
}

void Triangle::detectEntityCollision(Entity& e)
{
	e.detectPolygonCollision(*this);
}

void Triangle::detectCircleCollision(Circle& c)
{
	Engine::circlePolygonDetection(c, *this);
}

void Triangle::detectPolygonCollision(ConvexPolygon& polygon)
{
	Engine::polygonPolygonDetection(*this, polygon);
}

sf::Vector2f Triangle::getVertexPosition(int vertex)
{
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex);
	float angle = (3.14159 / 180.f) * currentAngle;

	float sinAngle = std::sin(angle);
	float cosAngle = std::cos(angle);

	float x = (nonRotatedPosition.x * cosAngle) - (nonRotatedPosition.y * sinAngle);
	float y = (nonRotatedPosition.x * sinAngle) + (nonRotatedPosition.y * cosAngle);

	return { x + currentPosition.x, y + currentPosition.y };
}

void Triangle::renderEntity(sf::RenderWindow& target)
{
	target.draw(body);
}

rectBarrier::rectBarrier()
{
	color = sf::Color::White;
	body.setFillColor(color);
	size = sf::Vector2f(10.f, 10.f);
	body.setSize(size);

	body.setOrigin({ size.x * 0.5f, size.y * 0.5f });

	position = sf::Vector2f(0.f, 0.f);
	body.setPosition(position);

	vertexPositions = Engine::getBarrierVertexPositions(*this);
}

rectBarrier::rectBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition)
{
	color = inputColor;
	body.setFillColor(color);
	size = inputSize;
	body.setSize(size);

	position = inputPosition;
	body.setPosition(position);

	body.setOrigin({ size.x * 0.5f, size.y * 0.5f });

	vertexPositions = Engine::getBarrierVertexPositions(*this);
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

	body.setOrigin({ size.x * 0.5f, size.y * 0.5f });

	vertexPositions = Engine::getBarrierVertexPositions(*this);
}

sf::Vector2f rectBarrier::getVertexPosition(int vertex)
{
	//For rectangle shape, regardless of setOrigin, getPoint for 0th vertex will always give 0,0.
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex) - size * 0.5f;
	float angle = (3.14159 / 180.f) * body.getRotation();

	float x = (nonRotatedPosition.x * std::cos(angle)) - (nonRotatedPosition.y * std::sin(angle));
	float y = (nonRotatedPosition.x * std::sin(angle)) + (nonRotatedPosition.y * std::cos(angle));

	return { x + position.x, y + position.y};
}

void rectBarrier::detectPolygonCollision(ConvexPolygon& polygon)
{
	Engine::polygonBarrierDetection(polygon, *this);
}

void rectBarrier::renderBarrier(sf::RenderWindow& target)
{
	//body.rotate(0.4f);
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
