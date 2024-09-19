#include "Engine.h"

int windowWidth = 1200;
int windowHeight = 800;
float f_windowWidth = 1200.f;
float f_windowHeight = 800.f;

int cell_size = 20;
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
	Entities.emplace_back(new Circle(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 10.f, sf::Color::Green));
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

	E.oldPosition = E.currentPosition;

	applyGravity(E);
	E.currentAcceleration = E.force / E.mass;

	E.currentPosition += displacement + (E.currentAcceleration * dt * dt);

	E.force = { 0.f, 0.f };
	//std::cout << "Position: " << E.currentPosition.x << " , " << E.currentPosition.y<<std::endl;
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

void Engine::circlePolygonCollision(Circle& c, ConvexPolygon& convexPolygon)
{
	/*
	sf::Vector2f circlePosition = c.currentPosition;
	sf::Vector2f pointOfContact;
	float minimumDistance = std::numeric_limits<float>::max();

	int polygonVertexCount = convexPolygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f vertex1Position = convexPolygon.body.getPoint(i) + convexPolygon.currentPosition;
		sf::Vector2f vertex2Position = convexPolygon.body.getPoint((i + 1) % polygonVertexCount) + convexPolygon.currentPosition;

		sf::Vector2f closestPoint = Engine::closestPointOnSegmentToCircle(c, vertex1Position, vertex2Position);
		float distance = Engine::findDistance(c.currentPosition, closestPoint);

		if (distance < minimumDistance) {
			minimumDistance = distance;
			pointOfContact = closestPoint;
		}
	}

	if (minimumDistance > c.size) {
		//Not touching
		//std::cout << "not touching" << std::endl;
		return;
	}

	float penetrationDepth = c.size - minimumDistance;
	sf::Vector2f normal = { (c.currentPosition.x - pointOfContact.x) / minimumDistance, (c.currentPosition.y - pointOfContact.y) / minimumDistance };

	std::cout << penetrationDepth << std::endl;

	//Check the normal is facing in the correct direction
	if (dotProduct(convexPolygon.currentPosition - c.currentPosition, normal) < 0.f) {
		normal = -normal;
	}

	circlePolygonResolution(c, convexPolygon, penetrationDepth + 0.001, normal);

	*/



	
	//SAT implementation to find out whether a polygon and a circle have collided
	//Draw an axis which is the normal of the polygon.
	//Project every vertex onto that axis and two points on the circle.
	//If there is any gap, the polygons are NOT touching

	//TODO: Tidy this up.

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
		//std::cout << convexPolygon.body.getPoint(i).x << " " << convexPolygon.body.getPoint(i).y << std::endl;

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

	//Find the axis point from the center of the circle to the closest polygon vertex.

	sf::Vector2f axis;
	float depth;

	float shortestDistance = std::numeric_limits<float>::max();

	for (int i = 0; i < polygonVertexCount; ++i) {
		float distance = Engine::findDistance(convexPolygon.getVertexPosition(i), c.currentPosition);
		if (distance < shortestDistance) {
			shortestDistance = distance;
			axis = c.currentPosition - convexPolygon.getVertexPosition(i); //Should this be reversed?
		}
	}

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

	//Check the normal is facing in the correct direction
	if (dotProduct(convexPolygon.currentPosition - c.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	//std::cout << minimumDepth << std::endl;
	circlePolygonResolution(c, convexPolygon, minimumDepth, minimumAxis);


	/*
	sf::Vector2f normal;
	float depth = std::numeric_limits<float>::max();

	int polygonVertexCount = convexPolygon.body.getPointCount();
	sf::Vector2f polygonPosition = convexPolygon.currentPosition;

	for (int i = 0; i < polygonVertexCount; ++i) {

		sf::Vector2f vertex1 = convexPolygon.body.getPoint(i) + polygonPosition;
		sf::Vector2f vertex2 = convexPolygon.body.getPoint((i + 1) % polygonVertexCount) + polygonPosition;

		sf::Vector2f edge = vertex2 - vertex1;
		sf::Vector2f axis = { edge.y, -edge.x };
		//Normalise axis
		axis = normalise(axis);

		//Now project every vertex onto this axis, keeping track of the minimum and the maximum for each polygon.
		float min1 = std::numeric_limits<float>::max();
		float max1 = std::numeric_limits<float>::min();

		float min2 = min1;
		float max2 = max1;

		//Projecting the polygon onto the current axis.
		for (int j = 0; j < polygonVertexCount; ++j) {
			float projection = dotProduct(convexPolygon.body.getPoint(j) + polygonPosition, axis);

			if (projection < min1) { min1 = projection; }
			if (projection > max1) { max1 = projection; }
		}
		//Projecting the circle onto the current axis.
		sf::Vector2f point1 = c.currentPosition + axis * c.size;
		sf::Vector2f point2 = c.currentPosition - axis * c.size;
		
		min2 = dotProduct(point1, axis);
		max2 = dotProduct(point2, axis);

		if (min2 > max2) {
			float temp = min2;
			min2 = max2;
			max2 = temp;
			
		}

		//Check for overlap in the projections.
		if (min1 >= max2 || min2 >= max1) {
			return;
		}

		//Objects are touching
		//Define axisDepth, the depth projected onto the current axis. There may be more than one and possibly
		//smaller therefore need to check and keep the smallest value in depth.
		float axisDepth;
		if ((max2 - min1) < (max1 - min2)) { axisDepth = max2 - min1; }
		else { axisDepth = max1 - min2; }

		//std::cout << "axis depth: " << axisDepth << std::endl;

		if (axisDepth < depth) {
			//std::cout << "executed" << std::endl;
			depth = axisDepth;
			normal = axis;
		}
	}

	//Repeat for the circle
	//Find the polygon vertex position closest to the circle
	sf::Vector2f closestPoint;
	float minDistance = std::numeric_limits<float>::max();
	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f vertex = convexPolygon.body.getPoint(i) + polygonPosition;
		float distance = Engine::findDistance(vertex, c.currentPosition);

		if (distance < minDistance) {
			minDistance = distance;
			closestPoint = vertex;
		}
	}

	//Find new axis to test against
	sf::Vector2f axis = normalise(closestPoint - c.currentPosition);

	//Project onto the axis
	float min1 = std::numeric_limits<float>::max();
	float max1 = std::numeric_limits<float>::min();

	float min2 = min1;
	float max2 = max1;

	//Project the polygon
	for (int j = 0; j < polygonVertexCount; ++j) {
		float projection = dotProduct(convexPolygon.body.getPoint(j) + polygonPosition, axis);

		if (projection < min1) { min1 = projection; }
		if (projection > max1) { max1 = projection; }
	}

	//Project the circle
	sf::Vector2f point1 = c.currentPosition + axis * c.size;
	sf::Vector2f point2 = c.currentPosition - axis * c.size;

	min2 = dotProduct(point1, axis);
	max2 = dotProduct(point2, axis);

	if (min2 > max2) {
		float temp = min2;
		min2 = max2;
		max2 = temp;
	}

	//Check for overlap in the projections.
	if (min1 >= max2 || min2 >= max1) {
		return;
	}

	//Objects are touching
	//Define axisDepth, the depth projected onto the current axis. There may be more than one and possibly
	//smaller therefore need to check and keep the smallest value in depth.
	float axisDepth;
	if ((max2 - min1) < (max1 - min2)) { axisDepth = max2 - min1; }
	else { axisDepth = max1 - min2; }

	//std::cout << "axis depth: " << axisDepth << std::endl;

	if (axisDepth < depth) {
		//std::cout << "executed" << std::endl;
		depth = axisDepth;
		normal = axis;
	}

	//Check to see if the normal is facing in the correct direction for the two polygons.
	if (dotProduct(polygonPosition - c.currentPosition, normal) < 0.f) {
		normal = -normal;
	}

	//std::cout << "Depth: " << depth << " Normal: " << normal.x << " , " << normal.y << std::endl;
	circlePolygonResolution(c, convexPolygon, depth, normal);
	*/
}

void Engine::circlePolygonResolution(Circle& c, ConvexPolygon& convexPolygon, float depth, sf::Vector2f axis)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	const sf::Vector2f circleVelocity = (c.currentPosition - c.oldPosition);
	const sf::Vector2f polygonVelocity = (convexPolygon.currentPosition - convexPolygon.oldPosition);

	float normRelativeVelocity = Engine::dotProduct(polygonVelocity - circleVelocity, axis);

	//Are the polygons moving away from each other? If so, don't do anything.
	if (normRelativeVelocity > 0) {
		return;
	}

	float j = -(1 + c.resCoeff) * normRelativeVelocity / ((1 / c.mass) + (1 / convexPolygon.mass));

	//Separate the two polygons. It should be noted axis MUST be normalised here.
	c.currentPosition -= (0.5f * depth * axis);
	convexPolygon.currentPosition += (0.5f * depth * axis);

	c.oldPosition += (((j / (1 / c.mass)) * axis) - 0.5f * depth * axis);
	convexPolygon.oldPosition -= (((j / (1 / convexPolygon.mass)) * axis) - 0.5f * depth * axis);

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

void Engine::polygonPolygonCollision(ConvexPolygon& convexPolygon1, ConvexPolygon& convexPolygon2)
{
	//SAT implementation to find out whether two polygons have collided
	//Draw an axis which is the normal of a polygon.
	//Project every vertex onto that axis.
	//If there is any gap, the polygons are NOT touching

	//TODO: Tidy this up.

	//Define normal and depth. These will be the axis and depth associated with the smallest 
	//displacement possible in order to push the two objects aside.

	float minimumDepth = std::numeric_limits<float>::max();
	sf::Vector2f minimumAxis;

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
			return;
		}
		//Projections are overlapping
		if (max1 - min2 < max2 - min1) {
			depth = max1 - min2;
		}
		else {
			depth = max2 - min1;
		}
		
		if (depth < minimumDepth) {
			minimumDepth = depth;
			minimumAxis = axis;
		}
	}


	for (int i = 0; i < polygon2VertexCount; ++i) {

		sf::Vector2f axis;
		float depth;

		sf::Vector2f vertex1Position = convexPolygon2.getVertexPosition(i);
		sf::Vector2f vertex2Position = convexPolygon2.getVertexPosition((i + 1) % polygon2VertexCount);

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
			return;
		}
		//Projections are overlapping
		if (max1 - min2 < max2 - min1) {
			depth = max1 - min2;
		}
		else {
			depth = max2 - min1;
		}

		if (depth < minimumDepth) {
			minimumDepth = depth;
			minimumAxis = axis;
		}
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (dotProduct(convexPolygon2.currentPosition - convexPolygon1.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	//std::cout << "Touching" << std::endl;
	polygonPolygonResolution(convexPolygon1, convexPolygon2, minimumDepth, minimumAxis);


	/*
	sf::Vector2f normal;
	float depth = std::numeric_limits<float>::max();

	int polygon1VertexCount = convexPolygon1.body.getPointCount();
	int polygon2VertexCount = convexPolygon2.body.getPointCount();
	sf::Vector2f polygon1Position = convexPolygon1.currentPosition;
	sf::Vector2f polygon2Position = convexPolygon2.currentPosition;


	for (int i = 0; i < polygon1VertexCount; ++i) {

		sf::Vector2f vertex1 = convexPolygon1.body.getPoint(i) + polygon1Position;
		sf::Vector2f vertex2 = convexPolygon1.body.getPoint((i + 1) % polygon1VertexCount) + polygon1Position;

		sf::Vector2f edge = vertex2 - vertex1;
		sf::Vector2f axis = { edge.y, -edge.x };
		//Normalise axis
		axis = normalise(axis);

		//Now project every vertex onto this axis, keeping track of the minimum and the maximum for each polygon.
		float min1 = std::numeric_limits<float>::max();
		float max1 = std::numeric_limits<float>::min();

		float min2 = min1;
		float max2 = max1;

		for (int j = 0; j < polygon1VertexCount; ++j) {
			float projection = dotProduct(convexPolygon1.body.getPoint(j) + polygon1Position, axis);

			if (projection < min1) { min1 = projection; }
			if (projection > max1) { max1 = projection; }
		}
		for (int j = 0; j < polygon2VertexCount; ++j) {
			float projection = dotProduct(convexPolygon2.body.getPoint(j) + polygon2Position, axis);

			if (projection < min2) { min2 = projection; }
			if (projection > max2) { max2 = projection; }
		}

		if (min1 >= max2 || min2 >= max1) {
			return;
		}
		
		//Objects are touching
		//Define axisDepth, the depth projected onto the current axis. There may be more than one and possibly
		//smaller therefore need to check and keep the smallest value in depth.
		float axisDepth;
		if ((max2 - min1) < (max1 - min2)) { axisDepth = max2 - min1; }
		else { axisDepth = max1 - min2; }

		//std::cout << "axis depth: " << axisDepth << std::endl;

		if (axisDepth < depth) {
			//std::cout << "executed" << std::endl;
			depth = axisDepth;
			normal = axis;
		}
	}

	//Repeat for convexPolygon2
	for (int i = 0; i < polygon2VertexCount; ++i) {

		sf::Vector2f vertex1 = convexPolygon2.body.getPoint(i) + polygon2Position;
		sf::Vector2f vertex2 = convexPolygon2.body.getPoint((i + 1) % polygon2VertexCount) + polygon2Position;

		sf::Vector2f edge = vertex2 - vertex1;
		sf::Vector2f axis = { edge.y, -edge.x };

		//Now project every vertex onto this axis, keeping track of the minimum and the maximum for each polygon.
		float min1 = std::numeric_limits<float>::max();
		float max1 = std::numeric_limits<float>::min();

		float min2 = min1;
		float max2 = max1;

		for (int j = 0; j < polygon1VertexCount; ++j) {
			float projection = dotProduct(convexPolygon1.body.getPoint(j) + polygon1Position, axis);

			if (projection < min1) { min1 = projection; }
			if (projection > max1) { max1 = projection; }
		}
		for (int j = 0; j < polygon2VertexCount; ++j) {
			float projection = dotProduct(convexPolygon2.body.getPoint(j) + polygon2Position, axis);

			if (projection < min2) { min2 = projection; }
			if (projection > max2) { max2 = projection; }
		}

		if (min1 >= max2 || min2 >= max1) {
			return;
		}

		float axisDepth;
		if ((max2 - min1) < (max1 - min2)) { axisDepth = max2 - min1; }
		else { axisDepth = max1 - min2; }

		if (axisDepth < depth) {
			//std::cout << "executed" << std::endl;
			depth = axisDepth;
			normal = axis;
		}
	}

	//Check to see if the normal is facing in the correct direction for the two polygons.
	if (dotProduct(convexPolygon2.currentPosition - convexPolygon1.currentPosition, normal) < 0.f) {
		normal = -normal;
	}
	
	//std::cout << "Depth: " << depth << " Normal: " << normal.x << " , " << normal.y << std::endl;
	polygonPolygonResolution(convexPolygon1, convexPolygon2, depth, normal);
	*/
}

void Engine::polygonPolygonResolution(ConvexPolygon& convexPolygon1, ConvexPolygon& convexPolygon2, float depth, sf::Vector2f axis)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	const sf::Vector2f polygon1Velocity = (convexPolygon1.currentPosition - convexPolygon1.oldPosition);
	const sf::Vector2f polygon2Velocity = (convexPolygon2.currentPosition - convexPolygon2.oldPosition);

	float normRelativeVelocity = Engine::dotProduct(polygon2Velocity - polygon1Velocity, axis);

	//Are the polygons moving away from each other? If so, don't do anything.
	if (normRelativeVelocity > 0) {
		return;
	}

	float j = -(1 + convexPolygon1.resCoeff) * normRelativeVelocity / ((1 / convexPolygon1.mass) + (1 / convexPolygon2.mass));

	//Separate the two polygons. It should be noted axis MUST be normalised here.
	convexPolygon1.currentPosition -= (0.5f * depth * axis);
	convexPolygon2.currentPosition += (0.5f * depth * axis);
	
	convexPolygon1.oldPosition += (((j / (1 / convexPolygon1.mass)) * axis) - 0.5f * depth * axis);
	convexPolygon2.oldPosition -= (((j / (1 / convexPolygon2.mass)) * axis) - 0.5f * depth * axis);

	return;
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
	mass = 1;
	size = 5.f;
	body.setRadius(size);
	color = sf::Color::White;
	body.setFillColor(color);
	resCoeff = 0.75f;

	body.setOrigin(size, size);
	force = { 0.f, 0.f };
}

Circle::Circle(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;
	mass = inputMass;
	size = inputSize;
	body.setRadius(size);
	color = inputColor;
	body.setFillColor(color);
	resCoeff = 0.75f;

	body.setOrigin(size, size);
	force = { 0.f, 0.f };
	body.setRadius(size);
}

Circle::~Circle()
{

}

void Circle::updatePosition()
{
	body.setPosition(currentPosition);
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
	//std::cout << "This is being executed" << std::endl;
	//grid = e, E = this

	/*
	sf::Vector2f separation = c.currentPosition - this->currentPosition;
	float separation_magnitude = pow((separation.x * separation.x) + (separation.y * separation.y), 0.5f);

	if (separation_magnitude < this->size + c.size) {

		//Resolve collision between the entities
		sf::Vector2f norm = separation / separation_magnitude;

		norm = sf::Vector2f(0.5 * (this->size + c.size - separation_magnitude) * norm.x,
			0.5 * (this->size + c.size - separation_magnitude) * norm.y);

		this->currentPosition -= this->resCoeff * norm;

		c.currentPosition += this->resCoeff * norm;
	}
	*/
	sf::Vector2f separation = c.currentPosition - this->currentPosition;
	float separation_magnitude = pow((separation.x * separation.x) + (separation.y * separation.y), 0.5f);

	if (separation_magnitude < this->size + c.size) {

		//Resolve collision between the circles
		const sf::Vector2f norm = separation / separation_magnitude;

		const sf::Vector2f vel_c = c.currentPosition - c.oldPosition;
		const sf::Vector2f vel_this = this->currentPosition - this->oldPosition;

		const float norm_vel_relative = Engine::dotProduct(vel_c - vel_this, norm);

		if (norm_vel_relative > 0) {
			return;
		}

		sf::Vector2f overLapCorrection = 0.5f * (this->size + c.size - separation_magnitude) * norm;

		this->currentPosition -= overLapCorrection;
		c.currentPosition += overLapCorrection;

		float j = -(1 + this->resCoeff) * norm_vel_relative / ((1 / this->mass) + (1 / c.mass));

		this->oldPosition += ((j / (1 / this->mass)) * norm - overLapCorrection);
		c.oldPosition -= ((j / (1 / c.mass)) * norm - overLapCorrection);
	}
}

void Circle::detectSquareCollision(Square& s)
{
	Engine::circlePolygonCollision(*this, s);
}

void Circle::renderEntity(sf::RenderWindow& target)
{
	target.draw(body);
}

Square::Square()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;
	mass = 1;
	size = 10.f;

	body.setPointCount(4);
	body.setPoint(0, { 0.f, 0.f });
	body.setPoint(1, sf::Vector2f(size, 0.f));
	body.setPoint(2, sf::Vector2f(size, size));
	body.setPoint(3, sf::Vector2f(0.f, size));

	color = sf::Color::Green;
	body.setFillColor(color);
	resCoeff = 0.75f;

	force = { 0.f, 0.f };
}

Square::Square(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;
	mass = inputMass;
	size = inputSize;

	//Points for a square are defined starting in top left corner and moving clockwise.
	//Co-ordinates of points are relative to the position of the body

	body.setPointCount(4);
	body.setPoint(0, { -(size / 2.f), -(size / 2.f)});
	body.setPoint(1, { (size / 2.f), -(size / 2.f) });
	body.setPoint(2, { (size / 2.f), (size / 2.f) });
	body.setPoint(3, { -(size / 2.f), (size / 2.f) });
	

	/*
	body.setPointCount(4);
	body.setPoint(0, { 0.f, 0.f });
	body.setPoint(1, sf::Vector2f(size, 0.f));
	body.setPoint(2, sf::Vector2f(size, size));
	body.setPoint(3, sf::Vector2f(0.f, size));
	*/

	color = inputColor;
	body.setFillColor(color);
	resCoeff = 0.75f;

	force = { 0.f, 0.f };
}

Square::~Square()
{
}

void Square::updatePosition()
{
	body.setPosition(currentPosition);
	body.rotate(1.f);
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
	Engine::circlePolygonCollision(c, *this);
}

void Square::detectSquareCollision(Square& s)
{
	Engine::polygonPolygonCollision(*this, s);
}

sf::Vector2f Square::getVertexPosition(int vertex)
{
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex);
	float angle = (3.14159 / 180.f) * body.getRotation();

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
