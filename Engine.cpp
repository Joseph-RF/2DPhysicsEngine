/*
* Note that very small acceleration doesn't work due to floating point precision.
* Bug? Feature? It looks like circles collide with squares, the circle loses
* all momentum perpendicular to the edge of the circle it collided with.
*/

#include "Engine.h"

int windowWidth = 1200;
int windowHeight = 800;
float f_windowWidth = 1200.f;
float f_windowHeight = 800.f;

int cell_size = 40;
int cell_number_x = windowWidth / cell_size;
int cell_number_y = windowHeight / cell_size;

extern sf::Vector2f lowerBarrier_position = { f_windowWidth * 0.5f , f_windowHeight * 0.95f };
extern sf::Vector2f upperBarrier_position = { f_windowWidth * 0.5f , 10.f };
extern sf::Vector2f rightBarrier_position = { f_windowWidth - 10.f , f_windowHeight * 0.5f};
extern sf::Vector2f leftBarrier_position = { 10.f, f_windowHeight * 0.5f };

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

	shapeButtonsText.setFont(font);
	shapeButtonsText.setCharacterSize(24);
	shapeButtonsText.setFillColor(sf::Color::White);
	shapeButtonsText.setPosition(50.f, 30.f);
	shapeButtonsText.setString("F1: Circle    F2: Square    F3: Triangle    F4: Hexagon");
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
	Entities.emplace_back(new Circle(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 10.f, generateVividColour()));
	entitiesSpawned++;
}

void Engine::addSquare()
{
	Entities.emplace_back(new Square(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 20.f, generateVividColour()));
	entitiesSpawned++;
}

void Engine::addTriangle()
{
	Entities.emplace_back(new Triangle(sf::Vector2f(rand() % 1000 + 100, 100.f), 1.f, 20.f, generateVividColour()));
	entitiesSpawned++;
}

void Engine::addHexagon()
{
	Entities.emplace_back(new Hexagon(sf::Vector2f(rand() % 1000 + 100, 100.f), 1.f, 15.f, generateVividColour()));
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
			} else if (e.key.code == sf::Keyboard::F1) {
				addCircle();
			} else if (e.key.code == sf::Keyboard::F2) {
				addSquare();
			} else if (e.key.code == sf::Keyboard::F3) {
				addTriangle();
			} else if (e.key.code == sf::Keyboard::F4) {
				addHexagon();
			} else if (e.key.code == sf::Keyboard::F5) {
				addSpring();
			} else if (e.key.code == sf::Keyboard::F6) {
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

	const float norm_vel_relative = VectorMath::dotProduct(vel_c2 - vel_c1, axis);

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
		sf::Vector2f tempVector = VectorMath::closestPointOnLineSegment(c.currentPosition, polygon.getVertexPosition(i),
																	polygon.getVertexPosition((i + 1) % polygonVertexCount));
		float tempDistance = VectorMath::findDistance(c.currentPosition, tempVector);

		if (tempDistance < c.size) {
			sf::Vector2f axis = tempVector - c.currentPosition;
			axis = VectorMath::normalise(axis);

			//Check the normal is facing in the correct direction
			if (VectorMath::dotProduct(polygon.currentPosition - c.currentPosition, axis) < 0.f) {
				axis = {-axis.x, -axis.y};
			}
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
	v_A -= (c.currentAngle - c.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(r_PA);

	sf::Vector2f v_B = polygon.currentPosition - polygon.oldPosition;
	v_B -= (polygon.currentAngle - polygon.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(contactPointOnPolygon);

	float contactVelocity_mag = VectorMath::dotProduct(v_B - v_A, axis);

	if (contactVelocity_mag > 0.f) {
		return;
	}

	float r_PA_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(r_PA), axis);
	float r_PB_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(contactPointOnPolygon), axis);

	float denom = (1.f / c.mass) + (1.f / polygon.mass) +
		(r_PA_PerpDotN * r_PA_PerpDotN) * (1.f / c.momentOfInertia) +
		(r_PB_PerpDotN * r_PB_PerpDotN) * (1.f / polygon.momentOfInertia);

	//Modify this to find the lower of the two restitution coefficients
	float restitutionCoefficient = (c.resCoeff > polygon.resCoeff ? polygon.resCoeff : c.resCoeff);

	float j = -(1.f + restitutionCoefficient) * contactVelocity_mag;
	j /= denom;

	sf::Vector2f impulse = j * axis;

	c.oldPosition += impulse * (1.f / c.mass);
	c.oldAngle += VectorMath::crossProduct(r_PA, impulse) * (1.f / c.momentOfInertia);

	polygon.oldPosition -= impulse * (1.f / polygon.mass);
	polygon.oldAngle -= VectorMath::crossProduct(contactPointOnPolygon, impulse) * (1.f / polygon.momentOfInertia);
}

void Engine::polygonPolygonDetection(ConvexPolygon& polygonA, ConvexPolygon& polygonB)
{
	if (!Engine::checkBoundingBox(polygonA, polygonB)) {
		return;
	}

	float minimumDepth = VectorMath::upperLimit;
	sf::Vector2f minimumAxis = {-1.f, -1.f};
	
	std::vector<sf::Vector2f> polygonAVertices = Engine::getPolygonVertexPositions(polygonA);
	std::vector<sf::Vector2f> polygonBVertices = Engine::getPolygonVertexPositions(polygonB);

	if ((projectionSAT(polygonAVertices, polygonBVertices, minimumDepth, minimumAxis)) ||
		(projectionSAT(polygonBVertices, polygonAVertices, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (VectorMath::dotProduct(polygonB.currentPosition - polygonA.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}

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

		axis = VectorMath::perpendicular(vertex2Position - vertex1Position);
		axis = VectorMath::normalise(axis);

		//Project all the vertices of both polygons onto the axis. Find the max and min for both polygons
		float max1 = VectorMath::lowerLimit;
		float min1 = VectorMath::upperLimit;

		float max2 = VectorMath::lowerLimit;
		float min2 = VectorMath::upperLimit;

		for (int j = 0; j < polygonAVertexCount; ++j) {
			sf::Vector2f vertexPosition = polygonAVertices[j];
			float projection = VectorMath::dotProduct(vertexPosition, axis);

			if (projection > max1) {
				max1 = projection;
			}
			if (projection < min1) {
				min1 = projection;
			}
		}
		for (int j = 0; j < polygonBVertexCount; ++j) {
			sf::Vector2f vertexPosition = polygonBVertices[j];
			float projection = VectorMath::dotProduct(vertexPosition, axis);

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

void Engine::polygonPolygonResolution(ConvexPolygon& polygonA, ConvexPolygon& polygonB, float depth, sf::Vector2f axis)
{
	sf::Vector2f overlapCorrection = 0.5f * axis * depth;

	polygonA.currentPosition -= overlapCorrection;
	polygonA.oldPosition -= overlapCorrection;

	polygonB.currentPosition += overlapCorrection;
	polygonB.oldPosition += overlapCorrection;

	std::vector<sf::Vector2f> contactPoints(2);
	int contactCount = 0;

	std::vector<sf::Vector2f> polygonAVertices = polygonA.getAllVertices();
	std::vector<sf::Vector2f> polygonBVertices = polygonB.getAllVertices();

	Engine::findContactPoints(polygonAVertices, polygonBVertices, contactPoints[0], contactPoints[1], contactCount);

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
		v_A -= (polygonA.currentAngle - polygonA.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(r_PA);

		sf::Vector2f v_B = (polygonB.currentPosition - polygonB.oldPosition);
		v_B -= (polygonB.currentAngle - polygonB.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(r_PB);

		float contactVelocity_mag = VectorMath::dotProduct(v_B - v_A, axis);

		if (contactVelocity_mag > 0.f)
		{
			continue;
		}

		float r_PA_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(r_PA), axis);
		float r_PB_PerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(r_PB), axis);

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
		polygonA.oldAngle += VectorMath::crossProduct(r_PA, impulse) * (1.f / polygonA.momentOfInertia) * VectorMath::radToDegFactor;

		polygonB.oldPosition -= impulse * (1.f / polygonB.mass);
		polygonB.oldAngle -= VectorMath::crossProduct(r_PB, impulse) * (1.f / polygonB.momentOfInertia) * VectorMath::radToDegFactor;
	}

	return;
}

bool Engine::checkBoundingBox(
	Circle& circle,
	rectBarrier& barrier
)
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

bool Engine::checkBoundingBox(
	ConvexPolygon& polygon,
	rectBarrier& barrier
)
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

bool Engine::checkBoundingBox(
	Circle& circle,
	ConvexPolygon& polygon
)
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

bool Engine::checkBoundingBox(
	ConvexPolygon& polygonA,
	ConvexPolygon& polygonB
)
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

sf::Vector2f Engine::findContactPoint(
	Circle& c,
	ConvexPolygon& convexPolygon
)
{
	sf::Vector2f contactPoint;
	float distanceToContactPoint = VectorMath::upperLimit;

	int polygonVertexCount = convexPolygon.body.getPointCount();

	for (int i = 0; i < polygonVertexCount; ++i) {
		sf::Vector2f tempVector = VectorMath::closestPointOnLineSegment(c.currentPosition, convexPolygon.getVertexPosition(i),
																	convexPolygon.getVertexPosition((i + 1) % polygonVertexCount));
		float tempDistance = VectorMath::findDistance(c.currentPosition, tempVector);

		if (tempDistance < distanceToContactPoint) {
			distanceToContactPoint = tempDistance;
			contactPoint = tempVector;
		}
	}
	return contactPoint;
}

void Engine::findContactPoints(
	std::vector<sf::Vector2f>& shapeAVertices,
	std::vector<sf::Vector2f>& shapeBVertices,
	sf::Vector2f& contactPoint1,
	sf::Vector2f& contactPoint2,
	int& contactCount
)
{
	contactPoint1 = { -1.f, -1.f };
	contactPoint2 = { -1.f, -1.f };

	float minDist = VectorMath::upperLimit;
	int shapeAVertexCount = shapeAVertices.size();
	int shapeBVertexCount = shapeBVertices.size();

	for (int i = 0; i < shapeAVertexCount; i++)
	{
		sf::Vector2f point = shapeAVertices[i];

		for (int j = 0; j < shapeBVertexCount; j++)
		{
			sf::Vector2f v1 = shapeBVertices[j];
			sf::Vector2f v2 = shapeBVertices[(j + 1) % shapeBVertexCount];

			sf::Vector2f contactPoint = VectorMath::closestPointOnLineSegment(point, v1, v2);
			float dist = VectorMath::findDistance(contactPoint, point);

			if (VectorMath::nearlyEqual(dist, minDist))
			{
				if (!VectorMath::nearlyEqual(contactPoint, contactPoint1)) {
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

	for (int i = 0; i < shapeBVertexCount; i++)
	{
		sf::Vector2f point = shapeBVertices[i];

		for (int j = 0; j < shapeAVertexCount; j++)
		{
			sf::Vector2f v1 = shapeAVertices[j];
			sf::Vector2f v2 = shapeAVertices[(j + 1) % shapeAVertexCount];

			sf::Vector2f contactPoint = VectorMath::closestPointOnLineSegment(point, v1, v2);
			float dist = VectorMath::findDistance(contactPoint, point);

			if (VectorMath::nearlyEqual(dist, minDist))
			{
				if (!VectorMath::nearlyEqual(contactPoint, contactPoint1)) {
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

void Engine::polygonBarrierDetection(
	ConvexPolygon& polygon,
	rectBarrier& barrier
)
{
	if (!Engine::checkBoundingBox(polygon, barrier)) {
		return;
	}

	float minimumDepth = VectorMath::upperLimit;
	sf::Vector2f minimumAxis;

	sf::Vector2f barrierPosition = barrier.position;

	std::vector<sf::Vector2f> polygonVertices = Engine::getPolygonVertexPositions(polygon);

	if ((projectionSAT(polygonVertices, barrier.vertexPositions, minimumDepth, minimumAxis)) ||
		(projectionSAT(barrier.vertexPositions, polygonVertices, minimumDepth, minimumAxis))) {
		return;
	}

	//Polygons are colliding and the minimum distance in order to separate the two has been found.

	//Check the normal is facing in the correct direction
	if (VectorMath::dotProduct(barrierPosition - polygon.currentPosition, minimumAxis) < 0.f) {
		minimumAxis = -minimumAxis;
	}
	polygonBarrierResolution(polygon, barrier, minimumDepth, minimumAxis);
}

void Engine::polygonBarrierResolution(
	ConvexPolygon& polygon,
	rectBarrier& barrier,
	const float& depth,
	const sf::Vector2f& axis
)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	
	polygon.currentPosition -= (depth * axis);
	polygon.oldPosition -= (depth * axis);

	std::vector<sf::Vector2f> contactPoints(2);
	int contactCount = 0;

	std::vector<sf::Vector2f> polygonVertices = polygon.getAllVertices();
	std::vector<sf::Vector2f> barrierVertices = Engine::getBarrierVertexPositions(barrier);

	Engine::findContactPoints(polygonVertices, barrierVertices, contactPoints[0], contactPoints[1], contactCount);

	std::vector<sf::Vector2f> raList(2);
	std::vector<sf::Vector2f> impulseList(2);

	for (int i = 0; i < contactCount; i++)
	{
		sf::Vector2f polygonToContactPoint = contactPoints[i] - polygon.currentPosition;

		raList[i] = polygonToContactPoint;

		sf::Vector2f polygonVelocity = (polygon.currentPosition - polygon.oldPosition);

		polygonVelocity -= (polygon.currentAngle - polygon.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(polygonToContactPoint);

		float contactVelocityMag = VectorMath::dotProduct(-polygonVelocity, axis);

		if (contactVelocityMag > 0.f)
		{
			continue;
		}
		
		float raPerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(polygonToContactPoint), axis);

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
		polygon.oldAngle += VectorMath::crossProduct(ra, impulse) * (1.f / polygon.momentOfInertia) * VectorMath::radToDegFactor;
	}

	return;
}

void Engine::circleBarrierDetection(
	Circle& c,
	rectBarrier& b
)
{
	if (!Engine::checkBoundingBox(c, b)) {
		return;
	}

	sf::Vector2f closestPoint;
	int barrierVertexCount = b.body.getPointCount();

	for (int i = 0; i < barrierVertexCount; ++i) {
		sf::Vector2f tempVector = VectorMath::closestPointOnLineSegment(c.currentPosition, b.vertexPositions[i],
			b.vertexPositions[(i + 1) % barrierVertexCount]);
		float tempDistance = VectorMath::findDistance(c.currentPosition, tempVector);

		if (tempDistance < c.size) {
			sf::Vector2f axis = tempVector - c.currentPosition;
			axis = VectorMath::normalise(axis);

			//Check the normal is facing in the correct direction
			if (VectorMath::dotProduct(b.position - c.currentPosition, axis) < 0.f) {
				axis = { -axis.x, -axis.y };
			}
			Engine::circleBarrierResolution(c, b, (c.size - tempDistance), axis, tempVector - b.position);
		}
	}
}

void Engine::circleBarrierResolution(
	Circle& c,
	rectBarrier& b,
	const float& depth,
	const sf::Vector2f& axis,
	const sf::Vector2f& contactPointOnBarrier
)
{
	//Note will be leaving out the time factor for velocities since it cancels out in the end.
	//Denoting circle as A and barrier as B.
	c.currentPosition -= (depth * axis);
	c.oldPosition -= (depth * axis);

	sf::Vector2f circleToContactPoint = (contactPointOnBarrier + b.position) - c.currentPosition;

	sf::Vector2f circleVelocity = (c.currentPosition - c.oldPosition);
	circleVelocity -= (c.currentAngle - c.oldAngle) * VectorMath::degToRadFactor * VectorMath::perpendicular(circleToContactPoint);

	float contactVelocityMag = VectorMath::dotProduct(-circleVelocity, axis);

	if (contactVelocityMag > 0.f) {
		return;
	}

	float raPerpDotN = VectorMath::dotProduct(VectorMath::perpendicular(circleToContactPoint), axis);

	float denom = (1.f / c.mass) + 0.f +
		(raPerpDotN * raPerpDotN) * (1.f / c.momentOfInertia) +
		0.f;


	float j = -(1 + c.resCoeff) * contactVelocityMag;
	j /= denom;

	sf::Vector2f impulse = j * axis;

	c.oldPosition += impulse * (1.f / c.mass);
	c.oldAngle += VectorMath::crossProduct(circleToContactPoint, impulse) * (1.f / c.momentOfInertia) * VectorMath::radToDegFactor;

	return;
}

std::vector<sf::Vector2f> Engine::getPolygonVertexPositions(
	ConvexPolygon& polygon
)
{
	int vertexCount = polygon.body.getPointCount();
	std::vector<sf::Vector2f> vertexPositions(vertexCount);

	for (int i = 0; i < vertexCount; ++i) {
		vertexPositions[i] = polygon.getVertexPosition(i);
	}
	return vertexPositions;
}

std::vector<sf::Vector2f> Engine::getBarrierVertexPositions(
	rectBarrier& barrier
)
{
	int vertexCount = barrier.body.getPointCount();
	std::vector<sf::Vector2f> vertexPositions(vertexCount);

	for (int i = 0; i < vertexCount; ++i) {
		vertexPositions[i] = barrier.getVertexPosition(i);
	}
	return vertexPositions;
}

sf::Color Engine::generateVividColour()
{
	int hue = (rand() % 360);
	float saturation = (rand() % 25) + 75.f;
	saturation /= 100;
	float value = (rand() % 25) + 75.f;
	value /= 100;

	return hsvToRgb(hue, saturation, value);
}

sf::Color Engine::hsvToRgb(int h, float s, float v)
{
	float c = v * s;
	float f = ((h / 60 ) % 2) - 1.f;
	if (f < 0.f) { f = -f; }

	float x = c * (1.f - f);
	float m = v - c;

	float R_1 = 0.f;
	float G_1 = 0.f;
	float B_1 = 0.f;

	if (h < 180.f) {
		if (h < 120.f) {
			if (h < 60.f) {
				R_1 = c;
				G_1 = x;
			}
			else {
				R_1 = x;
				G_1 = c;
			}
		}
		else {
			G_1 = c;
			B_1 = x;
		}
	}
	else {
		if (h < 300.f) {
			if (h < 240.f) {
				G_1 = x;
				B_1 = c;
			}
			else {
				R_1 = x;
				B_1 = c;
			}
		}
		else {
			R_1 = c;
			B_1 = x;
		}
	}
	sf::Uint32 R = (R_1 + m) * 255;
	sf::Uint32 G = (G_1 + m) * 255;
	sf::Uint32 B = (B_1 + m) * 255;
	return sf::Color(R, G, B, 255);
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

	window->draw(fpstext);
	window->draw(entitiesSpawnedText);
	window->draw(shapeButtonsText);

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