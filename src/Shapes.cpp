#include "Shapes.h"

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
	Collisions::circleBarrierDetection(*this, b);
}

void Circle::detectEntityCollision(Entity& e)
{
	e.detectCircleCollision(*this);
}

void Circle::detectCircleCollision(Circle& c)
{
	Collisions::circleCircleDetection(*this, c);
}

void Circle::detectPolygonCollision(ConvexPolygon& polygon)
{
	Collisions::circlePolygonDetection(*this, polygon);
}

void Circle::renderEntity(sf::RenderWindow& target)
{
	target.draw(body);
}

sf::Vector2f ConvexPolygon::getVertexPosition(int vertex)
{
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex);
	float angle = VectorMath::degToRadFactor * currentAngle;

	float sinAngle = std::sin(angle);
	float cosAngle = std::cos(angle);

	float x = (nonRotatedPosition.x * cosAngle) - (nonRotatedPosition.y * sinAngle);
	float y = (nonRotatedPosition.x * sinAngle) + (nonRotatedPosition.y * cosAngle);

	return { x + currentPosition.x, y + currentPosition.y };
}

std::vector<sf::Vector2f> ConvexPolygon::getAllVertices()
{
	int vertexCount = this->body.getPointCount();
	std::vector<sf::Vector2f> vertices(vertexCount);

	for (int i = 0; i < vertexCount; ++i) {
		vertices[i] = this->getVertexPosition(i);
	}
	return vertices;
}

void ConvexPolygon::updatePosition()
{
	body.setPosition(currentPosition);
	body.setRotation(currentAngle);
}

void ConvexPolygon::detectBarrierCollision(rectBarrier& b)
{
	Collisions::polygonBarrierDetection(*this, b);
}

void ConvexPolygon::detectEntityCollision(Entity& e)
{
	e.detectPolygonCollision(*this);
}

void ConvexPolygon::detectCircleCollision(Circle& c)
{
	Collisions::circlePolygonDetection(c, *this);
}

void ConvexPolygon::detectPolygonCollision(ConvexPolygon& polygon)
{
	Collisions::polygonPolygonDetection(*this, polygon);
}

void ConvexPolygon::renderEntity(sf::RenderWindow& target)
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
	body.setPoint(0, { -(size / 2.f), -(size / 2.f) });
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

Triangle::Triangle()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.f;

	mass = 1.f;
	size = 10.f;
	diameter = VectorMath::root3 * 0.5f * size;
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
	diameter = VectorMath::root3 * 0.5f * size;
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

Hexagon::Hexagon()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = 0.f;

	mass = 1.f;
	size = 10.f;
	diameter = 2.f * size;
	momentOfInertia = mass * size * size / 8.f;

	body.setPointCount(6);
	body.setPoint(0, { -(0.5f * size), -(size * VectorMath::root3 * 0.5f) });
	body.setPoint(1, { (0.5f * size), -(size * VectorMath::root3 * 0.5f) });
	body.setPoint(2, { size, 0.f });
	body.setPoint(3, { (0.5f * size), (size * VectorMath::root3 * 0.5f) });
	body.setPoint(4, { -(0.5f * size), (size * VectorMath::root3 * 0.5f) });
	body.setPoint(5, { -size, 0.f });

	color = sf::Color::Green;
	body.setFillColor(color);
	resCoeff = 0.8f;

	force = { 0.f, 0.f };
}

Hexagon::Hexagon(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;

	currentAngle = 0.f;
	oldAngle = -0.4f;

	mass = inputMass;
	size = inputSize;
	diameter = 2.f * size;
	momentOfInertia = mass * size * size / 8.f;

	//Points for a hexagon are defined starting in top left corner and moving clockwise.
	//Co-ordinates of points are relative to the position of the body

	body.setPointCount(6);
	body.setPoint(0, { -(0.5f * size), -(size * VectorMath::root3 * 0.5f) });
	body.setPoint(1, { (0.5f * size), -(size * VectorMath::root3 * 0.5f) });
	body.setPoint(2, { size, 0.f });
	body.setPoint(3, { (0.5f * size), (size * VectorMath::root3 * 0.5f) });
	body.setPoint(4, { -(0.5f * size), (size * VectorMath::root3 * 0.5f) });
	body.setPoint(5, { -size, 0.f });

	color = inputColor;
	body.setFillColor(color);
	resCoeff = 0.8f;

	force = { 0.f, 0.f };
	/*
	body.setOutlineThickness(1);
	body.setOutlineColor(sf::Color(250, 150, 100));
	*/
}

Hexagon::~Hexagon()
{
}

void Hexagon::getBoundingBox(float& maxX, float& minX, float& maxY, float& minY)
{
	maxX = this->currentPosition.x + size;
	minX = this->currentPosition.x - size;
	maxY = this->currentPosition.y + size;
	minY = this->currentPosition.y - size;
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

	vertexPositions = this->getAllVertices();
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

	vertexPositions = this->getAllVertices();
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

	vertexPositions = this->getAllVertices();
}

sf::Vector2f rectBarrier::getVertexPosition(int vertex)
{
	//For rectangle shape, regardless of setOrigin, getPoint for 0th vertex will always give 0,0.
	sf::Vector2f nonRotatedPosition = body.getPoint(vertex) - size * 0.5f;
	float angle = VectorMath::degToRadFactor * body.getRotation();

	float x = (nonRotatedPosition.x * std::cos(angle)) - (nonRotatedPosition.y * std::sin(angle));
	float y = (nonRotatedPosition.x * std::sin(angle)) + (nonRotatedPosition.y * std::cos(angle));

	return { x + position.x, y + position.y };
}

std::vector<sf::Vector2f> rectBarrier::getAllVertices()
{
	int vertexCount = this->body.getPointCount();
	std::vector<sf::Vector2f> vertices(vertexCount);

	for (int i = 0; i < vertexCount; ++i) {
		vertices[i] = this->getVertexPosition(i);
	}
	return vertices;
}

void rectBarrier::detectPolygonCollision(ConvexPolygon& polygon)
{
	Collisions::polygonBarrierDetection(polygon, *this);
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
	springBody.setOrigin({ 0, springWidth / 2 });
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

	springBody.setPosition({ entity1->currentPosition.x, entity1->currentPosition.y });
	position = entity1->currentPosition;

	springBody.setSize({ pow(v.x * v.x + v.y * v.y, 0.5f), springWidth });

	springBody.setRotation(VectorMath::radToDegFactor * (atan2(10.f, 0.f) - atan2(v.x, v.y)));

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