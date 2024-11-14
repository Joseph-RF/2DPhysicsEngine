#pragma once

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>

#include <iostream>
#include <cmath>
#include <memory>
#include <limits>

extern int windowWidth;
extern int windowHeight;
extern float f_windowWidth;
extern float f_windowHeight;

extern int cell_size;
extern int cell_number_x;
extern int cell_number_y;

extern int counter;
extern float float_upperLimit;
extern float float_lowerLimit;
extern float root3;

extern sf::Vector2f lowerBarrier_position;
extern sf::Vector2f upperBarrier_position;
extern sf::Vector2f rightBarrier_position;
extern sf::Vector2f leftBarrier_position;

class Circle;
class ConvexPolygon;
class Square;
class Triangle;
class rectBarrier;

class Entity
{
public:
	sf::Vector2f currentPosition;
	sf::Vector2f oldPosition;

	float currentAngle;
	float oldAngle;

	sf::Vector2f currentAcceleration;
	sf::Vector2f force;

	float diameter;
	float mass;
	float momentOfInertia;

	float resCoeff;

	sf::Color color;

	virtual ~Entity() = default;

	virtual void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) = 0;

	virtual void updatePosition() = 0;
	virtual void detectBarrierCollision(rectBarrier& b) = 0;
	virtual void detectEntityCollision(Entity& e) = 0;
	virtual void detectCircleCollision(Circle& c) = 0;
	virtual void detectPolygonCollision(ConvexPolygon& polygon) = 0;

	virtual void renderEntity(sf::RenderWindow& target) = 0;
};

class Circle : public Entity
{
public:
	float size;

	sf::CircleShape body;

	Circle();
	Circle(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Circle();

	void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) override;

	void updatePosition() override;
	void detectBarrierCollision(rectBarrier& b) override;
	void detectEntityCollision(Entity& e) override;
	void detectCircleCollision(Circle& c) override;
	void detectPolygonCollision(ConvexPolygon& polygon) override;

	void renderEntity(sf::RenderWindow& target) override;
};

class ConvexPolygon : public Entity
{
public:
	sf::ConvexShape body;
	virtual sf::Vector2f getVertexPosition(int vertex) = 0;
};

class Square : public ConvexPolygon
{
public:
	float size;

	Square();
	Square(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Square();

	void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) override;

	void updatePosition() override;
	void detectBarrierCollision(rectBarrier& b) override;
	void detectEntityCollision(Entity& e) override;
	void detectCircleCollision(Circle& c) override;
	void detectPolygonCollision(ConvexPolygon& polygon) override;

	sf::Vector2f getVertexPosition(int vertex) override;

	void renderEntity(sf::RenderWindow& target) override;
};

class Triangle : public ConvexPolygon
{
public:
	float size;

	Triangle();
	Triangle(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Triangle();

	void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) override;

	void updatePosition() override;
	void detectBarrierCollision(rectBarrier& b) override;
	void detectEntityCollision(Entity& e) override;
	void detectCircleCollision(Circle& c) override;
	void detectPolygonCollision(ConvexPolygon& polygon) override;

	sf::Vector2f getVertexPosition(int vertex) override;

	void renderEntity(sf::RenderWindow& target) override;
};

class rectBarrier
{
public:
	sf::RectangleShape body;
	sf::Color color;

	sf::Vector2f size;
	sf::Vector2f position;

	std::vector<sf::Vector2f> vertexPositions;

	rectBarrier();
	rectBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition);
	~rectBarrier();

	void setBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition);

	sf::Vector2f getVertexPosition(int vertex);
	void detectPolygonCollision(ConvexPolygon& polygon);

	void renderBarrier(sf::RenderWindow &target);
};

class Spring
{
private:
	Entity* entity1;
	Entity* entity2;

public:
	sf::RectangleShape springBody;
	sf::Vector2f position;
	float springWidth;
	
	float springConstant;
	float dampingConstant;
	float restLength;
	sf::Vector2f centreOfMass;
	float e1RestDistance;
	float e2RestDistance;

	Spring(Entity* inputEntity1, Entity* inputEntity2, float inputSpringConstant, float inputDampingConstant, float inputRestLength);
	~Spring();

	void update();
	void applyForces(float dt);

	void render(sf::RenderWindow& target);
};

class Engine
{
private:

	sf::RenderWindow* window;
	sf::Event e;
	sf::Clock clock;
	sf::Font font;

	int framerate;

public:
	Engine();
	~Engine();

	float timeStep;
	float timeElapsed;
	int subSteps;
	int entitiesSpawned;

	static std::vector<sf::RectangleShape> objectsToDraw;
	std::vector<Entity*> Entities;
	std::vector<rectBarrier> rectBarriers;
	std::vector<Spring> Springs;

	std::vector<std::vector<Entity*>> grid;

	sf::Text fpstext;
	sf::Text entitiesSpawnedText;

	void initVariables();
	void initWindow();
	void initFont();
	void initText();
	void initScene();

	void addCircle();
	void addSquare();
	void addTriangle();
	void addSpring();
	void addSponge();

	void pollEvents();
	void solver(Entity& E, float dt);
	inline void applyGravity(Entity& E);
	void detectEntityBarrierCollision();
	void wideSweep();
	void detectEntityEntityCollision(int grid_cell, Entity* E);
	void sortEntities();
	int getCellNumber(sf::Vector2f pos);

	static bool checkBoundingBox(Circle& circle, rectBarrier& barrier);
	static bool checkBoundingBox(ConvexPolygon& polygon, rectBarrier& barrier);
	static bool checkBoundingBox(Circle& circle, ConvexPolygon& polygon);
	static bool checkBoundingBox(ConvexPolygon& polygonA, ConvexPolygon& polygonB);

	static void circleCircleDetection(Circle& c1, Circle& c2);
	static void circleCircleResolution(Circle& c1, Circle& c2, float depth, sf::Vector2f axis);

	static void circlePolygonDetection(Circle& c, ConvexPolygon& polygon);
	static void circlePolygonResolution(Circle& c, ConvexPolygon& polygon, float depth, sf::Vector2f axis, const sf::Vector2f& contactPointOnPolygon);
	static sf::Vector2f closestPointOnSegmentToCircle(Circle& c, sf::Vector2f& a, sf::Vector2f& b);

	static void polygonPolygonDetection(ConvexPolygon& polygonA, ConvexPolygon& polygonB);
	static bool projectionSAT(std::vector<sf::Vector2f>& polygonAVertices, std::vector<sf::Vector2f>& polygonBVertices, float& minDepth, sf::Vector2f& minAxis);
	static void polygonPolygonResolution(ConvexPolygon& polygonA, ConvexPolygon& polygonB, float depth, sf::Vector2f axis);

	static sf::Vector2f closestPointOnLineSegment(const sf::Vector2f& circlePos, const sf::Vector2f& vertex1Pos, const sf::Vector2f& vertex2Pos);
	static sf::Vector2f findContactPoint(Circle& c, ConvexPolygon& convexPolygon);

	static void findContactPoints(ConvexPolygon& polygon, rectBarrier& barrier, sf::Vector2f& contactPoint1, sf::Vector2f& contactPoint2, int& contactCount);
	static void findContactPoints(ConvexPolygon& polygonA, ConvexPolygon& polygonB, sf::Vector2f& contactPoint1, sf::Vector2f& contactPoint2, int& contactCount);

	static std::pair<sf::Vector2f, sf::Vector2f> findReferenceEdge(ConvexPolygon& polygonA, ConvexPolygon& polygonB, sf::Vector2f& axis);
	static std::pair<sf::Vector2f, sf::Vector2f> findReferenceEdge(ConvexPolygon& polygon, rectBarrier& barrier, sf::Vector2f& axis);

	static void polygonBarrierDetection(ConvexPolygon& polygon, rectBarrier& barrier);
	static void polygonBarrierResolution(ConvexPolygon& polygon, rectBarrier& barrier, const float& depth, const sf::Vector2f& axis);

	static void circleBarrierDetection(Circle& c, rectBarrier& b);
	static void circleBarrierResolution(Circle& c, rectBarrier& b, const float& depth, const sf::Vector2f& axis, const sf::Vector2f& contactPointOnBarrier);

	static float dotProduct(const sf::Vector2f& v1, const sf::Vector2f& v2);
	static float crossProduct(const sf::Vector2f& v1, const sf::Vector2f& v2);
	static sf::Vector2f normalise(const sf::Vector2f& v);
	static float findDistance(const sf::Vector2f& v1, const sf::Vector2f& v2);
	static sf::Vector2f perpendicular(const sf::Vector2f& v);
	static bool nearlyEqual(const float a1, const float a2);
	static bool nearlyEqual(const sf::Vector2f& v1, const sf::Vector2f& v2);

	static std::vector<sf::Vector2f> getPolygonVertexPositions(ConvexPolygon& polygon);
	static std::vector<sf::Vector2f> getBarrierVertexPositions(rectBarrier& barrier);

	void update(float dt);
	void updateEntities();
	void updateSprings();
	void updateText();

	void render();

	const bool isWindowOpen() const;
};