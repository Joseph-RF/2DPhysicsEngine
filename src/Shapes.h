#pragma once

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>

#include "VectorMath.h"
#include "Collisions.h"
#include "Engine.h"

class Engine;
class Circle;
class ConvexPolygon;
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

	sf::Vector2f getVertexPosition(int vertex);
	std::vector<sf::Vector2f> getAllVertices();

	void updatePosition();

	void detectBarrierCollision(rectBarrier& b);
	void detectEntityCollision(Entity& e);
	void detectCircleCollision(Circle& c);
	void detectPolygonCollision(ConvexPolygon& polygon);

	void renderEntity(sf::RenderWindow& target);
};

class Square : public ConvexPolygon
{
public:
	float size;

	Square();
	Square(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Square();

	void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) override;
};

class Triangle : public ConvexPolygon
{
public:
	float size;

	Triangle();
	Triangle(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Triangle();

	void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) override;
};

class Hexagon : public ConvexPolygon
{
public:
	float size;

	Hexagon();
	Hexagon(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Hexagon();

	void getBoundingBox(float& maxX, float& minX, float& maxY, float& minY) override;
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
	std::vector<sf::Vector2f> getAllVertices();
	void detectPolygonCollision(ConvexPolygon& polygon);

	void renderBarrier(sf::RenderWindow& target);
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