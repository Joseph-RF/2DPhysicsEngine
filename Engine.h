#pragma once

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>

#include <iostream>
#include <cmath>
#include <memory>

extern int windowWidth;
extern int windowHeight;
extern float f_windowWidth;
extern float f_windowHeight;

class Entity
{
public:
	sf::Vector2f currentPosition;
	sf::Vector2f centrePosition;
	sf::Vector2f currentVelocity;
	sf::Vector2f currentAcceleration;
	sf::Vector2f force;

	float mass;
	float size;
	float resCoeff;

	sf::CircleShape body;
	sf::Color color;

	Entity();
	Entity(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor);
	~Entity();

	void updatePosition();

	void renderEntity(sf::RenderWindow &target);
};

class rectBarrier
{
public:
	sf::RectangleShape body;
	sf::Color color;

	sf::Vector2f size;
	sf::Vector2f position;

	rectBarrier();
	rectBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition);
	~rectBarrier();

	void setBarrier(sf::Color inputColor, sf::Vector2f inputSize, sf::Vector2f inputPosition);

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
	void applyForces();

	void render(sf::RenderWindow& target);
};

class Quadtree
{
private:
	int max_objects = 10;
	int max_levels = 5;

	sf::Vector2f position;
	sf::Vector2f bounds;

	int level;
	bool isSplit;
	std::vector<Entity*> entities;
	std::vector<Quadtree> nodes;

	void split();
	int getIndex(Entity* entity);

public:
	Quadtree();
	Quadtree(int pLevel, sf::Vector2f pPosition, sf::Vector2f pBounds);
	~Quadtree();

	void init(int pLevel, sf::Vector2f pPosition, sf::Vector2f pBounds);

	void clear();
	void insert(Entity* entity);

	std::vector<Entity*> retrieve(std::vector<Entity*>& returnObjects, Entity* entity);

};

class Engine
{
private:

	sf::RenderWindow* window;
	sf::Event e;

	Quadtree quad;

public:
	Engine();
	~Engine();

	float timeStep;
	float timeElapsed;
	int subSteps;
	int entitiesSpawned;

	std::vector<Entity*> Entities;
	std::vector<rectBarrier> rectBarriers;
	std::vector<Spring> Springs;
	rectBarrier upperBarrier;
	rectBarrier rightBarrier;
	rectBarrier lowerBarrier;
	rectBarrier leftBarrier;

	void initVariables();
	void initWindow();
	void initScene();

	void addEntities();
	void addSpring();
	void addSponge();

	void pollEvents();
	void solver(Entity& E, float dt);
	inline void applyGravity(Entity& E);
	void detectEntityBarrierCollision();
	void detectEntityEntityCollision();

	void update();
	void updateQuadtree();
	void updateEntities();
	void updateSprings();

	void render();


	const bool isWindowOpen() const;
};
