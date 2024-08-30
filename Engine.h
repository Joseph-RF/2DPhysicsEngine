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

extern int cell_size;
extern int cell_number_x;
extern int cell_number_y;

extern sf::Vector2f lowerBarrier_position;
extern sf::Vector2f upperBarrier_position;
extern sf::Vector2f rightBarrier_position;
extern sf::Vector2f leftBarrier_position;

class Circle;

class Entity
{
public:
	sf::Vector2f currentPosition;
	sf::Vector2f oldPosition;
	sf::Vector2f centrePosition;
	sf::Vector2f currentAcceleration;
	sf::Vector2f force;

	float mass;
	float resCoeff;

	sf::Color color;

	virtual ~Entity() = default;

	virtual void updatePosition() = 0;
	virtual void entityBarrierCollision() = 0;
	virtual void detectEntityCollision(Entity& e) = 0;
	virtual void detectCircleCollision(Circle& c) = 0;

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

	void updatePosition() override;
	void entityBarrierCollision() override;
	void detectEntityCollision(Entity& e) override;
	void detectCircleCollision(Circle& c) override;

	void renderEntity(sf::RenderWindow& target) override;
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

	std::vector<Entity*> Entities;
	std::vector<rectBarrier> rectBarriers;
	std::vector<Spring> Springs;
	rectBarrier upperBarrier;
	rectBarrier rightBarrier;
	rectBarrier lowerBarrier;
	rectBarrier leftBarrier;

	std::vector<std::vector<Entity*>> grid;

	sf::Text fpstext;
	sf::Text entitiesSpawnedText;

	void initVariables();
	void initWindow();
	void initFont();
	void initText();
	void initScene();

	void addCircle();
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

	void update(float dt);
	void updateEntities();
	void updateSprings();
	void updateText();

	void render();


	const bool isWindowOpen() const;
};