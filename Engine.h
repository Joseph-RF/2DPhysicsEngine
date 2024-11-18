#pragma once

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>

#include "VectorMath.h"
#include "Shapes.h"

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

class Entity;
class Circle;
class ConvexPolygon;
class rectBarrier;
class Spring;

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

	std::vector<std::vector<Entity*>> grid;

	sf::Text fpstext;
	sf::Text entitiesSpawnedText;
	sf::Text shapeButtonsText;

	void initVariables();
	void initWindow();
	void initFont();
	void initText();
	void initScene();

	void addCircle();
	void addSquare();
	void addTriangle();
	void addHexagon();
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

	sf::Color generateVividColour();
	sf::Color hsvToRgb(int h, float s, float v);

	void update(float dt);
	void updateEntities();
	void updateSprings();
	void updateText();

	void render();

	const bool isWindowOpen() const;
};