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

	static bool checkBoundingBox(
		Circle& circle,
		rectBarrier& barrier
	);
	static bool checkBoundingBox(
		ConvexPolygon& polygon,
		rectBarrier& barrier
	);
	static bool checkBoundingBox(
		Circle& circle,
		ConvexPolygon& polygon
	);
	static bool checkBoundingBox(
		ConvexPolygon& polygonA,
		ConvexPolygon& polygonB
	);

	static void circleCircleDetection(
		Circle& c1,
		Circle& c2
	);
	static void circleCircleResolution(
		Circle& c1,
		Circle& c2,
		float depth,
		sf::Vector2f axis
	);

	static void circlePolygonDetection(
		Circle& c,
		ConvexPolygon& polygon
	);
	static void circlePolygonResolution(
		Circle& c,
		ConvexPolygon& polygon,
		float depth,
		sf::Vector2f axis,
		const sf::Vector2f& contactPointOnPolygon
	);

	static void polygonPolygonDetection(
		ConvexPolygon& polygonA,
		ConvexPolygon& polygonB
	);
	static bool projectionSAT(
		std::vector<sf::Vector2f>& polygonAVertices,
		std::vector<sf::Vector2f>& polygonBVertices,
		float& minDepth,
		sf::Vector2f& minAxis
	);
	static void polygonPolygonResolution(
		ConvexPolygon& polygonA,
		ConvexPolygon& polygonB,
		float depth,
		sf::Vector2f axis
	);

	static sf::Vector2f findContactPoint(
		Circle& c,
		ConvexPolygon& convexPolygon
	);

	static void findContactPoints(
		std::vector<sf::Vector2f>& shapeAVertices,
		std::vector<sf::Vector2f>& shapeBVertices,
		sf::Vector2f& contactPoint1,
		sf::Vector2f& contactPoint2,
		int& contactCount
	);

	static void polygonBarrierDetection(
		ConvexPolygon& polygon,
		rectBarrier& barrier
	);
	static void polygonBarrierResolution(
		ConvexPolygon& polygon,
		rectBarrier& barrier,
		const float& depth,
		const sf::Vector2f& axis
	);

	static void circleBarrierDetection(
		Circle& c,
		rectBarrier& b
	);
	static void circleBarrierResolution(
		Circle& c,
		rectBarrier& b,
		const float& depth,
		const sf::Vector2f& axis,
		const sf::Vector2f& contactPointOnBarrier
	);

	static std::vector<sf::Vector2f> getPolygonVertexPositions(
		ConvexPolygon& polygon
	);
	static std::vector<sf::Vector2f> getBarrierVertexPositions(
		rectBarrier& barrier
	);

	sf::Color generateVividColour();
	sf::Color hsvToRgb(int h, float s, float v);

	void update(float dt);
	void updateEntities();
	void updateSprings();
	void updateText();

	void render();

	const bool isWindowOpen() const;
};