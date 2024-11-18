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