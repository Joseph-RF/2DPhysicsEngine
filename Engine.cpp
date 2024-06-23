#include "Engine.h"

int windowWidth = 1200;
int windowHeight = 800;
float f_windowWidth = 1200.f;
float f_windowHeight = 800.f;

int cell_size = 10;
int cell_number_x = windowWidth / cell_size;
int cell_number_y = windowHeight / cell_size;

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
	rightBarrier.setBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(f_windowWidth - 20, 0));
	lowerBarrier.setBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, f_windowHeight * 0.95f));
	leftBarrier.setBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(0, 0));
}

void Engine::initWindow()
{
	window = new sf::RenderWindow(sf::VideoMode(windowWidth, windowHeight), "Physics Engine");
	window->setFramerateLimit(0);
}

void Engine::addEntities()
{
	Entities.emplace_back(new Entity(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 5.f, sf::Color::Green));
	entitiesSpawned++;
}

void Engine::addSpring()
{
	addEntities();
	addEntities();

	Springs.emplace_back(Spring(Entities[Entities.size() - 2], Entities[Entities.size() - 1], 10.f, 0.2f, 300.f));
}

void Engine::addSponge()
{
	Entities.emplace_back(new Entity(sf::Vector2f(50.f, 50.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Entity(sf::Vector2f(120.f, 50.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Entity(sf::Vector2f(50.f, 120.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Entity(sf::Vector2f(120.f, 120.f), 1.f, 10.f, sf::Color::Green));
	Entities.emplace_back(new Entity(sf::Vector2f(85.f, 85.f), 1.f, 10.f, sf::Color::Green));

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
			} else if (e.key.code == sf::Keyboard::S) {
				addEntities();
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
}

inline void Engine::applyGravity(Entity& E)
{
	//Different forces will be added here
	E.force += {0.f * E.mass, (1000.f * E.mass)}; //Gravity
}

void Engine::detectEntityBarrierCollision()
{
	size_t entityNumber = Entities.size();
	for (size_t i = 0; i < entityNumber; i++) {

		Entity& entity = *Entities[i];

		const sf::Vector2f displacement = entity.currentPosition - entity.oldPosition;

		if (entity.currentPosition.y > lowerBarrier.position.y - (2 * entity.size)) {
			//Move the entity to the very edge where it would be acceptable for it to be in

			entity.currentPosition.y = lowerBarrier.position.y - (2 * entity.size);

			//Change the old position such that the entitiy receieves the right impulse. Term in brackets accounts for the coefficient 
			// of restitution since displacement is a stand-in for velocity.
			entity.oldPosition.y = entity.currentPosition.y + (entity.resCoeff * displacement.y);
		}

		if (entity.currentPosition.x > rightBarrier.position.x - (2 * entity.size)) {

			entity.currentPosition.x = rightBarrier.position.x - (2 * entity.size);

			entity.oldPosition.x = entity.currentPosition.x + (entity.resCoeff * displacement.x);
		}

		if (entity.currentPosition.y < upperBarrier.position.y + 20) {

			entity.currentPosition.y = upperBarrier.position.y + 20;

			entity.oldPosition.y = entity.currentPosition.y + (entity.resCoeff * displacement.y);
		}

		if (entity.currentPosition.x < upperBarrier.position.x + 20) {

			entity.currentPosition.x = upperBarrier.position.x + 20;

			entity.oldPosition.x = entity.currentPosition.x + (entity.resCoeff * displacement.x);
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

		sf::Vector2f separation = grid[grid_cell][k]->currentPosition - E->currentPosition;
		float separation_magnitude = pow((separation.x * separation.x) + (separation.y * separation.y), 0.5f);

		if (separation_magnitude < E->size + grid[grid_cell][k]->size) {

			//Resolve collision between the entities
			sf::Vector2f norm = separation / separation_magnitude;

			norm = sf::Vector2f(0.5 * (E->size + grid[grid_cell][k]->size - separation_magnitude) * norm.x,
				0.5 * (E->size + grid[grid_cell][k]->size - separation_magnitude) * norm.y);

			E->currentPosition -= E->resCoeff * norm;

			grid[grid_cell][k]->currentPosition += E->resCoeff * norm;
		}
	}
}

void Engine::sortEntities()
{
	for (int i = 0; i < cell_number_x * cell_number_y; ++i) {
		grid[i].clear();
	}

	for (Entity* entity : Entities) {
		grid[getCellNumber(entity->centrePosition)].emplace_back(entity);
	}
}

int Engine::getCellNumber(sf::Vector2f pos)
{
	return ((std::floor(pos.x / cell_size)) + std::floor(pos.y / cell_size) * cell_number_x);
}

void Engine::update(float dt)
{
	timeStep = dt;

	pollEvents();

	framerate = 1.f / clock.getElapsedTime().asSeconds();
	clock.restart();

	addEntities();

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

Entity::Entity()
{
	currentPosition = sf::Vector2f(f_windowWidth / 2, f_windowHeight / 2);
	oldPosition = currentPosition;
	mass = 1;
	size = 5.f;
	body.setRadius(size);
	color = sf::Color::White;
	body.setFillColor(color);
	resCoeff = 0.75f;

	force = { 0.f, 0.f };
}

Entity::Entity(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	oldPosition = currentPosition;
	mass = inputMass;
	size = inputSize;
	body.setRadius(size);
	color = inputColor;
	body.setFillColor(color);
	resCoeff = 0.75f;

	centrePosition = currentPosition + sf::Vector2f(size, size);

	force = { 0.f, 0.f };
	body.setRadius(size);
}

Entity::~Entity()
{

}

void Entity::updatePosition()
{
	body.setPosition(currentPosition);
	centrePosition = currentPosition + sf::Vector2f(size, size);
}

void Entity::renderEntity(sf::RenderWindow& target)
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
	springBody.setSize({ abs(entity1->centrePosition.x - entity2->centrePosition.x), springWidth });
	springBody.setPosition({ entity1->centrePosition.x, entity1->centrePosition.y });
	springBody.setOrigin({ 0, springWidth / 2});
	position = entity1->centrePosition;

	e1RestDistance = restLength * entity2->mass / (entity1->mass + entity2->mass);
	e2RestDistance = restLength - e1RestDistance;
}

Spring::~Spring()
{

}

void Spring::update()
{
	const sf::Vector2f v = entity2->centrePosition - entity1->centrePosition;

	springBody.setPosition({ entity1->centrePosition.x, entity1->centrePosition.y});
	position = entity1->centrePosition;

	springBody.setSize({ pow(v.x * v.x + v.y * v.y, 0.5f), springWidth });

	springBody.setRotation((180.f / 3.14f) * (atan2(10.f, 0.f) - atan2(v.x ,v.y)));

	centreOfMass = (entity1->mass * entity1->centrePosition + entity2->mass * entity2->centrePosition) / (entity1->mass + entity2->mass);
}

void Spring::applyForces(float dt)
{
	//Will first find where both entities should be a.k.a their rest positions
	const sf::Vector2f v = entity2->centrePosition - entity1->centrePosition;

	//Find the velocity of each entity from the positions
	const sf::Vector2f entity1velocity = (entity1->currentPosition - entity1->oldPosition) / dt;
	const sf::Vector2f entity2velocity = (entity2->currentPosition - entity2->oldPosition) / dt;

	const sf::Vector2f e1RestPos = centreOfMass - (v / pow(v.x * v.x + v.y * v.y, 0.5f) * e1RestDistance);
	const sf::Vector2f e2RestPos = centreOfMass + (v / pow(v.x * v.x + v.y * v.y, 0.5f) * e2RestDistance);

	//Apply the restoring force to the entities
	entity1->force += -springConstant * (entity1->centrePosition - e1RestPos) - dampingConstant * (entity1velocity - entity2velocity);
	entity2->force += -springConstant * (entity2->centrePosition - e2RestPos) - dampingConstant * (entity2velocity - entity1velocity);
}

void Spring::render(sf::RenderWindow& target)
{
	target.draw(springBody);
}