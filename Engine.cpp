#include "Engine.h"

int windowWidth = 1200;
int windowHeight = 860;
float f_windowWidth = 1200.f;
float f_windowHeight = 860.f;

Game::Game()
{
	initVariables();
	initWindow();
	initScene();
}

Game::~Game()
{
	for (size_t i = 0; i < Entities.size(); i++) {
		delete Entities[i];
	}

	delete window;
}

void Game::initVariables()
{
	window = nullptr;
	timeStep = (1.f / 30.f);
	timeElapsed = 0.0f;
	subSteps = 8;
	entitiesSpawned = 0;
}

void Game::initScene()
{
	//Rectangular barrier near the bottom of the screen to act as a base
	//rectBarriers.push_back(new rectBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, f_windowHeight * 0.9f)));
	//rectBarriers.push_back(new rectBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(0, 0)));
	//rectBarriers.push_back(new rectBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(0, f_windowHeight - 20)));
	//rectBarriers.push_back(new rectBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, f_windowHeight)));
	//For now will just add a singular entity near the top of the screen in the middle
	//Entities.push_back(new Entity(sf::Vector2f(f_windowWidth / 2, f_windowHeight * 0.1f), 1.f, 120.f, sf::Color::White));

	upperBarrier.setBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, 0));
	rightBarrier.setBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(f_windowWidth - 20, 0));
	lowerBarrier.setBarrier(sf::Color::White, sf::Vector2f(f_windowWidth, 20), sf::Vector2f(0, f_windowHeight * 0.9f));
	leftBarrier.setBarrier(sf::Color::White, sf::Vector2f(20, f_windowHeight), sf::Vector2f(0, 0));
}

void Game::initWindow()
{
	window = new sf::RenderWindow(sf::VideoMode(windowWidth, windowHeight), "Physics Engine");
	window->setFramerateLimit(60);
}

void Game::addEntities()
{
	//Entities.emplace_back(Entity(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 10.f, sf::Color(rand() % 255, rand() % 255, rand() % 255, 255)));
	Entities.emplace_back(new Entity(sf::Vector2f(rand() % 1000 + 100, 50.f), 1.f, 10.f, sf::Color::Green));
	entitiesSpawned++;
}

void Game::addSpring()
{
	addEntities();
	addEntities();

	Springs.emplace_back(Spring(Entities[Entities.size() - 2], Entities[Entities.size() - 1], 10.f, 300.f));
}

void Game::pollEvents()
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
				//std::cout << entitiesSpawned << std::endl;
			}
			break;
		}
	}
}

void Game::solver(Entity& E, float dt)
{
	timeElapsed += timeStep;
	//std::cout << "Called the solver" << std::endl;
	sf::Vector2f newPos = E.currentPosition + E.currentVelocity * dt + E.currentAcceleration * (dt * dt * 0.5f);
	applyGravity(E);
	sf::Vector2f newAcc = (E.force / E.mass);
	//std::cout << "Acceleration should be: " << newAcc.y << std::endl;
	//std::cout << "Acceleration is: " << E.currentAcceleration.y << std::endl;
	sf::Vector2f newVel = E.currentVelocity + (E.currentAcceleration + newAcc) * (dt * 0.5f);
	//std::cout << "Time interval is: " << dt << std::endl;
	//std::cout << "Velocity y currently is: " << E.currentVelocity.y << std::endl;
	//std::cout << "Velocity x currently is: " << E.currentVelocity.x << std::endl;
	//std::cout << "Time elapsed is: " << timeElapsed << std::endl;
	E.currentPosition = newPos;
	E.currentVelocity = newVel;
	E.currentAcceleration = newAcc;

	E.force = { 0.f, 0.f };
}

inline void Game::applyGravity(Entity& E)
{
	//Different forces will be added here
	E.force += {0.0f, (200.f * E.mass)}; //Gravity
}

/*
void Game::detectEntityBarrierCollision()
{
	size_t entityNumber = Entities.size();
	size_t barrierNumber = rectBarriers.size();
	for (size_t i = 0; i < barrierNumber; i++) {
		sf::Vector2f barrierPos = rectBarriers[i]->position;
		for (size_t j = 0; j < entityNumber; j++) { 
			if ((rectBarriers[i]->position.y - (Entities[j]->currentPosition.y + Entities[j]->size)) < Entities[j]->size) {
				//Moving the entity to the right position
				Entities[j]->currentPosition.y -= ((2 * Entities[j]->size) - (rectBarriers[i]->position.y - Entities[j]->currentPosition.y));
				//Giving the entity a normal force
				Entities[j]->normalForce = { 0.f, -200.f * Entities[j]->mass};

				//Giving the entity an impulse
				Entities[j]->currentVelocity.y *= -Entities[j]->resCoeff;

			} else {
				//Remove the normal force because the entity is no longer touching a barrier
				Entities[j]->normalForce = { 0.f, 0.f };
			}
		}
	}
}
*/

void Game::detectEntityBarrierCollision()
{
	size_t entityNumber = Entities.size();
	for (size_t i = 0; i < entityNumber; i++) {

		Entity& entity = *Entities[i];

		if (lowerBarrier.position.y - entity.currentPosition.y <= 2 * entity.size) {
			//Move the entity to the right position (not inside of the barrier)
			entity.currentPosition.y -= ((2 * entity.size) - (lowerBarrier.position.y - entity.currentPosition.y));
			//Give the enitity an impulse
			entity.currentVelocity.y *= -entity.resCoeff;
		}
		if (rightBarrier.position.x - entity.currentPosition.x <= 2 * entity.size) {

			entity.currentPosition.x -= ((2 * entity.size) - (rightBarrier.position.x - entity.currentPosition.x));

			entity.currentVelocity *= -entity.resCoeff;
		} 
		if (upperBarrier.position.y + 20 > entity.currentPosition.y) {

			entity.currentPosition.y += (upperBarrier.position.y + 20.f - entity.currentPosition.y);

			entity.currentVelocity.y *= -entity.resCoeff;
		}
		if (leftBarrier.position.x + 20 > entity.currentPosition.x) {

			entity.currentPosition.x += (leftBarrier.position.x + 20 - entity.currentPosition.x);

			entity.currentVelocity.x *= -entity.resCoeff;
		}
	}
}

void Game::detectEntityEntityCollision()
{
	for (Entity* entity1 : Entities) {

		for (Entity* entity2 : Entities) {
			
			if (entity1 == entity2) {
				//Prevent comparison between the same two entities
				break;
			}
			
			const sf::Vector2f LoCij = entity2->centrePosition - entity1->centrePosition;
			const float LoCij_mag = pow((LoCij.x * LoCij.x) + (LoCij.y * LoCij.y), 0.5f);
			if (LoCij_mag < entity1->size + entity2->size) {
				//Moving the entities to the right position
				entity2->currentPosition += ((LoCij / 2.f) / LoCij_mag) * (entity1->size + entity2->size - LoCij_mag);
				entity1->currentPosition += -((LoCij / 2.f) / LoCij_mag) * (entity1->size + entity2->size - LoCij_mag);

				const sf::Vector2f relativeVel = entity2->currentVelocity - entity1->currentVelocity;

				const float impulse = (1.f + entity1->resCoeff) * (((relativeVel.x * LoCij.x / LoCij_mag)) + (relativeVel.y * LoCij.y / LoCij_mag)) / (1.f / entity1->mass + 1.f / entity2->mass);
				entity1->currentVelocity += impulse / entity1->mass * LoCij / LoCij_mag;
				entity2->currentVelocity -= impulse / entity2->mass * LoCij / LoCij_mag;
			}
		}
	}
}

void Game::update()
{
	pollEvents();

	for (int n = 0; n < subSteps; ++n) {

		updateSprings();
		updateEntities();
		detectEntityBarrierCollision();
		detectEntityEntityCollision();
	}
}

void Game::updateEntities()
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

void Game::updateSprings()
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
			it->applyForces();
			it++;
		}
	}
	
	/*
	for (Spring& spring : Springs) {
		spring.update();
	}
	*/
}

void Game::render()
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

	window->display();
}

const bool Game::isWindowOpen() const
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
	mass = 1;
	size = 10.f;
	body.setRadius(size);
	color = sf::Color::White;
	body.setFillColor(color);
	resCoeff = 1.f;

	//Add a small random horizontal velocity
	currentVelocity.x = static_cast<float>(rand() % 10);
	force = { 0.f, 0.f };
}

Entity::Entity(sf::Vector2f inputPos, float inputMass, float inputSize, sf::Color inputColor)
{
	currentPosition = inputPos;
	mass = inputMass;
	size = inputSize;
	body.setRadius(size);
	color = inputColor;
	body.setFillColor(color);
	resCoeff = 1.f;

	centrePosition = currentPosition + sf::Vector2f(size, size);

	//Add a small random horizontal velocity
	currentVelocity.x = static_cast<float>(rand() % 10);
	force = { 0.f, 0.f };

	/*
	float sizeModifier = (rand() % 5);
	mass = sizeModifier;
	size = (sizeModifier) * 5.f;
	*/
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

Spring::Spring(Entity* inputEntity1, Entity* inputEntity2, float inputSpringConstant, float inputRestLength)
{
	entity1 = inputEntity1;
	entity2 = inputEntity2;
	springWidth = 10.f;
	springConstant = inputSpringConstant;
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

void Spring::applyForces()
{
	//Will first find where both entities should be a.k.a their rest positions
	const sf::Vector2f v = entity2->centrePosition - entity1->centrePosition;

	const sf::Vector2f e1RestPos = centreOfMass - (v / pow(v.x * v.x + v.y * v.y, 0.5f) * e1RestDistance);
	const sf::Vector2f e2RestPos = centreOfMass + (v / pow(v.x * v.x + v.y * v.y, 0.5f) * e2RestDistance);

	//Apply the restoring force to the entities
	entity1->force += -springConstant * (entity1->centrePosition - e1RestPos);
	entity2->force += -springConstant * (entity2->centrePosition - e2RestPos);
}

void Spring::render(sf::RenderWindow& target)
{
	target.draw(springBody);
}