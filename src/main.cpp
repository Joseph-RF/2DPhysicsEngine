#include "Engine.h"

int main()
{
	sf::Clock mainClock;
	float t = 0.0;
	const float dt = 1.f / 60.f;

	float currentTime = mainClock.getElapsedTime().asSeconds();
	float accumulator = 0.0;

	srand(time(NULL));

	Engine engine;

	while (engine.isWindowOpen()) {

		float newTime = mainClock.getElapsedTime().asSeconds();
		float frameTime = newTime - currentTime;
		currentTime = newTime;

		accumulator += frameTime;
		
		while (accumulator >= dt)
		{
			engine.update(dt);
			accumulator -= dt;
			t += dt;
		}
		
		engine.render();
	}
}