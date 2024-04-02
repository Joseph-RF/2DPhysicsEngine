#include "Engine.h"

int main()
{
	srand(time(NULL));

	Engine engine;

	while (engine.isWindowOpen()) {
		engine.update();

		engine.render();
	}
}