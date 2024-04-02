#include "Engine.h"

int main()
{
	srand(time(NULL));

	Game game;

	while (game.isWindowOpen()) {
		game.update();

		game.render();
	}
}