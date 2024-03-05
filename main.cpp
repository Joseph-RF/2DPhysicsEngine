#include "Engine.h"

//Consider making a more complicated collision detection program later down the line

int main()
{
	srand(time(NULL));

	Game game;

	while (game.isWindowOpen()) {
		game.update();

		game.render();
	}
}