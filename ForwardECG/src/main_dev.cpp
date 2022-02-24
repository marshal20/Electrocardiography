#include "main_dev.h"

static glGraphicsDevice* main_gdev = 0;

glGraphicsDevice* gdevGet()
{
	return main_gdev;
}

void gdevSet(glGraphicsDevice* gdev)
{
	main_gdev = gdev;
}
