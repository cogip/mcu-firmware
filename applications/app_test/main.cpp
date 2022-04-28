#include "app.hpp"
#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"

#include "obstacles/obstacles.hpp"
#include "obstacles/Circle.hpp"
#include "obstacles/Rectangle.hpp"
#include "obstacles/List.hpp"

class FixedObstacle: public cogip::obstacles::Rectangle {
public:
    FixedObstacle(
        const cogip::cogip_defs::Coords &center, double angle,
        double length_x, double length_y
        ): cogip::obstacles::Rectangle(
            center, angle, length_x + ROBOT_MARGIN * 2, length_y + ROBOT_MARGIN * 2) {};
};

class SampleObstacle: public cogip::obstacles::Circle {
public:
    SampleObstacle(const cogip::cogip_defs::Coords &center, double radius):
        cogip::obstacles::Circle(center, radius + ROBOT_MARGIN) {};
};

static void _init_border_obstacles(void)
{
    cogip::obstacles::List *border_obstacles = new cogip::obstacles::List();

    // Table borders
    border_obstacles->push_back(new FixedObstacle({ 0, 0 }, 0, 3000, 1));
    border_obstacles->push_back(new FixedObstacle({ 0, 2000 }, 0, 3000, 1));
    border_obstacles->push_back(new FixedObstacle({ -1500, 1000 }, 0, 1, 2000));
    border_obstacles->push_back(new FixedObstacle({ 1500, 1000 }, 0, 1, 2000));

    // Gallery split cleats
    border_obstacles->push_back(new FixedObstacle({ 0, 150 }, 0, 11, 150));

    // Statuette supports
    border_obstacles->push_back(new FixedObstacle({ -1500, 2000 }, 45, 721.25, 721.25));
    border_obstacles->push_back(new FixedObstacle({ 1500, 2000 }, -45, 721.25, 721.25));

    // Sample supports
    border_obstacles->push_back(new FixedObstacle({ -150, 51 }, 0, 150, 102));
    border_obstacles->push_back(new FixedObstacle({ 150, 51 }, 0, 150, 102));
    border_obstacles->push_back(new FixedObstacle({ -1449, 1250 }, 0, 102, 150));
    border_obstacles->push_back(new FixedObstacle({ 1449, 1250 }, 0, 102, 150));

    // Exhibition galleries
    border_obstacles->push_back(new FixedObstacle({ -690, 0 }, 0, 720, 84.87*2));
    border_obstacles->push_back(new FixedObstacle({ 690, 0 }, 0, 720, 84.87*2));
}

static void _init_excavation_sites_obstacles(void)
{
    cogip::obstacles::List *obstacles = new cogip::obstacles::List();
    obstacles->push_back(new FixedObstacle({ 525, 1375 }, 0, 350, 350));
    obstacles->push_back(new FixedObstacle({ -525, 1375 }, 0, 350, 350));
}

static void _init_samples_obstacles(void)
{
    cogip::obstacles::List *obstacles = new cogip::obstacles::List();
    obstacles->push_back(new SampleObstacle({ 600, 555 }, 75));
    obstacles->push_back(new SampleObstacle({ 670, 675 }, 75));
    obstacles->push_back(new SampleObstacle({ 600, 795 }, 75));
    obstacles->push_back(new SampleObstacle({ -600, 555 }, 75));
    obstacles->push_back(new SampleObstacle({ -670, 675 }, 75));
    obstacles->push_back(new SampleObstacle({ -600, 795 }, 75));
}

static int cmd_wizard(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_wizard();
    return 0;
}

int main(void)
{
    pf_init();
    app_init();

    pf_init_tasks();
    app_init_tasks();

    _init_border_obstacles();
    _init_excavation_sites_obstacles();
    _init_samples_obstacles();

    cogip::shell::root_menu.push_back(
        new cogip::shell::Command("_wizard", "Run Wizard", cmd_wizard));

    // Start shell
    cogip::shell::start();

    return 0;
}
