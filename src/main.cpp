#include "cleanerBot.hpp"

int main (int argc, char** argv) {
ros::init(argc, argv, "walker");

CleanerBot bot;
bot.walkCleaner();

return 0;
}
