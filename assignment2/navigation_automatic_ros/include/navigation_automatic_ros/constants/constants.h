#ifndef CONSTANTSNAVIGATIONAUTOMATICROS_H
#define CONSTANTSNAVIGATIONAUTOMATICROS_H

#include <string>

/**
 * @author : Manuel Barusco, Riccardo Rampon, Francesco Caldivezzi
*/
namespace navigation_automatic_ros
{
    //NODES
    const static std::string NODE_TIAGO_SERVER = "tiago_server";

    //TOPICS
    const static std::string TOPIC_TIAGO_SERVER = "/tiago_server";
    const static std::string TOPIC_MOVE_BASE = "move_base";
}

#endif //CONSTANTSNAVIGATIONAUTOMATICROS_H