#ifndef CONNECTION_H
#define CONNECTION_H

#define CPPHTTPLIB_OPENSSL_SUPPORT
#include <httplib.h>

#include <ArduinoJson.h>
#include <map>
#include <set>
#include <string>
#include "Navigator.h"
#include "RobotMovements.h"
#include "../../src/Common.h"

class Explorer;

enum MessTypeConn
{
    UNKNOWN = 0,
    SET_TARGET = 1,
    START_EXPLORE_CONN = 2,
    STOP_EXPLORE = 3
};

class Connection
{

private:
    httplib::Server server;

    void handleMessage(MessTypeConn messageType, JsonVariant bodyMessage);
    void openResource(const httplib::Request& req, httplib::Response& res, bool isRoot);
    void sendMap(const httplib::Request& req, httplib::Response& res);

    const std::map<Pos, Chunk> *_map = nullptr;
    Navigator *_nav = nullptr;
    Explorer *_exp = nullptr;
    RobotMovements *_rb = nullptr;

    MessTypeConn stringToEnum(const std::string& str)
    {
        if (str == "SET_TARGET")
            return SET_TARGET;
        if (str == "START_EXPLORE_CONN")
            return START_EXPLORE_CONN;
        if (str == "STOP_EXPLORE")
            return STOP_EXPLORE;

        return UNKNOWN;
    }

public:
    Connection() {}
    void init(Navigator &nav, Explorer &exp, RobotMovements &rb);
    void update();
};

#endif