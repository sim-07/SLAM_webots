#include "Connection.h"
#include <ArduinoJson.h>
#include <Navigator.h>
#include <set>
#include <Explorer.h>
#include <fstream>
#include <iostream>

void Connection::init(Navigator &nav, Explorer &exp, RobotMovements &rb)
{
    std::cout << "Init conn in Webots" << std::endl;

    _map = &nav.getMap();
    _nav = &nav;
    _rb = &rb;
    _exp = &exp;

    server.Get("/api/getMap", [this](const httplib::Request& req, httplib::Response& res) {
        //std::cout << "Received /api/getMap request" << std::endl;
        this->sendMap(req, res);
    });

    server.Get("/", [this](const httplib::Request& req, httplib::Response& res) {
        this->openResource(req, res, true);
    });

    server.Post("/api/sendMessageToEsp", [this](const httplib::Request& req, httplib::Response& res) {
        if (!req.body.empty()) {
            std::string body = req.body;

            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, body);

            if (error) {
                res.status = 400;
                res.set_content("Corrupt JSON", "text/plain");
                return;
            }

            std::string messageType = doc["messageType"] | "";
            JsonVariant bodyMessage = doc["body"];

            this->handleMessage(stringToEnum(messageType), bodyMessage);
            
            res.status = 200;
            res.set_content("Received " + messageType, "text/plain");
        } else {
            res.status = 400;
            res.set_content("Body missing", "text/plain");
        }
    });

    server.Get(".*", [this](const httplib::Request& req, httplib::Response& res) {
        this->openResource(req, res, false);
    });
    
    std::thread([this]() {
        server.listen("127.0.0.1", 8080);
    }).detach();
}

void Connection::handleMessage(MessTypeConn messageType, JsonVariant bodyMessage)
{
    switch (messageType)
    {
    case SET_TARGET:
        std::cout << "Received SET_TARGET" << std::endl;
        if (bodyMessage["x"].is<int>() && bodyMessage["y"].is<int>())
        {
            _rb->setCurrentState(GOTO);
            _rb->setDestination(bodyMessage["x"], bodyMessage["y"]);
        }
        break;

    case START_EXPLORE_CONN:
        std::cout << "Received START_EXPLORE_CONN" << std::endl;
        _exp->setCurrentState(START_EXPLORE);
        break;

    case STOP_EXPLORE:
        std::cout << "Received STOP_EXPLORE" << std::endl;
        _exp->setCurrentState(COMPLETED);
        break;
    default:
        break;
    }
}

void Connection::sendMap(const httplib::Request& req, httplib::Response& res)
{
    std::vector<Pos> positions;

    for (const auto &pair : *_map)
    {
        positions.push_back(pair.first);
    }

    std::string binary_data = "";
    uint8_t tempCells[256];

    int16_t rbCoords[2] = {_nav->getPos().x, _nav->getPos().y};
    binary_data.append((const char*)rbCoords, sizeof(rbCoords));

    for (const auto &p : positions)
    {
        auto it = _map->find(p);
        if (it != _map->end())
        {
            memcpy(tempCells, it->second.cells, 256);
        }
        else
        {
            continue;
        }

        int16_t coords[2] = {p.x, p.y};
        
        binary_data.append((const char*)coords, sizeof(coords));
        binary_data.append((const char*)tempCells, 256);
    }

    res.status = 200;
    res.set_content(binary_data, "application/octet-stream");
}

void Connection::update() {}

void Connection::openResource(const httplib::Request& req, httplib::Response& res, bool isRoot)
{

    std::string path = "./data/index.html";

    std::ifstream file(path, std::ios::binary);
    if (file.is_open())
    {
        std::string contentType = "text/plain";
        if (path.find(".html") != std::string::npos) contentType = "text/html";
        else if (path.find(".css") != std::string::npos) contentType = "text/css";
        else if (path.find(".js") != std::string::npos) contentType = "application/javascript";

        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        res.status = 200;
        res.set_content(content, contentType);
        file.close();
    }
    else
    {
        res.status = 404;
        res.set_content("404 Not Found", "text/plain");
    }
}