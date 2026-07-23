#include <cmath>
#include <algorithm>

#include "Explorer.h"
#include "Navigator.h"
#include <iostream>

void Explorer::init(Navigator *n, RobotMovements *r, ServoMotor *s, LaserSensor *l, Ultrasonic *u)
{
    std::cout << "Init explorer in Webots" << std::endl;

    if (n == nullptr)
        std::cout << "Navigator (*n) e' nullptr" << std::endl;
    if (r == nullptr)
        std::cout << "RobotMovements (*r) e' nullptr" << std::endl;
    if (s == nullptr)
        std::cout << "ServoMotor (*s) e' nullptr" << std::endl;
    if (l == nullptr)
        std::cout << "LaserSensor (*l) e' nullptr" << std::endl;
    if (u == nullptr)
        std::cout << "Ultrasonic (*u) e' nullptr" << std::endl;

    _nav = n;
    _rb = r;
    _servo = s;
    _laser = l;
    _ultrasonic = u;

    if (_servo != nullptr)
    {
        _currScanPoint = _servo->MIN_ANGLE;
    }
    else
    {
        std::cout << "_servo e' nullptr" << std::endl;
        _currScanPoint = -1.4835;
    }

    _delay = 0;
    _currDelayState = INITSTATE;
}

void Explorer::setCurrentState(ExpState currState)
{
    std::cout << "Current state explorer: " << currState << std::endl;

    _currentState = currState;
}

void Explorer::update()
{

    if (_delay > 0)
    {
        _delay--;
        return;
    }
    else if (_currDelayState == AWAITING)
    {
        _currDelayState = TIMEPASSED;
    }

    switch (_currentState)
    {
    case START_EXPLORE:
        std::cout << "Inside switch branch START_EXPLORE" << std::endl;

        if (!_nav)
        {
            std::cout << "PROBLEM with _nav" << std::endl;
        }

        _firstPos = _nav->getPos();
        setCurrentState(SCAN);
        break;

    case SCAN:
        scan();
        break;

    case MOVE_TO_FRONTIER:
    {

        _attemptsFindBorders = 0;

        while (true)
        {
            if (_bordersToExplore.empty())
            {
                std::cout << "_bordersToExplore empty, finding other borders..." << std::endl;

                findBorder();

                if (_bordersToExplore.empty())
                {
                    std::cout << "No frontier left" << std::endl;
                    break;
                }
            }

            Pos closestBorder = findClosestBorder(_nav->getPos());
            deleteBorder(closestBorder);

            // if (closestBorder.x == _nav->getPos().x && closestBorder.y == _nav->getPos().y)
            // {
            //     std::cout << "closestBorder.x == _nav->getPos().x && closestBorder.y == _nav->getPos().y" << std::endl;
            //     Route r;
            //     r.numSteps = 1;
            //     r.turnAngle = 3.14159;

            //     _rb->setRoute(r);
            //     _rb->setCurrentState(FOLLOWING);
            //     setCurrentState(FOLLOWING_F);
            //     return;
            // }

            std::cout << "Calculating route for " << closestBorder.x << ":" << closestBorder.y << std::endl;
            Route routeFrontier = _nav->calcRoute({closestBorder.x, closestBorder.y});
            std::cout << "routeFrontier.numSteps: " << routeFrontier.numSteps << std::endl;

            if (routeFrontier.numSteps > 0)
            {
                //////////////////////
                std::cout << "Path: ";
                for (size_t i = 0; i < routeFrontier.route.size(); ++i)
                {
                    std::cout << "(" << routeFrontier.route[i].x << "," << routeFrontier.route[i].y << ")";

                    if (i < routeFrontier.route.size() - 1)
                    {
                        std::cout << " -> ";
                    }
                }
                std::cout << std::endl;
                //////////////////////

                _rb->setRoute(routeFrontier);
                _rb->setCurrentState(FOLLOWING);
                setCurrentState(FOLLOWING_F);
                return;
            }
            else if (_bordersToExplore.size() == 0 && _attemptsFindBorders > 3)
            {
                std::cout << "_bordersToExplore.size() == 0" << std::endl;
                _attemptsFindBorders = 0;
                break;
            } else {
                _attemptsFindBorders++;
            }
        }

        std::cout << "No frontier left, return home: _firstPos.x: " << _firstPos.x << " _firstPos.y: " << _firstPos.y << std::endl;

        Route rHome = _nav->calcRoute({_firstPos.x, _firstPos.y});
        if (rHome.numSteps == -1)
        {
            std::cout << "Can't go home" << std::endl;
        }
        setCurrentState(COMPLETED);
        _rb->setRoute(rHome);
        _rb->setCurrentState(FOLLOWING);
        return;
    }
    break;

    case FOLLOWING_F:
        if (_rb->getCurrentState() == COMPLETED_ROUTE)
        {
            _bordersExplored++;
            setCurrentState(SCAN);
        }
        break;

    case COMPLETED:
        break;

    default:
        break;
    }

    // TODO gestire rilevamento ostacoli
}

double Explorer::normAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

Pos Explorer::calcCoordinates(Pos currPos, float dis, double totAngle)
{
    float disX = floor((cos(totAngle) * dis) / _nav->CELL_CM + (float)currPos.x);
    float disY = floor((sin(totAngle) * dis) / _nav->CELL_CM + (float)currPos.y);

    return {static_cast<int16_t>(disX), static_cast<int16_t>(disY)};
}

void Explorer::scan()
{
    if (_currScanPoint <= _servo->MAX_ANGLE && _currScanPoint >= _servo->MIN_ANGLE)
    {
        //std::cout << "Scanning angle: " << _currScanPoint << std::endl;
        _servo->moveToAngleFast(_currScanPoint);

        if (_currDelayState == INITSTATE)
        {
            _delay = 6;
            _currDelayState = AWAITING;
            return;
        }
        else if (_currDelayState == TIMEPASSED)
        {
            searchObstacles();

            _currScanPoint += (SCAN_PRECISION * _dirServo);
            _currDelayState = INITSTATE;
        }
    }
    else
    {
        _currDelayState = INITSTATE;
        // Finito di girare, torno al minimo o al massimo a seconda di chi è più vicino
        if (std::abs(_servo->getAngle() - _servo->MIN_ANGLE) < std::abs(_servo->getAngle() - _servo->MAX_ANGLE))
        {
            _currScanPoint = _servo->MIN_ANGLE;
            _dirServo = 1;
            setCurrentState(MOVE_TO_FRONTIER);
        }
        else
        {
            _currScanPoint = _servo->MAX_ANGLE;
            _dirServo = -1;
            setCurrentState(MOVE_TO_FRONTIER);
        }
    }

    return;
}

void Explorer::searchObstacles()
{
    double laserDis = -1;
    double ultrasonicDis = -1;
    double servoAngle = _servo->getAngle();
    double robotAngle = _nav->getDir();
    double totAngle = normAngle(servoAngle + robotAngle);

    Pos p = _nav->getPos();

    if (_laser->isReady())
    {
        laserDis = _laser->getDistance();

        if (laserDis > 0)
        {
            Pos cLaser = calcCoordinates(p, laserDis, totAngle);
            _nav->sculpt(cLaser.x, cLaser.y, Navigator::LASER);
        }
    }

    ultrasonicDis = _ultrasonic->getDistance();

    if (ultrasonicDis > 0)
    {
        Pos cUltrasonic = calcCoordinates(p, ultrasonicDis, totAngle);
        _nav->sculpt(cUltrasonic.x, cUltrasonic.y, Navigator::ULTRASONIC);
    }
}

void Explorer::findBorder()
{
    // Parte dalle coordinate del robot correnti e si espande in tutte le 8 celle adiacenti. Quindi con un for controllo singolarmente ogni cella adiacente alla cella correntemente analizzata. Se è un ostacolo o l'ho già analizzata la ignoro e la metto in closedlist. Se è libera la metto al fondo di openlist (da cui poi prendo e rimuovo il primo elemento come cella corrente al prossimo ciclo). Se è sconosciuta allora ho trovato una possibile frontiera e analizzo le celle adiacenti ad essa per capire se è rumore o è davvero un possibile passaggio. Se ci sono almeno altre 3 celle sconosciute adiacenti (3 sulle 8 adiacenti) allora calcolo la route fino a quel punto e restituisco. Se sono meno di 3 o se il percorso non è utilizzabile continuo a cercare

    // TODO meglio memorizzare tutte le possibili frontiere in una volta, senza dover rianalizzare tutto. Magari mettere un limite al numero di frontiere trovate

    int16_t maxExp = 5000; // Massime celle analizzate
    int16_t counterMax = 0;

    int16_t currX = _nav->getPos().x;
    int16_t currY = _nav->getPos().y;

    std::cout << "Currently in findBorder(): currX: " << currX << " currY: " << currY << std::endl;

    std::deque<Pos> mapOpenList;
    std::set<Pos> mapClosedList;

    int8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    mapOpenList.push_front({currX, currY}); // Inserisco come primo valore la posizione corrente
    // mapClosedList.insert({currX, currY});

    // if (mapOpenList.empty())
    // {
    //     std::cout << "mapOpenList empty" << std::endl;
    // }

    while (!mapOpenList.empty())
    {
        const auto &map = _nav->getMap();

        if (map.empty())
        {
            std::cout << "Map empty" << std::endl;
            return;
        }

        if (counterMax > maxExp)
        {
            std::cout << "Countermax exceeded" << std::endl;
            return;
        }
        counterMax++;

        Pos currAnalyzedCell = mapOpenList.front();
        mapOpenList.pop_front();

        for (uint8_t i = 0; i < 8; i++)
        {
            int16_t nX = currAnalyzedCell.x + dX[i]; // Coordinate da analizzare
            int16_t nY = currAnalyzedCell.y + dY[i];

            if (mapClosedList.find({nX, nY}) != mapClosedList.end())
            { // Se l'ho già vista e scartata passo oltre
                continue;
            }

            // std::cout << "Currently analyzing in findBorder(): " << nX << ":" << nY << std::endl;

            Pos chunkPos = _nav->getChunkPos({nX, nY});
            int16_t index = _nav->getPosIndex({nX, nY});

            auto it = map.find(chunkPos);

            Pos foundBorder;
            bool isFrontierFound = false;

            // Se il chunk esiste e trovo punti ignoti all'interno è una possibile frontiera. Se il chunk non esiste è sicuramente una frontiera
            if (it != map.end())
            {
                const Chunk &chunk = it->second; // Chunk in cui si trova la cella analizzata

                if (chunk.cells[index] > _nav->THRESHOLD_OBSTACLE && chunk.cells[index] != DEFAULT_VAL)
                { // Se è libero va in openlist
                    // std::cout << "It's free" << std::endl;
                    mapOpenList.push_back({nX, nY});
                }
                else if (chunk.cells[index] == DEFAULT_VAL)
                { // Cella sconosciuta, trovata possibile frontiera
                    //std::cout << "It's unknown" << std::endl;
                    uint8_t countFrontier = 0;
                    for (uint8_t j = 0; j < 8; j++)
                    { // Analizzo celle adiacenti a quella sconosciuta
                        int16_t nXC = nX + dX[j];
                        int16_t nYC = nY + dY[j];

                        Pos chunkPosC = _nav->getChunkPos({nXC, nYC});
                        int16_t indexC = _nav->getPosIndex({nXC, nYC});

                        auto itC = map.find(chunkPosC);
                        if (itC != map.end())
                        {
                            const Chunk &chunkC = itC->second;

                            if (chunkC.cells[indexC] == DEFAULT_VAL)
                            {
                                countFrontier++; // Trovata una'altra frontiera adiacente
                            }
                        }
                    }

                    if (countFrontier >= 3)
                    {
                        // Se ci sono almeno 3 celle sconosciute adiacenti è una frontiera, restituisco il percorso
                        //std::cout << "Found frontier: " << nX << ":" << nY << std::endl;
                        isFrontierFound = true;
                        foundBorder = {currAnalyzedCell.x, currAnalyzedCell.y};
                    }
                }

                mapClosedList.insert({nX, nY});
            }
            else
            {
                // il chunk della cella analizzata è sconosciuto, sicuramente è una frontiera
                std::cout << "Chunk does NOT exists, found sure frontier: " << currAnalyzedCell.x << ":" << currAnalyzedCell.y << std::endl;

                mapClosedList.insert({currAnalyzedCell.x, currAnalyzedCell.y});

                foundBorder = {currAnalyzedCell.x, currAnalyzedCell.y};
                isFrontierFound = true;
            }

            if (isFrontierFound)
            {
                if (!_bordersToExplore.empty())
                {
                    Pos closBorder = findClosestBorder(foundBorder);
                    int distance = std::floor(_nav->calcDistanceBetween(closBorder, foundBorder) / 10);
                    if (distance < 4)
                    {
                        // std::cout << "Distance too short (" << distance
                        //           << ") between last border (" << closBorder.x << ":" << closBorder.y
                        //           << ") and new border (" << foundBorder.x << ":" << foundBorder.y << ")"
                        //           << std::endl;
                        continue;
                    }
                }

                std::cout << "Inserting in _bordesToexplore: " << foundBorder.x << ":" << foundBorder.y << std::endl;
                _bordersToExplore.push_back(foundBorder);

                if (_bordersToExplore.size() >= MAX_BORDERS_FOUND)
                {
                    std::cout << "_bordersToExplore.size() >= MAX_BORDERS_FOUND" << std::endl;
                    return;
                }
            }
        }
    }

    return;
}

Pos Explorer::findClosestBorder(Pos currPos)
{

    if (_bordersToExplore.empty())
    {
        return {32767, 32767};
    }

    Pos closestBorder = _bordersToExplore.front();

    for (int i = 1; i < _bordersToExplore.size(); i++)
    {
        Pos &e = _bordersToExplore[i];

        if (std::abs(currPos.x - e.x) <= std::abs(currPos.x - closestBorder.x) &&
            std::abs(currPos.y - e.y) <= std::abs(currPos.y - closestBorder.y))
        {
            closestBorder = e;
        }
    }

    return closestBorder;
}

void Explorer::deleteBorder(Pos closestBorder)
{
    auto it = std::find_if(_bordersToExplore.begin(), _bordersToExplore.end(), [closestBorder](const Pos &p)
                           { return p.x == closestBorder.x && p.y == closestBorder.y; });

    if (it != _bordersToExplore.end())
    {
        _bordersToExplore.erase(it);
    }
}