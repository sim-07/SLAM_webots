#include <string.h>
#include <math.h>
#include <float.h>
#include <set>

#include "Navigator.h"
#include <stack>
#include <map>
#include <iostream>
#include <algorithm>

using namespace std;

Navigator::Navigator()
{
    _currPos = {0, 0};
    _currDir = 0;
}

double Navigator::getDir()
{
    return _currDir;
}

void Navigator::updateGps(const double *gpsValues, const double *compassValues) 
{
    if (std::isnan(gpsValues[0]) || std::isnan(compassValues[0])) return;

    const double cellSizeMetres = (double)CELL_CM / 100.0;

    double rawCompassRad = -std::atan2(compassValues[0], compassValues[2]);

    if (!_isSensorsCalibrated) 
    {
        _gpsOffsetX = gpsValues[0];
        _gpsOffsetY = gpsValues[1];

        std::cout << "[CALIBRATION] GPS Offset X (metri): " << _gpsOffsetX 
                  << " | Y (metri): " << _gpsOffsetY << std::endl;

         // Inizialmente il robot potrebbe non essere a 0 rad di webots, quindi calcolo l'offset
        _compassOffset = -rawCompassRad; // Angolo iniziale nella mia logica è 0, quindi metto negativo (0-rawCompassRad)

        std::cout << "[COMPASS RAW] X: " << compassValues[0] 
          << " | Y: " << compassValues[1] 
          << " | Z: " << compassValues[2] << std::endl;

        std::cout << "[CALIBRATION] Compass Offset (rad): " << _compassOffset << std::endl;

        _isSensorsCalibrated = true;
    }

    double relativeXMetres = gpsValues[0] - _gpsOffsetX;
    double relativeYMetres = gpsValues[1] - _gpsOffsetY;

    _currXWebots = (int16_t)std::round(relativeXMetres / cellSizeMetres);
    _currYWebots = (int16_t)std::round(-relativeYMetres / cellSizeMetres);

    
    double correctedCompassRad = rawCompassRad + _compassOffset;
    
    _currDirCompass = std::atan2(std::sin(correctedCompassRad), std::cos(correctedCompassRad));
}

void Navigator::setDir(double rad)
{

    std::cout << "_currDirCompass WETBOTS: " << _currDirCompass << std::endl;

    std::cout << "_currDir in nav set to: " << rad << std::endl;
    _currDir = rad;
}

void Navigator::setCurrPos(int16_t x, int16_t y)
{
    std::cout << "Current position in WEBOTS set to: " << _currXWebots << ":" << _currYWebots << std::endl;

    std::cout << "Current position in nav set to: " << x << ":" << y << std::endl;
    _currPos = {x, y};
}

bool Navigator::isFree(int16_t x, int16_t y)
{
    Pos p = getChunkPos(x, y);

    auto it = _map.find(p);

    if (it == _map.end())
    {
        return false;
    }

    int16_t cellIndex = getPosIndex(x, y);
    bool free = it->second.cells[cellIndex] > DEFAULT_VAL;

    return free;
}

const std::map<Pos, Chunk> &Navigator::getMap() const
{
    return _map;
}

Pos Navigator::getChunkPos(int16_t x, int16_t y)
{
    return {(int16_t)(x >> 4), (int16_t)(y >> 4)}; // / 16 e arrotondato per difetto, mantenendo il segno
}

int16_t Navigator::getPosIndex(int16_t x, int16_t y)
{
    int16_t posCellX = x & 0x0F; // % 16 sempre positivo
    int16_t posCellY = y & 0x0F;

    return (posCellY * CHUNK_WIDTH) + posCellX;
}

void Navigator::sculpt(int16_t targetX, int16_t targetY, SensorType st)
{

    int v = 0;
    switch (st)
    {
    case ULTRASONIC:
        v = ULTRASONIC_A;
        break;

    case LASER:
        v = LASER_A;
        break;

    default:
        break;
    }

    createBlanks(targetX, targetY);

    int p = 2;

    float dis = (calcDistanceBetween(getPos(), {targetX, targetY}) / 10);
    if (dis > p)
    {
        int xMax = targetX + p;
        int xMin = targetX - p;
        int yMax = targetY + p;
        int yMin = targetY - p;

        for (int i = xMin; i <= xMax; i++)
        {
            for (int j = yMin; j <= yMax; j++)
            {
                Pos cPos = getChunkPos(i, j);
                int16_t cellIndex = getPosIndex(i, j);

                Chunk &currChunk = _map[cPos];

                if (currChunk.cells[cellIndex] - v <= 0)
                {
                    currChunk.cells[cellIndex] = 0;
                }
                else if (currChunk.cells[cellIndex] > THRESHOLD_OBSTACLE)
                {
                    currChunk.cells[cellIndex] -= v;
                }
            }
        }
    }
}

Route Navigator::calcRoute(int16_t dest_x, int16_t dest_y)
{
    std::cout << "Inside calcRoute in nav for: " << dest_x << ":" << dest_y << std::endl;

    Pos destPos = {dest_x, dest_y};

    return aStar(_currPos, destPos);
}

void Navigator::createBlanks(int16_t targetX, int16_t targetY)
{

    Pos chunkCurrPos = getChunkPos(getPos().x, getPos().y);
    Pos chunkDestPos = getChunkPos(targetX, targetY);
    int16_t cellIndex = getPosIndex(targetX, targetY);

    int16_t x0 = getPos().x;
    int16_t y0 = getPos().y;
    int16_t x1 = targetX;
    int16_t y1 = targetY;

    // delta x e y
    int16_t dX = std::abs(x1 - x0);
    int16_t dY = -std::abs(y1 - y0);

    if (dX > 1000 || dY < -1000)
    {
        std::cout << "Map too big " << std::endl;
        return;
    }

    // direzione destinazione
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;

    // err serve per decidere in quale direzione muoversi
    int16_t err = dX + dY;

    Pos lastPos = {0, 0};
    Chunk *currChunk = nullptr;

    while (true)
    {
        // esco prima della destinazione, non voglio resettare l'ultima cella
        if (x0 == x1 && y0 == y1)
            break;

        // int16_t pX[] = {-1, 1, 0, 0, 0, -1, -1, 1, 1};
        // int16_t pY[] = {0, 0, 1, -1, 0, 1, -1, 1, -1};
        // // Abbasso il valore celle celle in linea retta con la destinazione e quelle adiacenti ad esse
        // for (int i = 0; i < 9; i++)
        // {
        //     Pos cPos = getChunkPos(x0 + pX[i], y0 + pY[i]);
        //     if (currChunk == nullptr || cPos.x != lastPos.x || cPos.y != lastPos.y)
        //     {
        //         // riprendo il chunk dal map solo se è cambiato o è nullptr
        //         currChunk = &_map[cPos];
        //         lastPos = cPos;
        //     }

        //     int16_t cellIndex = getPosIndex(x0 + pX[i], y0 + pY[i]);
        //     if (cellIndex < CHUNK_DIM && currChunk->cells[cellIndex] <= DEFAULT_VAL)
        //     {
        //         if (currChunk->cells[cellIndex] + BLANK_A > 255)
        //         {
        //             currChunk->cells[cellIndex] = 255;
        //         }
        //         else if (currChunk->cells[cellIndex] > THRESHOLD_OBSTACLE)
        //         {
        //             currChunk->cells[cellIndex] += BLANK_A;
        //         }
        //     }
        // }

        Pos cPos = getChunkPos(x0, y0);
        if (currChunk == nullptr || cPos.x != lastPos.x || cPos.y != lastPos.y)
        {
            // riprendo il chunk dal map solo se è cambiato o è nullptr
            currChunk = &_map[cPos];
            lastPos = cPos;
        }

        int16_t cellIndex = getPosIndex(x0, y0);
        if (cellIndex < CHUNK_DIM && currChunk->cells[cellIndex] <= DEFAULT_VAL)
        {
            if (currChunk->cells[cellIndex] + BLANK_A > 255)
            {
                currChunk->cells[cellIndex] = 255;
            }
            else if (currChunk->cells[cellIndex] > THRESHOLD_OBSTACLE)
            {
                currChunk->cells[cellIndex] += BLANK_A;
            }
        }

        int e2 = 2 * err;

        if (e2 >= dY)
        {
            err += dY;
            x0 += sx;
        }

        if (e2 <= dX)
        {
            err += dX;
            y0 += sy;
        }
    }
}

///////////////////////// A* /////////////////////////

bool isDestination(int16_t row, int16_t col, Pos dest)
{
    if (row == dest.x && col == dest.y)
        return true;
    else
        return false;
}

int Navigator::calcDistanceBetween(Pos start, Pos dest)
{
    // return (sqrt(
    //     (start.x - dest.x) * (start.x - dest.x) + (start.y - dest.y) * (start.y - dest.y)));

    int dX = std::abs(start.x - dest.x);
    int dY = std::abs(start.y - dest.y);

    if (dX > dY)
    {
        return 14 * dY + 10 * (dX - dY);
    }
    else
    {
        return 14 * dX + 10 * (dY - dX);
    }
}

Route tracePath(std::map<Pos, Node> &cellDetails, Pos dest)
{
    Route r;

    r.numSteps = 0;

    int16_t row = dest.x;
    int16_t col = dest.y;

    while (true)
    {
        auto it = cellDetails.find({row, col});

        if (it == cellDetails.end())
            break;

        Pos p = {row, col};
        r.route.push_back(p);
        r.numSteps++;

        if (it->second.parent_i == row && it->second.parent_j == col) // la cella iniziale è l'unica ad avere i parents con le stesse proprie coordinate
        {
            break;
        }

        row = it->second.parent_i;
        col = it->second.parent_j;
    }

    std::reverse(r.route.begin(), r.route.end());

    return r;
}

const int MAX_V = 65535;
Route Navigator::aStar(Pos start, Pos goal)
{

    // cella di partenza
    int16_t currRow = start.x;
    int16_t currCol = start.y;

    if (start.x == goal.x && start.y == goal.y)
    {
        std::cout << "Return empty route, goal == start in aStar" << std::endl;
        return Route();
    }

    std::map<Pos, Node> cellDetails;                 // informazioni sulle celle visitate (coordinate, g, f, h, parent)
    set<pair<int, pair<int16_t, int16_t>>> openList; // celle da esplorare ordinate in base alla f
    std::set<Pos> closedList;                        // celle già esplorate

    openList.insert({0, {currRow, currCol}}); // fNew, {x, y}
    cellDetails[{currRow, currCol}] = {currRow, currCol, 0, 0, 0};

    int8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!openList.empty())
    {
        auto p = *openList.begin();
        openList.erase(openList.begin());

        currRow = p.second.first;  // x
        currCol = p.second.second; // y
        closedList.insert({currRow, currCol});

        for (int k = 0; k < 8; k++)
        {
            // vengono controllate tutte le 8 direzioni
            int16_t nRow = currRow + dX[k];
            int16_t nCol = currCol + dY[k];

            auto it = cellDetails.find({nRow, nCol});

            if (isDestination(nRow, nCol, {goal.x, goal.y}))
            {
                // se la prossima cella (nRow e nCol) è destinazione, metto i parent della prossima cella con i valori row e col della cella corrente e return
                cellDetails[{nRow, nCol}].parent_i = currRow;
                cellDetails[{nRow, nCol}].parent_j = currCol;

                return tracePath(cellDetails, {nRow, nCol});
            }

            if (!closedList.count({nRow, nCol}) && isFree(nRow, nCol))
            {
                // se non è la meta controllo che non sia già stato visto e che non sia un ostacolo
                int cost = (k < 4) ? 10 : 14; // se è diagonale costerà 1.41, gli ultimi 4 k sono diagonali
                int gNew = cellDetails[{currRow, currCol}].g + cost;
                int hNew = calcDistanceBetween({nRow, nCol}, {goal.x, goal.y}) * 10;
                int fNew = gNew + hNew;

                if (it == cellDetails.end() || it->second.f > fNew)
                {
                    cellDetails[{nRow, nCol}] = {
                        currRow,
                        currCol,
                        fNew,
                        gNew,
                        hNew};

                    openList.insert({fNew, {nRow, nCol}}); // aggiungo il prossimo valore alla lista delle celle da esplorare
                }
            }
        }
    }

    return Route();
}